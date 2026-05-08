#include "FreeRTOS.h"
#include "bsp_dwt.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "bsp_board.h"
#include "app_log.h"
#include "topic_bus.h"
#include "app_topics.h"
#include "msg_ins.h"
#include "user_def.h"

#define APP_LOG_BUFFER_SIZE 128U
#define APP_LOG_TX_PERIOD_MS 100U
#define APP_LOG_RX_SNAPSHOT_SIZE 16U

#if USER_UART6_SERIAL_STREAM_RX_TEST_ENABLE
#define APP_LOG_PRINT_UART_RX_DIAG_ENABLE 0U
#else
#define APP_LOG_PRINT_UART_RX_DIAG_ENABLE 1U
#endif

typedef struct app_log
{
    bspUARTInstance_t *print_uart_;
    moduleTopicSubscription_t ins_subscriber_;
    SemaphoreHandle_t ins_wait_sem_;
    msgINS_t ins_message_;
    char tx_line_[APP_LOG_BUFFER_SIZE];

    uint32_t wait_timeout_count_;
    uint32_t copy_error_count_;
    uint32_t tx_submit_count_;
    uint32_t tx_submit_ok_count_;
    uint32_t tx_busy_drop_count_;
    uint32_t tx_error_count_;
    uint32_t tx_cplt_count_;
    uint32_t tx_uart_error_count_;
    uint32_t tx_submit_last_status_;

    uint32_t print_rx_event_count_;
    uint32_t print_rx_total_bytes_;
    uint32_t print_rx_last_event_type_;
    uint32_t print_rx_last_span_bytes_;
    uint8_t print_rx_last_bytes_[APP_LOG_RX_SNAPSHOT_SIZE];
    uint8_t print_rx_last_bytes_count_;

    uint32_t throttle_skip_count_;
    uint32_t format_error_count_;
    uint32_t last_tx_tick_ms_;
} appLog_t;

static appLog_t app_log_;
StaticSemaphore_t ins_wait_semphr_;

static int32_t appLogFloatToCentiDeg(float value_deg)
{
    float scaled_value = value_deg * 100.0f;

    if (scaled_value >= 0.0f) {
        return (int32_t)(scaled_value + 0.5f);
    }

    return (int32_t)(scaled_value - 0.5f);
}

static uint32_t appLogAbsU32(int32_t value)
{
    if (value < 0) {
        return (uint32_t)(-value);
    }

    return (uint32_t)value;
}

static int appLogFormatINSLine(char *buffer, size_t buffer_size, const msgINS_t *message)
{
    uint32_t timestamp_ms = 0U;
    int32_t roll_centi_deg = 0;
    int32_t pitch_centi_deg = 0;
    int32_t yaw_centi_deg = 0;
    uint32_t roll_abs_centi_deg = 0U;
    uint32_t pitch_abs_centi_deg = 0U;
    uint32_t yaw_abs_centi_deg = 0U;

    if (buffer == NULL || message == NULL || buffer_size == 0U) {
        return -1;
    }

    uint64_t timestamp_us = bspDWTGetAbsTimeUs();
    timestamp_ms = (uint32_t)(timestamp_us / 1000ULL);
    roll_centi_deg = appLogFloatToCentiDeg(message->euler_zyx_deg_[2]);
    pitch_centi_deg = appLogFloatToCentiDeg(message->euler_zyx_deg_[1]);
    yaw_centi_deg = appLogFloatToCentiDeg(message->euler_zyx_deg_[0]);

    roll_abs_centi_deg = appLogAbsU32(roll_centi_deg);
    pitch_abs_centi_deg = appLogAbsU32(pitch_centi_deg);
    yaw_abs_centi_deg = appLogAbsU32(yaw_centi_deg);

    return snprintf(buffer,
                    buffer_size,
                    "[INFO:%lu]r=%s%lu.%02lu p=%s%lu.%02lu y=%s%lu.%02lu\r\n",
                    (unsigned long)timestamp_ms,
                    (roll_centi_deg < 0) ? "-" : "",
                    (unsigned long)(roll_abs_centi_deg / 100U),
                    (unsigned long)(roll_abs_centi_deg % 100U),
                    (pitch_centi_deg < 0) ? "-" : "",
                    (unsigned long)(pitch_abs_centi_deg / 100U),
                    (unsigned long)(pitch_abs_centi_deg % 100U),
                    (yaw_centi_deg < 0) ? "-" : "",
                    (unsigned long)(yaw_abs_centi_deg / 100U),
                    (unsigned long)(yaw_abs_centi_deg % 100U));
}

#if APP_LOG_PRINT_UART_RX_DIAG_ENABLE
static void appLogHandlePrintRxEvent(void *owner_ptr, const bspUARTRxEventContext_t *rx_context)
{
    appLog_t *instance = (appLog_t *)owner_ptr;
    uint16_t total_bytes = 0U;
    uint8_t snapshot_count = 0U;

    if (instance == NULL || rx_context->rx_buffer_ptr_ == NULL || rx_context->rx_buffer_size_ == 0U) {
        return;
    }

    instance->print_rx_event_count_++;
    instance->print_rx_last_event_type_ = (uint32_t)rx_context->rx_event_;

    if (rx_context->rx_data_start_index_ < rx_context->rx_data_end_pos_) {
        total_bytes = (uint16_t)(rx_context->rx_data_end_pos_ - rx_context->rx_data_start_index_);
        snapshot_count = (uint8_t)((total_bytes < APP_LOG_RX_SNAPSHOT_SIZE) ? total_bytes : APP_LOG_RX_SNAPSHOT_SIZE);
        memcpy(instance->print_rx_last_bytes_,
               &rx_context->rx_buffer_ptr_[rx_context->rx_data_start_index_],
               snapshot_count);
    } else if (rx_context->rx_data_start_index_ > rx_context->rx_data_end_pos_) {
        uint16_t first_part_bytes = (uint16_t)(rx_context->rx_buffer_size_ - rx_context->rx_data_start_index_);
        uint16_t second_part_bytes = rx_context->rx_data_end_pos_;

        total_bytes = (uint16_t)(first_part_bytes + second_part_bytes);
        snapshot_count = (uint8_t)((total_bytes < APP_LOG_RX_SNAPSHOT_SIZE) ? total_bytes : APP_LOG_RX_SNAPSHOT_SIZE);

        if (snapshot_count <= first_part_bytes) {
            memcpy(instance->print_rx_last_bytes_,
                   &rx_context->rx_buffer_ptr_[rx_context->rx_data_start_index_],
                   snapshot_count);
        } else {
            uint8_t first_copy_bytes = (uint8_t)first_part_bytes;
            uint8_t second_copy_bytes = (uint8_t)(snapshot_count - first_copy_bytes);

            memcpy(instance->print_rx_last_bytes_,
                   &rx_context->rx_buffer_ptr_[rx_context->rx_data_start_index_],
                   first_copy_bytes);
            memcpy(&instance->print_rx_last_bytes_[first_copy_bytes],
                   &rx_context->rx_buffer_ptr_[0],
                   second_copy_bytes);
        }
    } else {
        total_bytes = 0U;
        snapshot_count = 0U;
    }

    instance->print_rx_total_bytes_ += total_bytes;
    instance->print_rx_last_span_bytes_ = total_bytes;
    instance->print_rx_last_bytes_count_ = snapshot_count;
}
#endif

static void appLogHandleTxCplt(void *owner_ptr)
{
    appLog_t *instance = (appLog_t *)owner_ptr;

    if (instance == NULL) {
        return;
    }

    instance->tx_cplt_count_++;
}

static void appLogHandleUARTError(void *owner_ptr)
{
    appLog_t *instance = (appLog_t *)owner_ptr;

    if (instance == NULL) {
        return;
    }

    instance->tx_uart_error_count_++;
}

static bool appLogInit(void)
{
    moduleTopic_t *ins_topic = NULL;

    memset(&app_log_, 0, sizeof(appLog_t));

    app_log_.print_uart_ = bspBoardGetUARTInstance(BSP_UART_PRINT);
    if (app_log_.print_uart_ == NULL) {
        return false;
    }

    bspUARTTxCpltCallbackRegister(app_log_.print_uart_, (void *)&app_log_, appLogHandleTxCplt);
    bspUARTErrorCallbackRegister(app_log_.print_uart_, (void *)&app_log_, appLogHandleUARTError);

#if APP_LOG_PRINT_UART_RX_DIAG_ENABLE
    bspUARTRxEventCallbackRegister(app_log_.print_uart_, (void *)&app_log_, appLogHandlePrintRxEvent);
    if (bspUARTRxStart(app_log_.print_uart_) != BSP_UART_OK) {
        return false;
    }
#endif

    app_log_.ins_wait_sem_ = xSemaphoreCreateBinaryStatic(&ins_wait_semphr_);
    if (app_log_.ins_wait_sem_ == NULL) {
        return false;
    }

    ins_topic = appTopicsGet(APP_TOPICS_INS);
    if (ins_topic == NULL) {
        return false;
    }

    app_log_.ins_subscriber_.topic_ = ins_topic;
    app_log_.ins_subscriber_.wait_backend_ = MODULE_TOPIC_WAIT_SEMAPHORE;
    app_log_.ins_subscriber_.wait_handle_.wait_sem_ = app_log_.ins_wait_sem_;

    if (moduleTopicBusSubscribe(ins_topic, &app_log_.ins_subscriber_) == false) {
        return false;
    }

    return true;
}

void appLogTaskEntry(void *argument)
{
    (void)argument;

    while (appLogInit() == false) {
        osDelay(pdMS_TO_TICKS(APP_LOG_TRY_INIT_DELAY_MS));
    }

    for (;;) {
        uint32_t now_tick_ms = 0U;
        int log_line_length = 0;
        bspUARTStatus_e tx_status;

        if (moduleTopicBusWait(&app_log_.ins_subscriber_, pdMS_TO_TICKS(1000U)) == false) {
            // app_log_.wait_timeout_count_++;
            // continue;
        }

        if (moduleTopicBusCopy(&app_log_.ins_subscriber_, &app_log_.ins_message_) == false) {
            // app_log_.copy_error_count_++;
            // continue;
        }

        now_tick_ms = osKernelGetTickCount();
        if ((now_tick_ms - app_log_.last_tx_tick_ms_) < APP_LOG_TX_PERIOD_MS) {
            app_log_.throttle_skip_count_++;
            continue;
        }

        if (bspUARTTxIsBusy(app_log_.print_uart_) == true) {
            app_log_.tx_busy_drop_count_++;
            continue;
        }

        log_line_length = appLogFormatINSLine(app_log_.tx_line_,
                                              sizeof(app_log_.tx_line_),
                                              &app_log_.ins_message_);
        if (log_line_length <= 0 || log_line_length >= (int)sizeof(app_log_.tx_line_)) {
            app_log_.format_error_count_++;
            continue;
        }

        app_log_.tx_submit_count_++;
        tx_status = bspUARTTransimt(app_log_.print_uart_,
                                    (uint8_t *)app_log_.tx_line_,
                                    (uint16_t)log_line_length);
        app_log_.tx_submit_last_status_ = (uint32_t)tx_status;

        if (tx_status == BSP_UART_OK) {
            app_log_.tx_submit_ok_count_++;
            app_log_.last_tx_tick_ms_ = now_tick_ms;
        } else if (tx_status == BSP_UART_BUSY) {
            app_log_.tx_busy_drop_count_++;
        } else {
            app_log_.tx_error_count_++;
        }
    }
}

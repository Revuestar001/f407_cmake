#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_uart6_serial_stream_rx_test.h"
#include "bsp_board.h"
#include "serial_stream.h"
#include "user_def.h"

#define APP_UART6_SERIAL_STREAM_RX_TEST_READ_BUFFER_SIZE 128U
#define APP_UART6_SERIAL_STREAM_RX_TEST_STREAM_STORAGE_SIZE (BSP_UART_RX_BUFFER_SIZE * 4U + 2U)
#define APP_UART6_SERIAL_STREAM_RX_TEST_LINE_BUFFER_SIZE 128U
#define APP_UART6_SERIAL_STREAM_RX_TEST_INIT_RETRY_DELAY_MS 100U

#if USER_UART6_SERIAL_STREAM_RX_TEST_USE_ZERO_COPY
#define APP_UART6_SERIAL_STREAM_RX_TEST_BACKEND MODULE_SERIAL_STREAM_BACKEND_ZERO_COPY
#if USER_UART6_SERIAL_STREAM_RX_TEST_USE_SEMAPHORE
#define APP_UART6_SERIAL_STREAM_RX_TEST_NOTIFY_BACKEND MODULE_SERIAL_STREAM_NOTIFY_SEMAPHORE
#else
#define APP_UART6_SERIAL_STREAM_RX_TEST_NOTIFY_BACKEND MODULE_SERIAL_STREAM_NOTIFY_TASK
#endif
#else
#define APP_UART6_SERIAL_STREAM_RX_TEST_BACKEND MODULE_SERIAL_STREAM_BACKEND_STREAM_BUFFER
#define APP_UART6_SERIAL_STREAM_RX_TEST_NOTIFY_BACKEND MODULE_SERIAL_STREAM_NOTIFY_NONE
#endif

typedef enum app_uart6_serial_stream_rx_test_init_fail
{
    APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_NONE = 0U,
    APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_UART_INSTANCE = 1U,
    APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_STREAM_BUFFER = 2U,
    APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_SEMAPHORE = 3U,
    APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_SERIAL_STREAM = 4U,
    APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_RX_START = 5U,
} appUART6SerialStreamRxTestInitFail_e;

typedef struct app_uart6_serial_stream_rx_test
{
    bspUARTInstance_t *uart_instance_;
    moduleSerialStream_t serial_stream_;

    StreamBufferHandle_t stream_buffer_handle_;
    SemaphoreHandle_t notify_semaphore_;

    uint8_t line_buffer_[APP_UART6_SERIAL_STREAM_RX_TEST_LINE_BUFFER_SIZE];
    uint32_t line_length_;
    bool line_overflow_active_;
} appUART6SerialStreamRxTest_t;

static appUART6SerialStreamRxTest_t app_uart6_serial_stream_rx_test_ = {0};
static uint8_t app_uart6_serial_stream_rx_test_stream_storage_[APP_UART6_SERIAL_STREAM_RX_TEST_STREAM_STORAGE_SIZE] = {0};
static StaticStreamBuffer_t app_uart6_serial_stream_rx_test_stream_struct_;
static StaticSemaphore_t app_uart6_serial_stream_rx_test_semaphore_struct_;

volatile appUART6SerialStreamRxTestStats_t g_app_uart6_serial_stream_rx_test_stats_ = {0};

static void appUART6SerialStreamRxTestUpdateStackUsage(void)
{
    uint32_t current_hwm_words = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

    g_app_uart6_serial_stream_rx_test_stats_.stack_hwm_words_ = current_hwm_words;
    if (g_app_uart6_serial_stream_rx_test_stats_.stack_hwm_min_words_ == 0U ||
        current_hwm_words < g_app_uart6_serial_stream_rx_test_stats_.stack_hwm_min_words_) {
        g_app_uart6_serial_stream_rx_test_stats_.stack_hwm_min_words_ = current_hwm_words;
    }
}

static void appUART6SerialStreamRxTestRefreshSerialStats(void)
{
    g_app_uart6_serial_stream_rx_test_stats_.serial_total_bytes_ =
        app_uart6_serial_stream_rx_test_.serial_stream_.rx_total_bytes_;
    g_app_uart6_serial_stream_rx_test_stats_.serial_accept_bytes_ =
        app_uart6_serial_stream_rx_test_.serial_stream_.rx_accept_bytes_;
    g_app_uart6_serial_stream_rx_test_stats_.serial_drop_count_ =
        app_uart6_serial_stream_rx_test_.serial_stream_.rx_drop_count_;
    g_app_uart6_serial_stream_rx_test_stats_.serial_drop_bytes_ =
        app_uart6_serial_stream_rx_test_.serial_stream_.rx_drop_bytes_;
}

static void appUART6SerialStreamRxTestStoreLastChunk(const uint8_t *buffer, uint32_t length)
{
    uint32_t copy_length = length;

    if (copy_length > sizeof(g_app_uart6_serial_stream_rx_test_stats_.last_chunk_)) {
        copy_length = sizeof(g_app_uart6_serial_stream_rx_test_stats_.last_chunk_);
    }

    memset((void *)g_app_uart6_serial_stream_rx_test_stats_.last_chunk_,
           0,
           sizeof(g_app_uart6_serial_stream_rx_test_stats_.last_chunk_));
    if (buffer != NULL && copy_length > 0U) {
        memcpy((void *)g_app_uart6_serial_stream_rx_test_stats_.last_chunk_, buffer, copy_length);
    }

    g_app_uart6_serial_stream_rx_test_stats_.last_chunk_length_ = length;
    g_app_uart6_serial_stream_rx_test_stats_.last_chunk_bytes_ = length;
    if (length > g_app_uart6_serial_stream_rx_test_stats_.max_chunk_bytes_) {
        g_app_uart6_serial_stream_rx_test_stats_.max_chunk_bytes_ = length;
    }
}

static void appUART6SerialStreamRxTestStoreLastLine(const uint8_t *buffer, uint32_t length)
{
    uint32_t copy_length = length;

    if (copy_length > sizeof(g_app_uart6_serial_stream_rx_test_stats_.last_line_)) {
        copy_length = sizeof(g_app_uart6_serial_stream_rx_test_stats_.last_line_);
    }

    memset((void *)g_app_uart6_serial_stream_rx_test_stats_.last_line_,
           0,
           sizeof(g_app_uart6_serial_stream_rx_test_stats_.last_line_));
    if (buffer != NULL && copy_length > 0U) {
        memcpy((void *)g_app_uart6_serial_stream_rx_test_stats_.last_line_, buffer, copy_length);
    }

    g_app_uart6_serial_stream_rx_test_stats_.last_line_length_ = length;
}

static void appUART6SerialStreamRxTestFinishLine(void)
{
    if (app_uart6_serial_stream_rx_test_.line_overflow_active_ != false) {
        app_uart6_serial_stream_rx_test_.line_overflow_active_ = false;
        app_uart6_serial_stream_rx_test_.line_length_ = 0U;
        g_app_uart6_serial_stream_rx_test_stats_.line_buffered_bytes_ = 0U;
        return;
    }

    appUART6SerialStreamRxTestStoreLastLine(app_uart6_serial_stream_rx_test_.line_buffer_,
                                            app_uart6_serial_stream_rx_test_.line_length_);
    g_app_uart6_serial_stream_rx_test_stats_.line_count_++;
    app_uart6_serial_stream_rx_test_.line_length_ = 0U;
    g_app_uart6_serial_stream_rx_test_stats_.line_buffered_bytes_ = 0U;
}

static void appUART6SerialStreamRxTestFeedByte(uint8_t rx_byte)
{
    if (rx_byte == '\r') {
        return;
    }

    if (rx_byte == '\n') {
        appUART6SerialStreamRxTestFinishLine();
        return;
    }

    if (app_uart6_serial_stream_rx_test_.line_overflow_active_ != false) {
        return;
    }

    if (app_uart6_serial_stream_rx_test_.line_length_ >=
        (sizeof(app_uart6_serial_stream_rx_test_.line_buffer_) - 1U)) {
        g_app_uart6_serial_stream_rx_test_stats_.line_overflow_count_++;
        app_uart6_serial_stream_rx_test_.line_overflow_active_ = true;
        app_uart6_serial_stream_rx_test_.line_length_ = 0U;
        g_app_uart6_serial_stream_rx_test_stats_.line_buffered_bytes_ = 0U;
        return;
    }

    app_uart6_serial_stream_rx_test_.line_buffer_[app_uart6_serial_stream_rx_test_.line_length_] = rx_byte;
    app_uart6_serial_stream_rx_test_.line_length_++;
    g_app_uart6_serial_stream_rx_test_stats_.line_buffered_bytes_ =
        app_uart6_serial_stream_rx_test_.line_length_;
}

static void appUART6SerialStreamRxTestHandleReadChunk(const uint8_t *buffer, uint32_t length)
{
    uint32_t now_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t i;

    if (buffer == NULL || length == 0U) {
        g_app_uart6_serial_stream_rx_test_stats_.read_zero_length_count_++;
        return;
    }

    if (g_app_uart6_serial_stream_rx_test_stats_.last_read_tick_ms_ != 0U) {
        uint32_t inter_gap_ms = now_tick_ms - g_app_uart6_serial_stream_rx_test_stats_.last_read_tick_ms_;

        g_app_uart6_serial_stream_rx_test_stats_.last_inter_read_gap_ms_ = inter_gap_ms;
        if (inter_gap_ms > g_app_uart6_serial_stream_rx_test_stats_.max_inter_read_gap_ms_) {
            g_app_uart6_serial_stream_rx_test_stats_.max_inter_read_gap_ms_ = inter_gap_ms;
        }
    }
    g_app_uart6_serial_stream_rx_test_stats_.last_read_tick_ms_ = now_tick_ms;

    g_app_uart6_serial_stream_rx_test_stats_.read_count_++;
    g_app_uart6_serial_stream_rx_test_stats_.read_total_bytes_ += length;
    appUART6SerialStreamRxTestStoreLastChunk(buffer, length);

    for (i = 0U; i < length; i++) {
        appUART6SerialStreamRxTestFeedByte(buffer[i]);
    }

    appUART6SerialStreamRxTestRefreshSerialStats();
}

static bool appUART6SerialStreamRxTestDrainZeroCopy(void)
{
    uint8_t read_buffer[APP_UART6_SERIAL_STREAM_RX_TEST_READ_BUFFER_SIZE] = {0};
    uint32_t read_bytes = 0U;
    bool read_anything = false;

    for (;;) {
        read_bytes = moduleSerialStreamRead(&app_uart6_serial_stream_rx_test_.serial_stream_,
                                            read_buffer,
                                            sizeof(read_buffer),
                                            0U);
        if (read_bytes == 0U) {
            break;
        }

        read_anything = true;
        appUART6SerialStreamRxTestHandleReadChunk(read_buffer, read_bytes);
    }

    return read_anything;
}

#if USER_UART6_SERIAL_STREAM_RX_TEST_USE_ZERO_COPY == 0U
static bool appUART6SerialStreamRxTestEnsureStreamBuffer(void)
{
    if (app_uart6_serial_stream_rx_test_.stream_buffer_handle_ != NULL) {
        return true;
    }

    app_uart6_serial_stream_rx_test_.stream_buffer_handle_ =
        xStreamBufferCreateStatic(sizeof(app_uart6_serial_stream_rx_test_stream_storage_) - 1U,
                                  1U,
                                  app_uart6_serial_stream_rx_test_stream_storage_,
                                  &app_uart6_serial_stream_rx_test_stream_struct_);

    return app_uart6_serial_stream_rx_test_.stream_buffer_handle_ != NULL;
}
#endif

#if USER_UART6_SERIAL_STREAM_RX_TEST_USE_ZERO_COPY && USER_UART6_SERIAL_STREAM_RX_TEST_USE_SEMAPHORE
static bool appUART6SerialStreamRxTestEnsureSemaphore(void)
{
    if (app_uart6_serial_stream_rx_test_.notify_semaphore_ != NULL) {
        return true;
    }

    app_uart6_serial_stream_rx_test_.notify_semaphore_ =
        xSemaphoreCreateBinaryStatic(&app_uart6_serial_stream_rx_test_semaphore_struct_);

    return app_uart6_serial_stream_rx_test_.notify_semaphore_ != NULL;
}
#endif

static bool appUART6SerialStreamRxTestInit(void)
{
    moduleSerialStreamConfig_t stream_config;

    memset(&app_uart6_serial_stream_rx_test_.serial_stream_,
           0,
           sizeof(app_uart6_serial_stream_rx_test_.serial_stream_));
    app_uart6_serial_stream_rx_test_.line_length_ = 0U;
    app_uart6_serial_stream_rx_test_.line_overflow_active_ = false;
    memset(app_uart6_serial_stream_rx_test_.line_buffer_,
           0,
           sizeof(app_uart6_serial_stream_rx_test_.line_buffer_));

    app_uart6_serial_stream_rx_test_.uart_instance_ = bspBoardGetUARTInstance(BSP_UART_PRINT);
    if (app_uart6_serial_stream_rx_test_.uart_instance_ == NULL) {
        g_app_uart6_serial_stream_rx_test_stats_.init_fail_reason_ =
            APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_UART_INSTANCE;
        return false;
    }

    memset(&stream_config, 0, sizeof(stream_config));
    stream_config.backend_ = APP_UART6_SERIAL_STREAM_RX_TEST_BACKEND;
    stream_config.uart_instance_ = app_uart6_serial_stream_rx_test_.uart_instance_;
    stream_config.notify_backend_ = APP_UART6_SERIAL_STREAM_RX_TEST_NOTIFY_BACKEND;

#if USER_UART6_SERIAL_STREAM_RX_TEST_USE_ZERO_COPY
#if USER_UART6_SERIAL_STREAM_RX_TEST_USE_SEMAPHORE
    if (appUART6SerialStreamRxTestEnsureSemaphore() == false) {
        g_app_uart6_serial_stream_rx_test_stats_.init_fail_reason_ =
            APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_SEMAPHORE;
        return false;
    }
    while (xSemaphoreTake(app_uart6_serial_stream_rx_test_.notify_semaphore_, 0U) == pdPASS) {
    }
    stream_config.backend_handle_.semphr_notify_handle_ = app_uart6_serial_stream_rx_test_.notify_semaphore_;
#else
    while (ulTaskNotifyTake(pdTRUE, 0U) > 0U) {
    }
    stream_config.backend_handle_.task_notify_handle_ = xTaskGetCurrentTaskHandle();
#endif
#else
    if (appUART6SerialStreamRxTestEnsureStreamBuffer() == false) {
        g_app_uart6_serial_stream_rx_test_stats_.init_fail_reason_ =
            APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_STREAM_BUFFER;
        return false;
    }
    (void)xStreamBufferReset(app_uart6_serial_stream_rx_test_.stream_buffer_handle_);
    stream_config.backend_handle_.stream_buffer_handle_ =
        app_uart6_serial_stream_rx_test_.stream_buffer_handle_;
#endif

    if (moduleSerialStreamInit(&app_uart6_serial_stream_rx_test_.serial_stream_, &stream_config) == false) {
        g_app_uart6_serial_stream_rx_test_stats_.init_fail_reason_ =
            APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_SERIAL_STREAM;
        return false;
    }

    if (bspUARTRxStart(app_uart6_serial_stream_rx_test_.uart_instance_) != BSP_UART_OK) {
        g_app_uart6_serial_stream_rx_test_stats_.init_fail_reason_ =
            APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_RX_START;
        return false;
    }

    g_app_uart6_serial_stream_rx_test_stats_.init_ok_ = 1U;
    g_app_uart6_serial_stream_rx_test_stats_.backend_ = (uint32_t)APP_UART6_SERIAL_STREAM_RX_TEST_BACKEND;
    g_app_uart6_serial_stream_rx_test_stats_.notify_backend_ =
        (uint32_t)APP_UART6_SERIAL_STREAM_RX_TEST_NOTIFY_BACKEND;
    g_app_uart6_serial_stream_rx_test_stats_.init_fail_reason_ =
        APP_UART6_SERIAL_STREAM_RX_TEST_INIT_FAIL_NONE;
    appUART6SerialStreamRxTestRefreshSerialStats();
    return true;
}

bool appUART6SerialStreamRxTestGetStats(appUART6SerialStreamRxTestStats_t *stats_out)
{
    if (stats_out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *stats_out = g_app_uart6_serial_stream_rx_test_stats_;
    taskEXIT_CRITICAL();

    return true;
}

void appUART6SerialStreamRxTestTaskEntry(void *argument)
{
    TickType_t wait_ticks = pdMS_TO_TICKS(USER_UART6_SERIAL_STREAM_RX_TEST_WAIT_TIMEOUT_MS);

    (void)argument;

    memset((void *)&g_app_uart6_serial_stream_rx_test_stats_,
           0,
           sizeof(g_app_uart6_serial_stream_rx_test_stats_));

    while (appUART6SerialStreamRxTestInit() == false) {
        g_app_uart6_serial_stream_rx_test_stats_.init_retry_count_++;
        appUART6SerialStreamRxTestUpdateStackUsage();
        vTaskDelay(pdMS_TO_TICKS(APP_UART6_SERIAL_STREAM_RX_TEST_INIT_RETRY_DELAY_MS));
    }

    for (;;) {
        appUART6SerialStreamRxTestUpdateStackUsage();

#if USER_UART6_SERIAL_STREAM_RX_TEST_USE_ZERO_COPY
#if USER_UART6_SERIAL_STREAM_RX_TEST_USE_SEMAPHORE
        if (xSemaphoreTake(app_uart6_serial_stream_rx_test_.notify_semaphore_, wait_ticks) != pdPASS) {
            g_app_uart6_serial_stream_rx_test_stats_.wait_timeout_count_++;
            appUART6SerialStreamRxTestRefreshSerialStats();
            continue;
        }

        g_app_uart6_serial_stream_rx_test_stats_.wait_wakeup_count_++;
        g_app_uart6_serial_stream_rx_test_stats_.semaphore_take_count_++;
#else
        if (ulTaskNotifyTake(pdTRUE, wait_ticks) == 0U) {
            g_app_uart6_serial_stream_rx_test_stats_.wait_timeout_count_++;
            appUART6SerialStreamRxTestRefreshSerialStats();
            continue;
        }

        g_app_uart6_serial_stream_rx_test_stats_.wait_wakeup_count_++;
        g_app_uart6_serial_stream_rx_test_stats_.task_notify_take_count_++;
#endif
        if (appUART6SerialStreamRxTestDrainZeroCopy() == false) {
            g_app_uart6_serial_stream_rx_test_stats_.read_zero_length_count_++;
            appUART6SerialStreamRxTestRefreshSerialStats();
        }
#else
        uint8_t read_buffer[APP_UART6_SERIAL_STREAM_RX_TEST_READ_BUFFER_SIZE] = {0};
        uint32_t read_bytes = 0U;

        read_bytes = moduleSerialStreamRead(&app_uart6_serial_stream_rx_test_.serial_stream_,
                                            read_buffer,
                                            sizeof(read_buffer),
                                            wait_ticks);
        if (read_bytes == 0U) {
            g_app_uart6_serial_stream_rx_test_stats_.wait_timeout_count_++;
            appUART6SerialStreamRxTestRefreshSerialStats();
            continue;
        }

        g_app_uart6_serial_stream_rx_test_stats_.wait_wakeup_count_++;
        appUART6SerialStreamRxTestHandleReadChunk(read_buffer, read_bytes);
#endif
    }
}

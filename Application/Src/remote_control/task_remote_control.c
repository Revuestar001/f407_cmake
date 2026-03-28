#include "FreeRTOS.h"
#include "stream_buffer.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_board.h"
#include "bsp_uart.h"
#include "task_remote_control.h"
#include "sbus.h"

typedef struct task_remote_control
{
    bspUARTInstance_t *uart_instance_;
    StreamBufferHandle_t stream_buffer_handle_;

    // 统计丢失数据次数和字节数
    volatile uint32_t stream_buffer_drop_count_;
    volatile uint32_t stream_buffer_drop_bytes_;

    protocolSBUSDataPraser_t sbus_praser_;
    moduleRCMapper_t rc_mapper_;
} taskRemoteControlInstance_t;

static taskRemoteControlInstance_t task_instance_ = {0};

enum
{
    TASK_REMOTE_CONTROL_STREAM_BUFFER_CAPACITY = BSP_UART_RX_BUFFER_SIZE * 2U,
};

static uint8_t stream_buffer_[TASK_REMOTE_CONTROL_STREAM_BUFFER_CAPACITY + 1U];
static StaticStreamBuffer_t stream_buffer_struct_;

static void taskRemoteControlSendSegmentToBufferFromISR(taskRemoteControlInstance_t *instance,
                                                        uint8_t *rx_buffer_ptr,
                                                        uint16_t rx_buffer_size,
                                                        uint16_t rx_data_start_index,
                                                        uint16_t rx_data_end_pos,
                                                        BaseType_t *xHigherPriorityTaskWoken)
{
    size_t bytes_to_send;
    size_t bytes_sent;

    if (instance == NULL ||
        instance->stream_buffer_handle_ == NULL ||
        rx_buffer_ptr == NULL ||
        xHigherPriorityTaskWoken == NULL ||
        rx_data_start_index >= rx_buffer_size ||
        rx_data_end_pos > rx_buffer_size ||
        rx_data_start_index >= rx_data_end_pos) {
        return;
    }

    bytes_to_send = (size_t)(rx_data_end_pos - rx_data_start_index);
    bytes_sent = xStreamBufferSendFromISR(instance->stream_buffer_handle_,
                                          &rx_buffer_ptr[rx_data_start_index],
                                          bytes_to_send,
                                          xHigherPriorityTaskWoken);

    if (bytes_sent < bytes_to_send) {
        instance->stream_buffer_drop_count_++;
        instance->stream_buffer_drop_bytes_ += (uint32_t)(bytes_to_send - bytes_sent);
    }
}

// 在中断中
static void taskRemoteControlSendDataToBuffer(void *owner_ptr,
                                              uint8_t *rx_buffer_ptr,
                                              uint16_t rx_buffer_size,
                                              uint16_t rx_data_start_index,
                                              uint16_t rx_data_end_pos,
                                              bspUARTRxEventType_e rx_event)
{
    taskRemoteControlInstance_t *instance = (taskRemoteControlInstance_t *)owner_ptr;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    (void)rx_event;

    if (instance == NULL || rx_buffer_ptr == NULL || instance->stream_buffer_handle_ == NULL) {
        return;
    }

    if (rx_data_start_index < rx_data_end_pos) {
        taskRemoteControlSendSegmentToBufferFromISR(instance,
                                                    rx_buffer_ptr,
                                                    rx_buffer_size,
                                                    rx_data_start_index,
                                                    rx_data_end_pos,
                                                    &xHigherPriorityTaskWoken);
    } else if (rx_data_start_index > rx_data_end_pos) {
        taskRemoteControlSendSegmentToBufferFromISR(instance,
                                                    rx_buffer_ptr,
                                                    rx_buffer_size,
                                                    rx_data_start_index,
                                                    rx_buffer_size,
                                                    &xHigherPriorityTaskWoken);
        taskRemoteControlSendSegmentToBufferFromISR(instance,
                                                    rx_buffer_ptr,
                                                    rx_buffer_size,
                                                    0U,
                                                    rx_data_end_pos,
                                                    &xHigherPriorityTaskWoken);
    } else {
        return;
    }

    // 如果唤醒了更高优先级任务，触发调度
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void taskRemoteControlInit(void)
{
    memset(&task_instance_, 0, sizeof(taskRemoteControlInstance_t));

    task_instance_.uart_instance_ = bspBoardGetUARTInstance(BSP_UART_SBUS);
    if (task_instance_.uart_instance_ == NULL) {
        return;
    }

    // 触发字节数先只给1
    task_instance_.stream_buffer_handle_ = xStreamBufferCreateStatic(TASK_REMOTE_CONTROL_STREAM_BUFFER_CAPACITY,
                                                                     1U,
                                                                     stream_buffer_,
                                                                     &stream_buffer_struct_);
    if (task_instance_.stream_buffer_handle_ == NULL) {
        return;
    }

    protocolSBUSPraserInit(&task_instance_.sbus_praser_);
    moduleRCMapperInit(&task_instance_.rc_mapper_);

    bspUARTRxEventCallbackRegister(task_instance_.uart_instance_,
                                   (void *)&task_instance_,
                                   taskRemoteControlSendDataToBuffer);
    (void)bspUARTRxStart(task_instance_.uart_instance_);
}

// 返回是否成功更新一帧RC指令
bool taskRemoteControlUpdate(TickType_t timeout_tick)
{
    uint8_t rx_buffer[BSP_UART_RX_BUFFER_SIZE] = {0};
    uint16_t rx_data_length_actual = 0U;
    protocolSBUSDataFrame_t rx_sbus_frame;
    protocolSBUSFeedResult_e feed_result;
    bool updated = false;

    if (task_instance_.stream_buffer_handle_ == NULL) {
        return false;
    }

    rx_data_length_actual = (uint16_t)xStreamBufferReceive(task_instance_.stream_buffer_handle_,
                                                           rx_buffer,
                                                           sizeof(rx_buffer),
                                                           timeout_tick);

    if (rx_data_length_actual == 0U) {
        return false;
    }

    for (;;) {
        feed_result = protocolSBUSPraserFeedBufferLastFrame(&task_instance_.sbus_praser_,
                                                            rx_buffer,
                                                            rx_data_length_actual,
                                                            &rx_sbus_frame);
        if (feed_result == PROTOCOL_SBUS_FEED_FRAME_OK) {
            moduleRCMapperUpdateFromSBUSFrame(&task_instance_.rc_mapper_, &rx_sbus_frame);
            updated = true;
        }

        rx_data_length_actual = (uint16_t)xStreamBufferReceive(task_instance_.stream_buffer_handle_,
                                                               rx_buffer,
                                                               sizeof(rx_buffer),
                                                               0U);
        if (rx_data_length_actual == 0U) {
            break;
        }
    }

    return updated;
}

const moduleRCMapper_t *taskRemoteControlGetRCMapped(void)
{
    return &task_instance_.rc_mapper_;
}

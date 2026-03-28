#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_remote_control.h"
#include "bsp_board.h"
#include "bsp_uart.h"
#include "sbus.h"

typedef struct app_remote_control
{
    bspUARTInstance_t *uart_instance_;
    StreamBufferHandle_t stream_buffer_handle_;

    // 统计丢失数据次数和字节数
    volatile uint32_t stream_buffer_drop_count_;
    volatile uint32_t stream_buffer_drop_bytes_;

    protocolSBUSDataPraser_t sbus_praser_;
    appRemoteControlOutput_t output_;
} appRemoteControlInstance_t;

static appRemoteControlInstance_t app_instance_ = {0};

// stream buffer
// 请注意，stream buffer的大小+2(stream buffer实际容量)不能比底层bsp_uart的DMA缓冲区小，否则会造成字节丢失
static uint8_t stream_buffer_[BSP_UART_RX_BUFFER_SIZE * 2 + 2U];
static StaticStreamBuffer_t stream_buffer_struct_;

#define UPDATE_TEMP_BUFFER_SIZE PROTOCOL_SBUS_FRAME_SIZE

static appRemoteControlState_e appRemoteControlGetStateFromSBUSFrame(const protocolSBUSDataFrame_t *sbus_frame)
{
    if (sbus_frame == NULL) {
        return APP_REMOTE_CONTROL_STATE_LOST;
    }

    if (sbus_frame->frame_lost_flag_ != false || sbus_frame->failsafe_activate_flag_ != false) {
        return APP_REMOTE_CONTROL_STATE_FAILSAFE;
    }

    return APP_REMOTE_CONTROL_STATE_CONTROL;
}

static void appRemoteControlSetOutput(const appRemoteControlOutput_t *output)
{
    if (output == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    app_instance_.output_ = *output;
    taskEXIT_CRITICAL();
}

static void appRemoteControlSendSegmentToBufferFromISR(appRemoteControlInstance_t *instance,
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

    // 发送数据丢失，即中断中向streambuffer写入bytes_to_send字节，但实际只写入bytes_sent字节，说明容量不够
    if (bytes_sent < bytes_to_send) {
        instance->stream_buffer_drop_count_++;
        instance->stream_buffer_drop_bytes_ += (uint32_t)(bytes_to_send - bytes_sent);
    }
}

// 在中断中
static void appRemoteControlSendDataToBuffer(void *owner_ptr,
                                             uint8_t *rx_buffer_ptr,
                                             uint16_t rx_buffer_size,
                                             uint16_t rx_data_start_index,
                                             uint16_t rx_data_end_pos,
                                             bspUARTRxEventType_e rx_event)
{
    appRemoteControlInstance_t *instance = (appRemoteControlInstance_t *)owner_ptr;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    (void)rx_event;

    if (instance == NULL || rx_buffer_ptr == NULL || instance->stream_buffer_handle_ == NULL) {
        return;
    }

    if (rx_data_start_index < rx_data_end_pos) {
        appRemoteControlSendSegmentToBufferFromISR(instance,
                                                   rx_buffer_ptr,
                                                   rx_buffer_size,
                                                   rx_data_start_index,
                                                   rx_data_end_pos,
                                                   &xHigherPriorityTaskWoken);
    } else if (rx_data_start_index > rx_data_end_pos) {
        appRemoteControlSendSegmentToBufferFromISR(instance,
                                                   rx_buffer_ptr,
                                                   rx_buffer_size,
                                                   rx_data_start_index,
                                                   rx_buffer_size,
                                                   &xHigherPriorityTaskWoken);
        appRemoteControlSendSegmentToBufferFromISR(instance,
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

void appRemoteControlInit(void)
{
    memset(&app_instance_, 0, sizeof(appRemoteControlInstance_t));

    app_instance_.output_.state_ = APP_REMOTE_CONTROL_STATE_LOST;

    app_instance_.uart_instance_ = bspBoardGetUARTInstance(BSP_UART_SBUS);
    if (app_instance_.uart_instance_ == NULL) {
        return;
    }

    // 触发字节数先只给1
    // 实际容量还要再减1
    app_instance_.stream_buffer_handle_ = xStreamBufferCreateStatic(sizeof(stream_buffer_) - 1U,
                                                                    1U,
                                                                    stream_buffer_,
                                                                    &stream_buffer_struct_);
    if (app_instance_.stream_buffer_handle_ == NULL) {
        return;
    }

    protocolSBUSPraserInit(&app_instance_.sbus_praser_);
    moduleRCMapperInit(&app_instance_.output_.rc_mapper_);

    bspUARTRxEventCallbackRegister(app_instance_.uart_instance_,
                                   (void *)&app_instance_,
                                   appRemoteControlSendDataToBuffer);
    (void)bspUARTRxStart(app_instance_.uart_instance_);
}

// 返回是否成功更新一帧RC指令
bool appRemoteControlUpdate(TickType_t timeout_tick)
{
    // 临时缓冲数组
    uint8_t rx_buffer[UPDATE_TEMP_BUFFER_SIZE] = {0};
    uint16_t rx_data_length_actual = 0U;
    protocolSBUSDataFrame_t rx_sbus_frame;
    protocolSBUSFeedResult_e feed_result;
    appRemoteControlOutput_t output = app_instance_.output_;
    TickType_t first_wait_tick = timeout_tick;
    bool received_any_byte = false;
    bool received_valid_frame = false;

    if (app_instance_.stream_buffer_handle_ == NULL) {
        return false;
    }

    // 循环读取StreamBuffer，一个循环内把StreamBuffer内的数据全部给到SBUS praser,防止数据积压
    while (true) {
        // 第一次读取阻塞读取StreamBuffer
        // 一次最大只能读出临时数组大小
        rx_data_length_actual = (uint16_t)xStreamBufferReceive(app_instance_.stream_buffer_handle_,
                                                               rx_buffer,
                                                               sizeof(rx_buffer),
                                                               timeout_tick);

        // 如果从StreamBuffer里拿不到新数据了，终止循环
        if (rx_data_length_actual == 0U) {
            // 如果阻塞时间不为零且没有收到数据，就认为LOST，这个逻辑可能还需要更改
            if (received_any_byte == false && received_valid_frame == false && first_wait_tick != (TickType_t)0) {
                output.state_ = APP_REMOTE_CONTROL_STATE_LOST;
                appRemoteControlSetOutput(&output);
            }
            break;
        }

        received_any_byte = true;

        feed_result = protocolSBUSPraserFeedBufferLastFrame(&app_instance_.sbus_praser_,
                                                            rx_buffer,
                                                            rx_data_length_actual,
                                                            &rx_sbus_frame);
        if (feed_result == PROTOCOL_SBUS_FEED_FRAME_OK) {
            moduleRCMapperUpdateFromSBUSFrame(&output.rc_mapper_, &rx_sbus_frame);
            output.state_ = appRemoteControlGetStateFromSBUSFrame(&rx_sbus_frame);
            received_valid_frame = true;
        }

        // 第一次读取后，之后都不阻塞读取
        timeout_tick = 0;
    }

    if (received_valid_frame == true) {
        appRemoteControlSetOutput(&output);
        return true;
    }

    return false;
}

appRemoteControlOutput_t appRemoteControlGetOutput(void)
{
    appRemoteControlOutput_t output;

    taskENTER_CRITICAL();
    output = app_instance_.output_;
    taskEXIT_CRITICAL();

    return output;
}

appRemoteControlState_e appRemoteControlGetState(void)
{
    return appRemoteControlGetOutput().state_;
}

moduleRCMapper_t appRemoteControlGetRCMapped(void)
{
    return appRemoteControlGetOutput().rc_mapper_;
}

// 转换为速度油门指令
// 这里只是简单实现
static appRemoteControlCommand_t appRemoteControlBuildCommand(const appRemoteControlOutput_t *output)
{
    appRemoteControlCommand_t command = {0};

    if (output == NULL) {
        command.state_ = APP_REMOTE_CONTROL_STATE_LOST;
        return command;
    }

    command.state_ = output->state_;

    if (output->state_ != APP_REMOTE_CONTROL_STATE_CONTROL) {
        return command;
    }

    command.armed_ = (output->rc_mapper_.switch_left_.state_ == MODULE_RC_MAPPER_SWITCH_UP);

    switch (output->rc_mapper_.switch_right_.state_) {
    case MODULE_RC_MAPPER_SWITCH_UP:
        command.drive_mode_ = APP_REMOTE_CONTROL_DRIVE_MODE_AUTO;
        break;
    case MODULE_RC_MAPPER_SWITCH_MID:
        command.drive_mode_ = APP_REMOTE_CONTROL_DRIVE_MODE_ASSIST;
        break;
    default:
        command.drive_mode_ = APP_REMOTE_CONTROL_DRIVE_MODE_MANUAL;
        break;
    }

    command.linear_speed_ = output->rc_mapper_.stick_left_y_;
    command.angular_speed_ = output->rc_mapper_.stick_right_x_;

    return command;
}

appRemoteControlCommand_t appRemoteControlGetCommand(void)
{
    appRemoteControlOutput_t output = appRemoteControlGetOutput();
    return appRemoteControlBuildCommand(&output);
}








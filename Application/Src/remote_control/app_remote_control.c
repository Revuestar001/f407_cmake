#include "FreeRTOS.h"
#include "queue.h"
#include "stream_buffer.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_remote_control.h"
#include "bsp_board.h"
#include "bsp_uart.h"
#include "rc_mapper.h"
#include "sbus.h"

typedef struct app_remote_control_output
{
    appRemoteControlState_e state_;
    moduleRCMapper_t rc_mapper_;
} appRemoteControlOutput_t;

typedef struct app_remote_control
{
    bspUARTInstance_t *uart_instance_;
    StreamBufferHandle_t stream_buffer_handle_;
    QueueHandle_t command_queue_handle_;

    // 统计丢失数据次数和字节数
    volatile uint32_t stream_buffer_drop_count_;
    volatile uint32_t stream_buffer_drop_bytes_;

    protocolSBUSDataPraser_t sbus_praser_;
    appRemoteControlOutput_t output_;
    TickType_t last_valid_frame_tick_; // 记录上一次有效帧的timetick
} appRemoteControlInstance_t;

static appRemoteControlInstance_t app_rc_instance_ = {0};
static appRemoteControlCommand_t appRemoteControlBuildCommand(const appRemoteControlOutput_t *output);
static void appRemoteControlPublishCommand(const appRemoteControlOutput_t *output);

// stream buffer
// 请注意，stream buffer的大小+2(stream buffer实际容量)不能比底层bsp_uart的DMA缓冲区小，否则会造成字节丢失
static uint8_t stream_buffer_[BSP_UART_RX_BUFFER_SIZE * 2 + 2U];
static StaticStreamBuffer_t stream_buffer_struct_;
static uint8_t command_queue_storage_[sizeof(appRemoteControlCommand_t)];
static StaticQueue_t command_queue_struct_;

#define UPDATE_TEMP_BUFFER_SIZE PROTOCOL_SBUS_FRAME_SIZE
#define APP_REMOTE_CONTROL_REINIT_DELAY_MS 100U
#define APP_REMOTE_CONTROL_MAX_UPDATE_WAIT_MS 10U // 最低更新频率
#define APP_REMOTE_CONTROL_LOST_TIMEOUT_MS 50U // 认为RC LOST的超时时间，不要太小可能导致误判，请注意这个是指解析到两个完整25字节有效SBUS帧之间的时间，不是指frame_lost_flag
#define APP_REMOTE_CONTROL_COMMAND_QUEUE_LENGTH 1U

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

// 为rc输出状态设为lost
static void appRemoteControlSetOutputLost(appRemoteControlOutput_t *output)
{
    if (output == NULL) {
        return;
    }

    moduleRCMapperInit(&output->rc_mapper_);
    output->state_ = APP_REMOTE_CONTROL_STATE_LOST;
}

// 更新rc实例中的输出结构体
static void appRemoteControlUpdateOutput(const appRemoteControlOutput_t *output)
{
    if (output == NULL) {
        return;
    }

    // 这个临界区似乎不是很有必要
    taskENTER_CRITICAL();
    app_rc_instance_.output_ = *output;
    taskEXIT_CRITICAL();
}

// 判断是否超时了还没收到有效帧
static bool appRemoteControlIsFrameExpired(TickType_t timeout_tick)
{
    if (timeout_tick == (TickType_t)0) {
        return false;
    }

    return (xTaskGetTickCount() - app_rc_instance_.last_valid_frame_tick_) >= timeout_tick;
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

static bool appRemoteControlInit(void)
{
    memset(&app_rc_instance_, 0, sizeof(appRemoteControlInstance_t));
    appRemoteControlSetOutputLost(&app_rc_instance_.output_);
    app_rc_instance_.last_valid_frame_tick_ = xTaskGetTickCount();

    app_rc_instance_.uart_instance_ = bspBoardGetUARTInstance(BSP_UART_SBUS);
    if (app_rc_instance_.uart_instance_ == NULL) {
        return false;
    }

    // 触发字节数先只给1
    // 实际容量还要再减1
    app_rc_instance_.stream_buffer_handle_ = xStreamBufferCreateStatic(sizeof(stream_buffer_) - 1U,
                                                                    1U,
                                                                    stream_buffer_,
                                                                    &stream_buffer_struct_);
    if (app_rc_instance_.stream_buffer_handle_ == NULL) {
        return false;
    }

    app_rc_instance_.command_queue_handle_ = xQueueCreateStatic(APP_REMOTE_CONTROL_COMMAND_QUEUE_LENGTH,
                                                                sizeof(appRemoteControlCommand_t),
                                                                command_queue_storage_,
                                                                &command_queue_struct_);
    if (app_rc_instance_.command_queue_handle_ == NULL) {
        return false;
    }

    protocolSBUSPraserInit(&app_rc_instance_.sbus_praser_);
    moduleRCMapperInit(&app_rc_instance_.output_.rc_mapper_);
    appRemoteControlPublishCommand(&app_rc_instance_.output_);

    bspUARTRxEventCallbackRegister(app_rc_instance_.uart_instance_,
                                   (void *)&app_rc_instance_,
                                   appRemoteControlSendDataToBuffer);
    if (bspUARTRxStart(app_rc_instance_.uart_instance_) != BSP_UART_OK) {
        return false;
    }

    return true;
}

// 返回是否成功更新一帧RC指令
static bool appRemoteControlUpdate(TickType_t timeout_tick)
{
    // 临时缓冲数组
    uint8_t rx_buffer[UPDATE_TEMP_BUFFER_SIZE] = {0};
    uint16_t rx_data_length_actual = 0U;
    protocolSBUSDataFrame_t rx_sbus_frame;
    protocolSBUSFeedResult_e feed_result;
    // 这个读取也许要加临界区？似乎没有很大的必要
    appRemoteControlOutput_t output = app_rc_instance_.output_;
    bool received_valid_frame = false;

    if (app_rc_instance_.stream_buffer_handle_ == NULL) {
        return false;
    }

    // 循环读取StreamBuffer，一个循环内把StreamBuffer内的数据全部给到SBUS praser,防止数据积压
    while (true) {
        // 第一次读取阻塞读取StreamBuffer
        // 一次最大只能读出临时数组大小
        rx_data_length_actual = (uint16_t)xStreamBufferReceive(app_rc_instance_.stream_buffer_handle_,
                                                               rx_buffer,
                                                               sizeof(rx_buffer),
                                                               timeout_tick);

        // 如果从StreamBuffer里拿不到新数据了，终止循环
        if (rx_data_length_actual == 0U) {
            break;
        }

        feed_result = protocolSBUSPraserFeedBufferLastFrame(&app_rc_instance_.sbus_praser_,
                                                            rx_buffer,
                                                            rx_data_length_actual,
                                                            &rx_sbus_frame);
        if (feed_result == PROTOCOL_SBUS_FEED_FRAME_OK) {
            moduleRCMapperUpdateFromSBUSFrame(&output.rc_mapper_, &rx_sbus_frame);
            output.state_ = appRemoteControlGetStateFromSBUSFrame(&rx_sbus_frame);
            app_rc_instance_.last_valid_frame_tick_ = xTaskGetTickCount();
            received_valid_frame = true;
        }

        // 第一次读取后，之后都不阻塞读取
        timeout_tick = 0;
    }

    if (received_valid_frame == true) {
        appRemoteControlUpdateOutput(&output);
        appRemoteControlPublishCommand(&output);
        return true;
    }

    if (received_valid_frame == false &&
        appRemoteControlIsFrameExpired(pdMS_TO_TICKS(APP_REMOTE_CONTROL_LOST_TIMEOUT_MS)) == true) {
        appRemoteControlSetOutputLost(&output);
        appRemoteControlUpdateOutput(&output);
        appRemoteControlPublishCommand(&output);
    }

    return false;
}

static appRemoteControlOutput_t appRemoteControlGetOutput(void)
{
    appRemoteControlOutput_t output;

    taskENTER_CRITICAL();
    output = app_rc_instance_.output_;
    taskEXIT_CRITICAL();

    return output;
}

appRemoteControlState_e appRemoteControlGetState(void)
{
    return appRemoteControlGetOutput().state_;
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

static void appRemoteControlPublishCommand(const appRemoteControlOutput_t *output)
{
    if (output == NULL || app_rc_instance_.command_queue_handle_ == NULL) {
        return;
    }

    appRemoteControlCommand_t command = appRemoteControlBuildCommand(output);
    (void)xQueueOverwrite(app_rc_instance_.command_queue_handle_, &command);
}

appRemoteControlCommand_t appRemoteControlGetCommand(void)
{
    appRemoteControlOutput_t output = appRemoteControlGetOutput();
    return appRemoteControlBuildCommand(&output);
}

bool appRemoteControlReceiveCommand(appRemoteControlCommand_t *command_out, uint32_t timeout_tick)
{
    if (command_out == NULL || app_rc_instance_.command_queue_handle_ == NULL) {
        return false;
    }

    return xQueueReceive(app_rc_instance_.command_queue_handle_,
                         command_out,
                         (TickType_t)timeout_tick) == pdPASS;
}

void appRemoteControlTaskEntry(void *argument)
{
  (void)argument;

  // 尝试初始化
  while (appRemoteControlInit() == false) {
    vTaskDelay(pdMS_TO_TICKS(APP_REMOTE_CONTROL_REINIT_DELAY_MS));
  }

  for (;;) {
    // 无限阻塞不合理，超时10ms认为lost
    (void)appRemoteControlUpdate(pdMS_TO_TICKS(APP_REMOTE_CONTROL_MAX_UPDATE_WAIT_MS));
  }
}



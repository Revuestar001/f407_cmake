#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_remote_control.h"
#include "bsp_board.h"
#include "bsp_uart.h"
#include "bsp_dwt.h"
#include "rc_mapper.h"
#include "sbus.h"
#include "serial_stream.h"
#include "app_topics.h"
#include "topic_bus.h"
#include "msg_rc_command.h"

typedef struct app_remote_control_output
{
    appRemoteControlState_e state_;
    moduleRCMapper_t rc_mapper_;
} appRemoteControlOutput_t;

typedef struct app_remote_control
{
    StreamBufferHandle_t stream_buffer_handle_;

    moduleSerialStream_t serial_stream_;

    uint32_t serial_stats_base_total_bytes_;
    uint32_t serial_stats_base_accept_bytes_;
    uint32_t serial_stats_base_drop_count_;
    uint32_t serial_stats_base_drop_bytes_;

    appRemoteControlStats_t stats_;
    protocolSBUSDataPraser_t sbus_praser_;
    appRemoteControlOutput_t output_;

    uint64_t last_valid_frame_timestamp_us_; // 记录上一次有效帧的时间戳
} appRemoteControlInstance_t;

static appRemoteControlInstance_t app_rc_instance_ = {0};
static appRemoteControlCommand_t appRemoteControlBuildCommand(const appRemoteControlOutput_t *output);
static bool appRemoteControlPublishCommand(const appRemoteControlOutput_t *output);
static void appRemoteControlSyncSerialStats(void);
static uint32_t appRemoteControlCalcPermille(uint32_t numerator, uint32_t denominator);
static void appRemoteControlRefreshStatsDerived(appRemoteControlStats_t *stats);
static void appRemoteControlRecordValidSBUSFrame(const protocolSBUSDataFrame_t *sbus_frame);
static void appRemoteControlRecordSBUSParseError(void);
static void appRemoteControlHandleUARTError(void *owner_ptr);

// stream buffer
// 请注意，stream buffer的大小+2(stream buffer实际容量)不能比底层bsp_uart的DMA缓冲区小，否则会造成字节丢失
static uint8_t stream_buffer_[BSP_UART_RX_BUFFER_SIZE * 2 + 2U];
static StaticStreamBuffer_t stream_buffer_struct_;

#define UPDATE_TEMP_BUFFER_SIZE PROTOCOL_SBUS_FRAME_SIZE
#define APP_REMOTE_CONTROL_REINIT_DELAY_MS 100U
#define APP_REMOTE_CONTROL_MAX_UPDATE_WAIT_MS 10U // 最低更新频率
#define APP_REMOTE_CONTROL_LOST_TIMEOUT_MS 50U // 认为RC LOST的超时时间，不要太小可能导致误判，请注意这个是指解析到两个完整25字节有效SBUS帧之间的时间，不是指frame_lost_flag

static uint32_t appRemoteControlCalcPermille(uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0U) {
        return 0U;
    }

    return (uint32_t)((((uint64_t)numerator) * 1000U) / denominator);
}

static void appRemoteControlSyncSerialStats(void)
{
    taskENTER_CRITICAL();
    app_rc_instance_.stats_.uart_rx_total_bytes_ =
        app_rc_instance_.serial_stream_.rx_total_bytes_ - app_rc_instance_.serial_stats_base_total_bytes_;
    app_rc_instance_.stats_.stream_buffer_accept_bytes_ =
        app_rc_instance_.serial_stream_.rx_accept_bytes_ - app_rc_instance_.serial_stats_base_accept_bytes_;
    app_rc_instance_.stats_.stream_buffer_drop_count_ =
        app_rc_instance_.serial_stream_.rx_drop_count_ - app_rc_instance_.serial_stats_base_drop_count_;
    app_rc_instance_.stats_.stream_buffer_drop_bytes_ =
        app_rc_instance_.serial_stream_.rx_drop_bytes_ - app_rc_instance_.serial_stats_base_drop_bytes_;
    taskEXIT_CRITICAL();
}

static void appRemoteControlRefreshStatsDerived(appRemoteControlStats_t *stats)
{
    uint32_t sbus_frame_attempt_count;

    if (stats == NULL) {
        return;
    }

    sbus_frame_attempt_count = stats->sbus_valid_frame_count_ + stats->sbus_parse_error_count_;
    stats->sbus_frame_lost_flag_permille_ = appRemoteControlCalcPermille(stats->sbus_frame_lost_flag_count_,
                                                                         stats->sbus_valid_frame_count_);
    stats->sbus_failsafe_permille_ = appRemoteControlCalcPermille(stats->sbus_failsafe_count_,
                                                                  stats->sbus_valid_frame_count_);
    stats->sbus_parse_error_permille_ = appRemoteControlCalcPermille(stats->sbus_parse_error_count_,
                                                                     sbus_frame_attempt_count);
    stats->stream_buffer_drop_permille_ = appRemoteControlCalcPermille(stats->stream_buffer_drop_bytes_,
                                                                       stats->uart_rx_total_bytes_);
}

static void appRemoteControlRecordValidSBUSFrame(const protocolSBUSDataFrame_t *sbus_frame)
{
    if (sbus_frame == NULL) {
        return;
    }

    appRemoteControlSyncSerialStats();
    app_rc_instance_.stats_.sbus_valid_frame_count_++;
    if (sbus_frame->frame_lost_flag_ != false) {
        app_rc_instance_.stats_.sbus_frame_lost_flag_count_++;
    }
    if (sbus_frame->failsafe_activate_flag_ != false) {
        app_rc_instance_.stats_.sbus_failsafe_count_++;
    }

    appRemoteControlRefreshStatsDerived(&app_rc_instance_.stats_);
}

static void appRemoteControlRecordSBUSParseError(void)
{
    appRemoteControlSyncSerialStats();
    app_rc_instance_.stats_.sbus_parse_error_count_++;
}

static void appRemoteControlHandleUARTError(void *owner_ptr)
{
    appRemoteControlInstance_t *instance = (appRemoteControlInstance_t *)owner_ptr;

    if (instance == NULL) {
        return;
    }

    instance->stats_.uart_error_count_++;
}

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
static bool appRemoteControlIsFrameExpired(uint64_t timeout_us)
{
    if (timeout_us == 0U) {
        return false;
    }

    // return (xTaskGetTickCount() - app_rc_instance_.last_valid_frame_tick_) >= timeout_tick;
    return (bspDWTGetAbsTimeUs() - app_rc_instance_.last_valid_frame_timestamp_us_) >= timeout_us;
}

static bool appRemoteControlInit(void)
{
    memset(&app_rc_instance_, 0, sizeof(appRemoteControlInstance_t));
    appRemoteControlSetOutputLost(&app_rc_instance_.output_);
    app_rc_instance_.last_valid_frame_timestamp_us_ = bspDWTGetAbsTimeUs();

    // 触发字节数先只给1
    // 实际容量还要再减1
    app_rc_instance_.stream_buffer_handle_ = xStreamBufferCreateStatic(sizeof(stream_buffer_) - 1U,
                                                                    1U,
                                                                    stream_buffer_,
                                                                    &stream_buffer_struct_);
    if (app_rc_instance_.stream_buffer_handle_ == NULL) {
        return false;
    }

    protocolSBUSPraserInit(&app_rc_instance_.sbus_praser_);
    moduleRCMapperInit(&app_rc_instance_.output_.rc_mapper_);
    if (appRemoteControlPublishCommand(&app_rc_instance_.output_) == false) {
        return false;
    }
    
    moduleSerialStreamConfig_t serial_stream_config = {
        .uart_instance_ = bspBoardGetUARTInstance(BSP_UART_SBUS),
        .backend_ = MODULE_SERIAL_STREAM_BACKEND_STREAM_BUFFER,
        .backend_handle_.stream_buffer_handle_ = app_rc_instance_.stream_buffer_handle_,
        .notify_backend_ = MODULE_SERIAL_STREAM_NOTIFY_NONE,
    };

    if (serial_stream_config.uart_instance_ == NULL) {
        return false;
    }

    if (moduleSerialStreamInit(&app_rc_instance_.serial_stream_, &serial_stream_config) == false) {
        return false;
    }

    app_rc_instance_.serial_stats_base_total_bytes_ = app_rc_instance_.serial_stream_.rx_total_bytes_;
    app_rc_instance_.serial_stats_base_accept_bytes_ = app_rc_instance_.serial_stream_.rx_accept_bytes_;
    app_rc_instance_.serial_stats_base_drop_count_ = app_rc_instance_.serial_stream_.rx_drop_count_;
    app_rc_instance_.serial_stats_base_drop_bytes_ = app_rc_instance_.serial_stream_.rx_drop_bytes_;

    bspUARTErrorCallbackRegister(app_rc_instance_.serial_stream_.uart_instance_,
                                 (void *)&app_rc_instance_,
                                 appRemoteControlHandleUARTError);
    if (bspUARTRxStart(app_rc_instance_.serial_stream_.uart_instance_) != BSP_UART_OK) {
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
        rx_data_length_actual = (uint16_t)moduleSerialStreamRead(&app_rc_instance_.serial_stream_, 
                                                                rx_buffer, 
                                                                sizeof(rx_buffer), 
                                                                timeout_tick);
        // 如果从StreamBuffer里拿不到新数据了，终止循环
        if (rx_data_length_actual == 0U) {
            break;
        }

        for (uint16_t i = 0; i < rx_data_length_actual; i++) {
            protocolSBUSFeedResult_e feed_result = protocolSBUSPraserFeedByte(&app_rc_instance_.sbus_praser_,
                                                                              rx_buffer[i],
                                                                              &rx_sbus_frame);
            if (feed_result == PROTOCOL_SBUS_FEED_FRAME_OK) {
                appRemoteControlRecordValidSBUSFrame(&rx_sbus_frame);
                moduleRCMapperUpdateFromSBUSFrame(&output.rc_mapper_, &rx_sbus_frame);
                output.state_ = appRemoteControlGetStateFromSBUSFrame(&rx_sbus_frame);
                app_rc_instance_.last_valid_frame_timestamp_us_ = bspDWTGetAbsTimeUs();
                received_valid_frame = true;
            } else if (feed_result == PROTOCOL_SBUS_FEED_FRAME_ERROR) {
                appRemoteControlRecordSBUSParseError();
            }
        }

        // 第一次读取后，之后都不阻塞读取
        timeout_tick = 0;
    }

    if (received_valid_frame == true) {
        appRemoteControlUpdateOutput(&output);
        if (appRemoteControlPublishCommand(&output) == false) {
            return false;
        }
        return true;
    }

    if (received_valid_frame == false &&
        appRemoteControlIsFrameExpired(APP_REMOTE_CONTROL_LOST_TIMEOUT_MS * 1000U) == true) {
        if (output.state_ != APP_REMOTE_CONTROL_STATE_LOST) {
            app_rc_instance_.stats_.output_lost_timeout_count_++;
        }
        appRemoteControlSetOutputLost(&output);
        appRemoteControlUpdateOutput(&output);
        if (appRemoteControlPublishCommand(&output) == false) {
            return false;
        }
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

static bool appRemoteControlPublishCommand(const appRemoteControlOutput_t *output)
{
    if (output == NULL) {
        return false;
    }

    appRemoteControlCommand_t command = appRemoteControlBuildCommand(output);

    msgRCCommand_t msg = {
        .timestamp_ = bspDWTGetAbsTimeUs(),
        .state_ = command.state_,
        .armed_ = command.armed_,
        .drive_mode_ = command.drive_mode_,
        .linear_speed_ = command.linear_speed_,
        .angular_speed_ = command.angular_speed_,
    };

    if (moduleTopicBusPublish(appTopicsGet(APP_TOPICS_RC_COMMAND), &msg) == false) {
        return false;
    }

    return true;
}

appRemoteControlCommand_t appRemoteControlGetCommand(void)
{
    appRemoteControlOutput_t output = appRemoteControlGetOutput();
    return appRemoteControlBuildCommand(&output);
}

bool appRemoteControlGetStats(appRemoteControlStats_t *stats_out)
{
    if (stats_out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    appRemoteControlSyncSerialStats();
    *stats_out = app_rc_instance_.stats_;
    taskEXIT_CRITICAL();

    appRemoteControlRefreshStatsDerived(stats_out);
    return true;
}

void appRemoteControlResetStats(void)
{
    taskENTER_CRITICAL();
    app_rc_instance_.serial_stats_base_total_bytes_ = app_rc_instance_.serial_stream_.rx_total_bytes_;
    app_rc_instance_.serial_stats_base_accept_bytes_ = app_rc_instance_.serial_stream_.rx_accept_bytes_;
    app_rc_instance_.serial_stats_base_drop_count_ = app_rc_instance_.serial_stream_.rx_drop_count_;
    app_rc_instance_.serial_stats_base_drop_bytes_ = app_rc_instance_.serial_stream_.rx_drop_bytes_;
    app_rc_instance_.stats_ = (appRemoteControlStats_t){0};
    taskEXIT_CRITICAL();
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

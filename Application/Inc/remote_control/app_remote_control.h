#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "app_remote_control_def.h"

// 指令结构体
typedef struct app_remote_control_command
{
    appRemoteControlState_e state_;
    bool armed_;
    appRemoteControlDriveMode_e drive_mode_;
    int16_t linear_speed_;
    int16_t angular_speed_;
} appRemoteControlCommand_t;

typedef struct app_remote_control_stats
{
    uint32_t sbus_valid_frame_count_;
    uint32_t sbus_parse_error_count_;
    uint32_t sbus_frame_lost_flag_count_;
    uint32_t sbus_failsafe_count_;
    volatile uint32_t uart_rx_total_bytes_;
    volatile uint32_t stream_buffer_accept_bytes_;
    volatile uint32_t uart_error_count_;
    volatile uint32_t stream_buffer_drop_count_;
    volatile uint32_t stream_buffer_drop_bytes_;
    uint32_t output_lost_timeout_count_;
    uint32_t sbus_frame_lost_flag_permille_;
    uint32_t sbus_failsafe_permille_;
    uint32_t sbus_parse_error_permille_;
    uint32_t stream_buffer_drop_permille_;
} appRemoteControlStats_t;

appRemoteControlState_e appRemoteControlGetState(void);
// 一般直接获取指令值即可
appRemoteControlCommand_t appRemoteControlGetCommand(void);
// 通过任务间队列接收最新指令，timeout_tick 为 FreeRTOS tick
bool appRemoteControlReceiveCommand(appRemoteControlCommand_t *command_out, uint32_t timeout_tick);
// 获取当前统计快照
bool appRemoteControlGetStats(appRemoteControlStats_t *stats_out);
// 清零统计，便于按测试窗口重新观察
void appRemoteControlResetStats(void);

// 任务循环
void appRemoteControlTaskEntry(void *argument);

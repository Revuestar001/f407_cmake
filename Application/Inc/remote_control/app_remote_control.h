#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_REMOTE_CONTROL_STATE_LOST = 0,
    APP_REMOTE_CONTROL_STATE_CONTROL,
    APP_REMOTE_CONTROL_STATE_FAILSAFE
} appRemoteControlState_e;

typedef enum
{
    APP_REMOTE_CONTROL_DRIVE_MODE_MANUAL = 0,
    APP_REMOTE_CONTROL_DRIVE_MODE_ASSIST,
    APP_REMOTE_CONTROL_DRIVE_MODE_AUTO
} appRemoteControlDriveMode_e;

// 指令结构体
typedef struct app_remote_control_command
{
    appRemoteControlState_e state_;
    bool armed_;
    appRemoteControlDriveMode_e drive_mode_;
    int16_t linear_speed_;
    int16_t angular_speed_;
} appRemoteControlCommand_t;

appRemoteControlState_e appRemoteControlGetState(void);
// 一般直接获取指令值即可
appRemoteControlCommand_t appRemoteControlGetCommand(void);
// 通过任务间队列接收最新指令，timeout_tick 为 FreeRTOS tick
bool appRemoteControlReceiveCommand(appRemoteControlCommand_t *command_out, uint32_t timeout_tick);

// 任务循环
void appRemoteControlTaskEntry(void *argument);

#pragma once
#include "FreeRTOS.h"

#include <stdbool.h>

#include "rc_mapper.h"

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

typedef struct app_remote_control_output
{
    appRemoteControlState_e state_;
    moduleRCMapper_t rc_mapper_;
} appRemoteControlOutput_t;

// 指令结构体
typedef struct app_remote_control_command
{
    appRemoteControlState_e state_;
    bool armed_;
    appRemoteControlDriveMode_e drive_mode_;
    int16_t linear_speed_;
    int16_t angular_speed_;
} appRemoteControlCommand_t;

void appRemoteControlInit(void);
bool appRemoteControlUpdate(TickType_t timeout_tick);
appRemoteControlOutput_t appRemoteControlGetOutput(void);
appRemoteControlState_e appRemoteControlGetState(void);
moduleRCMapper_t appRemoteControlGetRCMapped(void);
// 一般直接获取指令值即可
appRemoteControlCommand_t appRemoteControlGetCommand(void);

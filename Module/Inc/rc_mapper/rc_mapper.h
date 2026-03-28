#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "sbus.h"

#define MODULE_RC_MAPPER_STICK_MIN_VALUE          (-1000)
#define MODULE_RC_MAPPER_STICK_MAX_VALUE          (1000)
#define MODULE_RC_MAPPER_STICK_DEADBAND_VALUE     (20)
#define MODULE_RC_MAPPER_CHANNEL_US_MID_VALUE     (1500U)
#define MODULE_RC_MAPPER_CHANNEL_US_HALF_RANGE    (500U)
#define MODULE_RC_MAPPER_SWITCH_LOW_THRESHOLD     (1300U)
#define MODULE_RC_MAPPER_SWITCH_HIGH_THRESHOLD    (1700U)

typedef enum
{
    MODULE_RC_MAPPER_SWITCH_DOWN = -1,
    MODULE_RC_MAPPER_SWITCH_MID = 0,
    MODULE_RC_MAPPER_SWITCH_UP = 1
} moduleRCMapperSwitchState_e;

typedef enum
{
    MODULE_RC_MAPPER_SWITCH_EDGE_NONE = 0,
    MODULE_RC_MAPPER_SWITCH_EDGE_RISING = (1U << 0),
    MODULE_RC_MAPPER_SWITCH_EDGE_FALLING = (1U << 1),
    MODULE_RC_MAPPER_SWITCH_EDGE_BOTH = MODULE_RC_MAPPER_SWITCH_EDGE_RISING | MODULE_RC_MAPPER_SWITCH_EDGE_FALLING
} moduleRCMapperSwitchEdge_e;

// 边沿判断请在同一个任务内立即处理，跨任务判断不可靠
typedef struct rc_mapper_switch
{
    moduleRCMapperSwitchState_e state_;
    moduleRCMapperSwitchState_e last_state_;
    moduleRCMapperSwitchEdge_e edge_;
} moduleRCSwitch_t;

// rc_mapper只提供rc通道量向摇杆量/开关量映射,不会自己创建实例，不会注册回调函数
// 因为rc_mapper不是板级资源，上层需要使用rc_mapper映射时才应该创建
typedef struct rc_mapper
{
    int16_t stick_left_x_;
    int16_t stick_left_y_;
    int16_t stick_right_x_;
    int16_t stick_right_y_;

    moduleRCSwitch_t switch_left_;
    moduleRCSwitch_t switch_right_;

    bool frame_lost_flag_;
    bool failsafe_activate_flag_;
} moduleRCMapper_t;

void moduleRCMapperInit(moduleRCMapper_t *instance);

bool moduleRCMapperUpdateFromSBUSFrame(moduleRCMapper_t *instance, const protocolSBUSDataFrame_t *sbus_frame);

bool moduleRCMapperSwitchStateChanged(const moduleRCSwitch_t *switch_instance);
moduleRCMapperSwitchEdge_e moduleRCMapperSwitchGetEdge(const moduleRCSwitch_t *switch_instance);
bool moduleRCMapperSwitchEdgeMatched(const moduleRCSwitch_t *switch_instance, moduleRCMapperSwitchEdge_e edge);

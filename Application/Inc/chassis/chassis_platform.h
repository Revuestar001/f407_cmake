#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "motor_actuator.h"

// 前向声明，平台内部上下文只在 .c 中定义
typedef struct app_chassis_platform_context appChassisPlatformContext_t;

typedef struct app_chassis_platform
{
    // 当前实现下，Init 后会指向 platform 模块内部维护的单例 context
    const appChassisPlatformContext_t *context_;
} appChassisPlatform_t;

// 初始化 platform，并装配当前机器人使用的所有 actuator
bool appChassisPlatformInit(appChassisPlatform_t *instance);

// 获取当前已注册的 actuator 数量
uint32_t appChassisPlatformGetActuatorCount(const appChassisPlatform_t *instance);

// 按索引获取 actuator 句柄；越界时返回 NULL
moduleMotorActuator_t *appChassisPlatformGetActuatorByIndex(const appChassisPlatform_t *instance, uint32_t index);

// 按索引获取 actuator 名称；越界时返回 NULL
const char *appChassisPlatformGetActuatorNameByIndex(const appChassisPlatform_t *instance, uint32_t index);

// 按名称查找 actuator；找不到时返回 NULL
moduleMotorActuator_t *appChassisPlatformFindActuatorByName(const appChassisPlatform_t *instance, const char *name);

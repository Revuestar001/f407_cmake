#pragma once

#include <stdbool.h>

#include "motor_actuator.h"
#include "motor_def.h"

#define MODULE_MOTOR_ACTUATOR_MAX_MOTOR_ONE_GROUP 4U

typedef struct module_motor_actuator_group_command_ops
{
    motorStatus_e (*group_commit_)(const void *motor);
} moduleMotorActuatorGroupCommandOps_t; // 指令虚函数表

typedef struct module_motor_actuator_group
{
    const moduleMotorActuatorGroupCommandOps_t *group_command_ops_;
    moduleMotorActuator_t *motor_instance_group[MODULE_MOTOR_ACTUATOR_MAX_MOTOR_ONE_GROUP]; // 只是几个指针，也许动态分配比较好？
} moduleMotorActuatorGroup_t;

// 初始化电机驱动器组，仅仅只是按组保存指针
bool moduleMotorActuatorGroupInit(moduleMotorActuatorGroup_t *instance, moduleMotorActuator_t *motors[MODULE_MOTOR_ACTUATOR_MAX_MOTOR_ONE_GROUP], const moduleMotorActuatorGroupCommandOps_t *ops);
// 按组提交所有组内电机驱动器的更新，也就是把几个电机驱动器更新好的电机命令打包发送出去
// 如果被控电机单独一个成组，也许不应该走这个api发送？
motorStatus_e moduleMotorActuatorGroupCommit(moduleMotorActuatorGroup_t *instance);
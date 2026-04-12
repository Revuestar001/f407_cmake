#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "pid.h"
#include "dji_m3508.h"
#include "motor_def.h"

typedef enum
{
    MODULE_MOTOR_ACTUATOR_CTRL_LOOP_OPEN = 0,
    MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE = 1U << 0,
    MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED = 1U << 1,
    // MODULE_MOTOR_ACTUATOR_CTRL_LOOP_EFFORT = 1U << 2, // 好像通常用不上
} moduleMotorActuatorControlLoop_e;

typedef enum
{
    MODULE_MOTOR_ACTUATOR_FF_TYPE_NONE = 0,
    MODULE_MOTOR_ACTUATOR_FF_TYPE_SPEED = 1U << 0,
    MODULE_MOTOR_ACTUATOR_FF_TYPE_EFFORT = 1U << 1,
} moduleMotorActuatorFeedforwardType_e;

typedef struct module_motor_actuator_commit_group
{
    const void *bus_; // 这一个group内电机所挂载的总线指针，通常这里是CAN
    uint32_t group_id_; // group的id，用于表示这个电机驱动器所在的group
} moduleMotorActuatorCommitGroup_t; // 用于存储在同一个控制报文内(同一个组)的电机驱动器信息

typedef struct module_motor_actuator_config
{
    motorDJIM3508Instance_t *motor_instance_; // 请注意，直接写出具体电机实例会使得这个模块无法适配多类型电机，之后要上oop!!!

    algorithmPIDConfig_t angle_pid_config_;
    algorithmPIDConfig_t angular_velocity_pid_config_;

    moduleMotorActuatorControlLoop_e control_loop_;
    moduleMotorActuatorFeedforwardType_e ff_type_;
} moduleMotorActuatorConfig_t;

typedef struct module_motor_actuator
{
    motorDJIM3508Instance_t *motor_instance_; // 请注意，直接写出具体电机实例会使得这个模块无法适配多类型电机，之后要上oop!!!
    moduleMotorActuatorCommitGroup_t commit_group_;

    algorithmPID_t angle_pid_;
    algorithmPID_t angular_velocity_pid_;

    moduleMotorActuatorControlLoop_e control_loop_;
    moduleMotorActuatorFeedforwardType_e ff_type_;

    motorWorkStatus_e work_status_;

    bool is_initialized_;
} moduleMotorActuator_t;

// 初始化电机驱动器，只绑定一个电机
bool moduleMotorActuatorInit(moduleMotorActuator_t *instance, moduleMotorActuatorConfig_t *config);
// 开启电机控制
bool moduleMotorActuatorEnableMotor(moduleMotorActuator_t *instance);
// 关闭电机控制并重置pid，实际上只是指令为0
bool moduleMotorActuatorDisableMotor(moduleMotorActuator_t *instance);
// 更新单个电机的控制指令值，不负责发送指令，注意这里暂时只按减速后的反馈数据计算pid，之后要增加减速前支持！！！
motorStatus_e moduleMotorActuatorUpdate(moduleMotorActuator_t *instance, float control_target);
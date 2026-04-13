#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "pid.h"
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

typedef struct module_motor_actuator_command_ops
{
    bool (*set_effort_ref_)(void *motor, float effort_ref);
    bool (*set_velocity_ref_)(void *motor, float velocity_ref);
    bool (*set_position_ref_)(void *motor, float position_ref);

    bool (*set_work_status_)(void *motor, motorWorkStatus_e work_status);

    motorStatus_e (*commit_command_)(const void *motor); // 请注意，按组发送指令的电机不支持commit

    bool (*get_commit_group_)(void *motor, moduleMotorActuatorCommitGroup_t *group_out);
} moduleMotorActuatorCommandOps_t; // 指令虚函数表

typedef struct module_motor_actuator_feedback_ops
{
    motorStatus_e (*update_feedback_)(void *source);
    motorStatus_e (*get_feedback_)(void *source, motorFeedBackData_t *feedback_out);
} moduleMotorActuatorFeedbackOps_t; // 反馈虚函数表

typedef struct module_motor_actuator_config
{
    void *motor_instance_; 
    const moduleMotorActuatorCommandOps_t *command_ops_;

    void *feedback_source_;
    const moduleMotorActuatorFeedbackOps_t *feedback_ops_;

    algorithmPIDConfig_t angle_pid_config_;
    algorithmPIDConfig_t angular_velocity_pid_config_;

    moduleMotorActuatorControlLoop_e control_loop_;
    moduleMotorActuatorFeedforwardType_e ff_type_;
} moduleMotorActuatorConfig_t;

typedef struct module_motor_actuator
{
    void *motor_instance_; 
    const moduleMotorActuatorCommandOps_t *command_ops_;
    moduleMotorActuatorCommitGroup_t commit_group_;

    void *feedback_source_;
    const moduleMotorActuatorFeedbackOps_t *feedback_ops_;

    algorithmPID_t angle_pid_;
    algorithmPID_t angular_velocity_pid_;

    moduleMotorActuatorControlLoop_e control_loop_;
    moduleMotorActuatorFeedforwardType_e ff_type_;

    motorWorkStatus_e work_status_;

    bool is_initialized_;
} moduleMotorActuator_t;

// 初始化电机驱动器，只绑定一个电机
bool moduleMotorActuatorInit(moduleMotorActuator_t *instance, const moduleMotorActuatorConfig_t *config);
// 开启电机控制
bool moduleMotorActuatorEnableMotor(moduleMotorActuator_t *instance);
// 关闭电机控制并重置pid，实际上只是指令为0
bool moduleMotorActuatorDisableMotor(moduleMotorActuator_t *instance);
// 更新单个电机的控制指令值，不负责发送指令，注意这里暂时只按减速后的反馈数据计算pid，之后要增加减速前支持！！！
motorStatus_e moduleMotorActuatorUpdate(moduleMotorActuator_t *instance, float control_target);
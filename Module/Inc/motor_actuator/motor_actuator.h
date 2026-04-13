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
    MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE_AND_SPEED = MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE | MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED, // 用于消除警告
} moduleMotorActuatorControlLoop_e; // actuator自己的闭环控制

typedef enum
{
    MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT = 0, // 电流/力矩/广义 effort 指令
    MODULE_MOTOR_ACTUATOR_CMD_TYPE_VELOCITY,
    MODULE_MOTOR_ACTUATOR_CMD_TYPE_POSITION,
    MODULE_MOTOR_ACTUATOR_CMD_TYPE_MIT, // mit模式下，actuator本身不需要闭环控制，请使用open_loop
} moduleMotorActuatorCommandType_e; // 请注意，这是指定最终输出的指令类型，不是指传入的ref类型！！

typedef enum
{
    MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_OUTPUT = 0, // 默认反馈数据使用输出轴(减速后)数据
    MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_ROTOR, // 反馈数据使用转子数据
} moduleMotorActuatorFeedbackSide_e;

typedef struct module_motor_actuator_command_ref
{
    float position_ref_rad_;
    float velocity_ref_rads_;
    float effort_ref_;

    float mit_kp_; // mit下独有
    float mit_kd_; // mit下独有
} moduleMotorActuatorCommandRef_t;

typedef struct module_motor_actuator_commit_group
{
    const void *bus_; // 这一个group内电机所挂载的总线指针，通常这里是CAN，如果电机没有协议group，那么指向电机实例自己
    uint32_t group_id_; // group的id，用于表示这个电机驱动器所在的group
} moduleMotorActuatorCommitGroup_t; // 用于存储在同一个控制报文内(同一个组)的电机驱动器信息

typedef struct module_motor_actuator_command_ops
{
    bool (*set_effort_ref_)(void *motor, float effort_ref);
    bool (*set_velocity_ref_)(void *motor, float velocity_ref);
    bool (*set_position_ref_)(void *motor, float position_ref);
    bool (*set_mit_ref_)(void *motor, const moduleMotorActuatorCommandRef_t *mit_ref);

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
    moduleMotorActuatorCommandType_e command_type_;
    moduleMotorActuatorFeedbackSide_e feedback_side_;

    float command_sign_; // actuator坐标系到电机命令坐标系，>=0表示同向，<0表示反向
    float feedback_sign_; // 反馈源坐标系到actuator坐标系，>=0表示同向，<0表示反向
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
    moduleMotorActuatorCommandType_e command_type_;
    moduleMotorActuatorFeedbackSide_e feedback_side_;

    float command_sign_;
    float feedback_sign_;

    motorWorkStatus_e work_status_;

    bool is_initialized_;
} moduleMotorActuator_t;

// 初始化电机驱动器，只绑定一个电机
bool moduleMotorActuatorInit(moduleMotorActuator_t *instance, const moduleMotorActuatorConfig_t *config);
// 开启电机控制
bool moduleMotorActuatorEnableMotor(moduleMotorActuator_t *instance);
// 关闭电机控制并重置pid，实际上只是指令为0
bool moduleMotorActuatorDisableMotor(moduleMotorActuator_t *instance);
// 使用结构化命令更新单个电机，不负责发送指令，MIT/位置/速度/力矩命令优先使用该接口
motorStatus_e moduleMotorActuatorUpdateCommand(moduleMotorActuator_t *instance, const moduleMotorActuatorCommandRef_t *control_ref);
// 更新单个电机的控制指令值，不负责发送指令，请尽量不要使用这个接口
motorStatus_e moduleMotorActuatorUpdate(moduleMotorActuator_t *instance, float control_target);
// 获取反馈数据，已转到actuator系下
motorStatus_e moduleMotorActuatorGetFeedbackData(moduleMotorActuator_t *instance, motorFeedBackData_t *data_out);

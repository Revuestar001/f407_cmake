#include <stdbool.h>
#include <string.h>

#include "dji_m3508.h"
#include "motor_actuator.h"
#include "motor_def.h"
#include "pid.h"

static bool checkControlLoopValid(moduleMotorActuatorControlLoop_e ctrl_loop)
{
    switch (ctrl_loop) {
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_OPEN:
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED:
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE | MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED:
            return true;
        default:
            // 串级pid，其他组合均不合法！
            return false;
    }
}

bool moduleMotorActuatorInit(moduleMotorActuator_t *instance, moduleMotorActuatorConfig_t *config)
{
    if (instance == NULL || config == NULL) {
        return false;
    }

    if (config->motor_instance_ == NULL) {
        return false;
    }

    // 暂时不支持前馈！！！之后会改！！！
    if (config->ff_type_ != MODULE_MOTOR_ACTUATOR_FF_TYPE_NONE) {
        return false;
    }

    if (checkControlLoopValid(config->control_loop_) == false) {
        return false;
    }

    memset(instance, 0, sizeof(moduleMotorActuator_t));

    instance->motor_instance_ = config->motor_instance_; // motor_instance_ 由 app 预先创建，actuator 只绑定并使用，不拥有其生命周期
    if (motorDJIM3508GetCommitGroup(instance->motor_instance_, &instance->commit_group_.bus_, &instance->commit_group_.group_id_) == false) {
        return false;
    }

    instance->control_loop_ = config->control_loop_;
    instance->ff_type_ = config->ff_type_;
    instance->work_status_ = MOTOR_WORK_STATUS_DISABLE;

    if ((config->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE) != 0U) {
        if (algorithmPIDInit(&instance->angle_pid_, &config->angle_pid_config_) == false) {
            return false;
        }
    }
    if ((config->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED) != 0U) {
        if (algorithmPIDInit(&instance->angular_velocity_pid_, &config->angular_velocity_pid_config_) == false) {
            return false;
        }
    }
    // 开环控制不需要pid

    instance->is_initialized_ = true;

    return true;
}

bool moduleMotorActuatorEnableMotor(moduleMotorActuator_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    if (motorDJIM3508SetWorkStatus(instance->motor_instance_, MOTOR_WORK_STATUS_ENABLE) == false) {
        return false;
    }

    instance->work_status_ = MOTOR_WORK_STATUS_ENABLE;

    return true;
}

bool moduleMotorActuatorDisableMotor(moduleMotorActuator_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    if (motorDJIM3508SetWorkStatus(instance->motor_instance_, MOTOR_WORK_STATUS_DISABLE) == false) {
        return false;
    }

    // 清空指令值
    if (motorDJIM3508SetCurrentRef(instance->motor_instance_, 0.0f) == false) {
        return false;
    }

    // reset pid
    if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE) != 0U) {
        if (algorithmPIDReset(&instance->angle_pid_) == false) {
            return false;
        }
    }
    if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED) != 0U) {
        if (algorithmPIDReset(&instance->angular_velocity_pid_) == false) {
            return false;
        }
    }

    instance->work_status_ = MOTOR_WORK_STATUS_DISABLE;

    return true;
}

motorStatus_e moduleMotorActuatorUpdate(moduleMotorActuator_t *instance, float control_target)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return MOTOR_ERROR;
    }

    if (instance->work_status_ != MOTOR_WORK_STATUS_ENABLE) {
        return MOTOR_STOP;
    }

    motorStatus_e motor_status;

    motorFeedBackData_t fb_data;
    motor_status = motorDJIM3508UpdateFeedbackData(instance->motor_instance_);
    if (motor_status == MOTOR_ERROR) {
        return motor_status;
    }
    // 允许在没有新反馈数据的时候，继续计算控制指令
    motor_status = motorDJIM3508GetFeedbackData(instance->motor_instance_, &fb_data);
    if (motor_status != MOTOR_OK && motor_status != MOTOR_NO_NEW_DATA) {
        return motor_status;
    }

    float pid_out = 0.0f;
    if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE) != 0U) {
        algorithmPIDUpdate(&instance->angle_pid_, control_target, fb_data.angle_fb_total_reduced_rad_);
        pid_out = instance->angle_pid_.PID_output_;
    }

    if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED) != 0U) {
        if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE) != 0U) {
            algorithmPIDUpdate(&instance->angular_velocity_pid_, instance->angle_pid_.PID_output_, fb_data.angular_velocity_fb_reduced_rads_);
        } else {
            algorithmPIDUpdate(&instance->angular_velocity_pid_, control_target, fb_data.angular_velocity_fb_reduced_rads_);
        }
        pid_out = instance->angular_velocity_pid_.PID_output_;
    }

    if (instance->control_loop_ == MODULE_MOTOR_ACTUATOR_CTRL_LOOP_OPEN) {
        pid_out = control_target;
    }

    if (motorDJIM3508SetCurrentRef(instance->motor_instance_, pid_out) == false) {
        return MOTOR_ERROR;
    }

    return MOTOR_OK;
}
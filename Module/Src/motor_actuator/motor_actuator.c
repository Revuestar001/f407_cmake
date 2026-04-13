#include <stdbool.h>
#include <string.h>

#include "motor_actuator.h"
#include "motor_def.h"
#include "pid.h"

static bool checkControlLoopValid(moduleMotorActuatorControlLoop_e ctrl_loop)
{
    switch (ctrl_loop) {
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_OPEN:
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE:
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED:
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE | MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED:
            return true;
        default:
            // 这里的 loop 表示 actuator 自己做的外环，不表示设备内部闭环
            return false;
    }
}

static bool checkCommandTypeValid(moduleMotorActuatorControlLoop_e ctrl_loop, moduleMotorActuatorCommandType_e command_type)
{
    switch (ctrl_loop) {
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_OPEN:
            return command_type == MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT ||
                   command_type == MODULE_MOTOR_ACTUATOR_CMD_TYPE_VELOCITY ||
                   command_type == MODULE_MOTOR_ACTUATOR_CMD_TYPE_POSITION ||
                   command_type == MODULE_MOTOR_ACTUATOR_CMD_TYPE_MIT;
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE: // actuator只做位置闭环，这里只能输出速度指令
            // 外部角度环输出速度指令；若设备自己闭位置环，应使用 OPEN + POSITION/MIT
            return command_type == MODULE_MOTOR_ACTUATOR_CMD_TYPE_VELOCITY;
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED: // // actuator只做速度闭环，输出effort指令
        case MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE | MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED: // actuator做位置-速度闭环，输出effort指令
            return command_type == MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT;
        default:
            return false;
    }
}

static bool checkCommandOpsValid(moduleMotorActuatorControlLoop_e ctrl_loop, moduleMotorActuatorCommandType_e command_type, const moduleMotorActuatorCommandOps_t *ops)
{
    if (ops == NULL || ops->set_work_status_ == NULL) {
        return false;
    }

    if (checkCommandTypeValid(ctrl_loop, command_type) == false) {
        return false;
    }

    switch (command_type) {
        case MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT:
            return ops->set_effort_ref_ != NULL;
        case MODULE_MOTOR_ACTUATOR_CMD_TYPE_VELOCITY:
            return ops->set_velocity_ref_ != NULL;
        case MODULE_MOTOR_ACTUATOR_CMD_TYPE_POSITION:
            return ops->set_position_ref_ != NULL;
        case MODULE_MOTOR_ACTUATOR_CMD_TYPE_MIT:
            return ops->set_mit_ref_ != NULL;
        default:
            return false;
    }
}

static bool checkFeedbackSideValid(moduleMotorActuatorFeedbackSide_e fb_side)
{
    switch (fb_side) {
        case MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_OUTPUT:
        case MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_ROTOR:
            return true;
        default:
            // 未知反馈side，既不是转子也不是输出轴   
            return false;
    }
}

static bool checkFeedbackOpsValid(moduleMotorActuatorControlLoop_e ctrl_loop, void *feedback_source, const moduleMotorActuatorFeedbackOps_t *ops)
{
    if (ctrl_loop == MODULE_MOTOR_ACTUATOR_CTRL_LOOP_OPEN) {
        // open loop不需要反馈源
        return true;
    }

    return feedback_source != NULL &&
           ops != NULL &&
           ops->update_feedback_ != NULL &&
           ops->get_feedback_ != NULL;
}

static float normalizeSign(float sign)
{
    return sign < 0.0f ? -1.0f : 1.0f;
}

static bool setCommandRef(moduleMotorActuator_t *instance, const moduleMotorActuatorCommandRef_t *control_ref)
{
    if (instance == NULL || instance->command_ops_ == NULL || control_ref == NULL) {
        return false;
    }

    // 使用command_sign_把actuator坐标系下的命令转到电机坐标系
    switch (instance->command_type_) {
        case MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT:
            return instance->command_ops_->set_effort_ref_(instance->motor_instance_, instance->command_sign_ * control_ref->effort_ref_);
        case MODULE_MOTOR_ACTUATOR_CMD_TYPE_VELOCITY:
            return instance->command_ops_->set_velocity_ref_(instance->motor_instance_, instance->command_sign_ * control_ref->velocity_ref_rads_);
        case MODULE_MOTOR_ACTUATOR_CMD_TYPE_POSITION:
            return instance->command_ops_->set_position_ref_(instance->motor_instance_, instance->command_sign_ * control_ref->position_ref_rad_);
        case MODULE_MOTOR_ACTUATOR_CMD_TYPE_MIT: {
            moduleMotorActuatorCommandRef_t signed_ref = *control_ref;
            signed_ref.position_ref_rad_ *= instance->command_sign_;
            signed_ref.velocity_ref_rads_ *= instance->command_sign_;
            signed_ref.effort_ref_ *= instance->command_sign_;
            return instance->command_ops_->set_mit_ref_(instance->motor_instance_, &signed_ref);
        }
        default:
            return false;
    }
}

// 仅用于设置电机停止函数内部使用
static bool clearCommandRefOnDisable(moduleMotorActuator_t *instance)
{
    if (instance == NULL || instance->command_ops_ == NULL) {
        return false;
    }

    if (instance->command_ops_->set_effort_ref_ != NULL &&
        instance->command_ops_->set_effort_ref_(instance->motor_instance_, 0.0f) == false) {
        return false;
    }

    if (instance->command_ops_->set_velocity_ref_ != NULL &&
        instance->command_ops_->set_velocity_ref_(instance->motor_instance_, 0.0f) == false) {
        return false;
    }

    // 不主动把 position/MIT ref 清 0，避免 position-only 设备 disable 时突然回零
    return true;
}

bool moduleMotorActuatorInit(moduleMotorActuator_t *instance, const moduleMotorActuatorConfig_t *config)
{
    if (instance == NULL || config == NULL) {
        return false;
    }

    if (config->motor_instance_ == NULL) {
        return false;
    }

    if (checkControlLoopValid(config->control_loop_) == false) {
        return false;
    }

    if (checkCommandOpsValid(config->control_loop_, config->command_type_, config->command_ops_) == false) {
        return false;
    }

    if (checkFeedbackSideValid(config->feedback_side_) == false) {
        return false;
    }

    if (checkFeedbackOpsValid(config->control_loop_, config->feedback_source_, config->feedback_ops_) == false) {
        return false;
    }

    memset(instance, 0, sizeof(moduleMotorActuator_t));

    // 绑定电机和反馈源
    instance->motor_instance_ = config->motor_instance_; // motor_instance_ 由 app 预先创建，actuator 只绑定并使用，不拥有其生命周期
    instance->feedback_source_ = config->feedback_source_;
    // 绑定虚函数表,直接指针赋值
    instance->command_ops_ = config->command_ops_;
    instance->feedback_ops_ = config->feedback_ops_;

    if (instance->command_ops_->get_commit_group_ != NULL) {
        if (instance->command_ops_->get_commit_group_(instance->motor_instance_, &instance->commit_group_) == false) {
            return false;
        }
    } else {
        // 没有协议级 group 的设备默认只允许自己单独成组，避免多个单帧设备被误判为同一 commit group
        instance->commit_group_.bus_ = instance->motor_instance_;
        instance->commit_group_.group_id_ = 0U;
    }

    instance->control_loop_ = config->control_loop_;
    instance->command_type_ = config->command_type_;
    instance->feedback_side_ = config->feedback_side_;
    instance->command_sign_ = normalizeSign(config->command_sign_);
    instance->feedback_sign_ = normalizeSign(config->feedback_sign_);
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

    if (instance->command_ops_->set_work_status_(instance->motor_instance_, MOTOR_WORK_STATUS_ENABLE) == false) {
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

    if (instance->command_ops_->set_work_status_(instance->motor_instance_, MOTOR_WORK_STATUS_DISABLE) == false) {
        return false;
    }

    if (clearCommandRefOnDisable(instance) == false) {
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

motorStatus_e moduleMotorActuatorUpdateCommand(moduleMotorActuator_t *instance, const moduleMotorActuatorCommandRef_t *control_ref)
{
    if (instance == NULL || control_ref == NULL || instance->is_initialized_ == false) {
        return MOTOR_ERROR;
    }

    if (instance->work_status_ != MOTOR_WORK_STATUS_ENABLE) {
        return MOTOR_STOP;
    }

    if (instance->control_loop_ == MODULE_MOTOR_ACTUATOR_CTRL_LOOP_OPEN) {
        // open loop
        // 请注意MIT模式本身需要open loop
        if (setCommandRef(instance, control_ref) == false) {
            return MOTOR_ERROR;
        }
        return MOTOR_OK;
    }

    // 反馈源更新反馈数据
    motorStatus_e motor_status = instance->feedback_ops_->update_feedback_(instance->feedback_source_);
    if (motor_status != MOTOR_OK && motor_status != MOTOR_NO_NEW_DATA) {
        return motor_status;
    }
    // 从反馈源获取反馈数据
    motorFeedBackData_t fb_data;
    motor_status = instance->feedback_ops_->get_feedback_(instance->feedback_source_, &fb_data);
    if (motor_status != MOTOR_OK) {
        return motor_status;
    }

    // 计算串级PID
    float pid_out = 0.0f;
    float position_estimate = 0.0f;
    float velocity_estimate = 0.0f;
    switch (instance->feedback_side_) {
        // 使用feedback_sign_把反馈源坐标系下的反馈数据转到actuator坐标系
        case MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_OUTPUT:
            position_estimate = instance->feedback_sign_ * fb_data.angle_fb_total_reduced_rad_;
            velocity_estimate = instance->feedback_sign_ * fb_data.angular_velocity_fb_reduced_rads_;
            break;
        case MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_ROTOR:
            position_estimate = instance->feedback_sign_ * fb_data.angle_fb_total_rad_;
            velocity_estimate = instance->feedback_sign_ * fb_data.angular_velocity_fb_rads_;
            break;
        default:
            return MOTOR_ERROR;
    }
    // 角度/位置环pid
    if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE) != 0U) {
        algorithmPIDUpdate(&instance->angle_pid_, control_ref->position_ref_rad_, position_estimate);
        pid_out = instance->angle_pid_.PID_output_;
    }
    // 速度环pid
    if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED) != 0U) {
        if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE) != 0U) {
            // 角度-速度 串级pid
            // control_ref的velocity_ref_rads_字段作为前馈
            algorithmPIDUpdate(&instance->angular_velocity_pid_, instance->angle_pid_.PID_output_ + control_ref->velocity_ref_rads_, velocity_estimate);
        } else {
            // 单速度环pid
            algorithmPIDUpdate(&instance->angular_velocity_pid_, control_ref->velocity_ref_rads_, velocity_estimate);
        }
        pid_out = instance->angular_velocity_pid_.PID_output_;
    }

    if (instance->control_loop_ == MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE) {
        // control_ref的velocity_ref_rads_字段作为前馈
        // 使用command_sign_把actuator坐标系下的命令转到电机坐标系
        if (instance->command_ops_->set_velocity_ref_(instance->motor_instance_, instance->command_sign_ * (pid_out + control_ref->velocity_ref_rads_)) == false) {
            return MOTOR_ERROR;
        }
        return MOTOR_OK;
    }

    // 只要不是单位置环pid，都输出effort指令
    // control_ref的effort_ref_字段作为前馈
    // 使用command_sign_把actuator坐标系下的命令转到电机坐标系
    if (instance->command_ops_->set_effort_ref_(instance->motor_instance_, instance->command_sign_ * (pid_out + control_ref->effort_ref_)) == false) {
        return MOTOR_ERROR;
    }

    return MOTOR_OK;
}

motorStatus_e moduleMotorActuatorUpdate(moduleMotorActuator_t *instance, float control_target)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return MOTOR_ERROR;
    }

    moduleMotorActuatorCommandRef_t control_ref;
    memset(&control_ref, 0, sizeof(moduleMotorActuatorCommandRef_t));

    if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE) != 0U) {
        control_ref.position_ref_rad_ = control_target;
    } else if ((instance->control_loop_ & MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED) != 0U) {
        control_ref.velocity_ref_rads_ = control_target;
    } else {
        switch (instance->command_type_) {
            case MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT:
                control_ref.effort_ref_ = control_target;
                break;
            case MODULE_MOTOR_ACTUATOR_CMD_TYPE_VELOCITY:
                control_ref.velocity_ref_rads_ = control_target;
                break;
            case MODULE_MOTOR_ACTUATOR_CMD_TYPE_POSITION:
            case MODULE_MOTOR_ACTUATOR_CMD_TYPE_MIT:
                control_ref.position_ref_rad_ = control_target;
                break;
            default:
                return MOTOR_ERROR;
        }
    }

    return moduleMotorActuatorUpdateCommand(instance, &control_ref);
}

motorStatus_e moduleMotorActuatorGetFeedbackData(moduleMotorActuator_t *instance, motorFeedBackData_t *data_out)
{
    if (instance == NULL || data_out == NULL || instance->is_initialized_ == false) {
        return MOTOR_ERROR;
    }

    if (instance->feedback_source_ == NULL || instance->feedback_ops_ == NULL || instance->feedback_ops_->get_feedback_ == NULL) {
        return MOTOR_ERROR;
    }

    motorStatus_e motor_status;
    motorFeedBackData_t fb_data;
    motor_status = instance->feedback_ops_->get_feedback_(instance->feedback_source_, &fb_data);
    if (motor_status != MOTOR_OK) {
        return motor_status;
    }

    // 使用feedback_sign_把反馈源坐标系下的反馈数据转到actuator坐标系
    fb_data.angle_fb_rad_ *= instance->feedback_sign_;
    fb_data.angle_fb_total_rad_ *= instance->feedback_sign_;
    fb_data.angle_fb_total_reduced_rad_ *= instance->feedback_sign_;
    fb_data.angular_velocity_fb_rads_ *= instance->feedback_sign_;
    fb_data.angular_velocity_fb_reduced_rads_ *= instance->feedback_sign_;

    *data_out = fb_data;

    return MOTOR_OK;
}

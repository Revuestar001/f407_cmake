#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pid.h"
#include "general_math.h"
#include "1st_order_lp_filter.h"

bool algorithmPIDInit(algorithmPID_t *instance, algorithmPIDConfig_t *config)
{
    if (instance == NULL || config == NULL) {
        return false;
    }

    if (config->k_P_ < 0.0f || 
        config->k_I_ < 0.0f || 
        config->k_D_ < 0.0f || 
        config->integral_upper_bound_ < config->integral_lower_bound_ ||
        config->output_upper_bound_ < config->output_lower_bound_ || 
        config->get_time_us_callback_ == NULL) {
        return false;
    }

    memset(instance, 0, sizeof(algorithmPID_t));
    instance->k_P_ = config->k_P_;
    instance->k_I_ = config->k_I_;
    instance->k_D_ = config->k_D_;
    instance->integral_lower_bound_ = config->integral_lower_bound_;
    instance->integral_upper_bound_ = config->integral_upper_bound_;
    instance->output_lower_bound_ = config->output_lower_bound_;
    instance->output_upper_bound_ = config->output_upper_bound_;
    instance->integral_method_ = config->integral_method_;
    instance->derivative_method_ = config->derivative_method_;
    instance->derivative_filter_ = config->derivative_filter_;
    instance->get_time_us_callback_ = config->get_time_us_callback_;
    instance->timestamp_last_us_ = instance->get_time_us_callback_();
    instance->dt_s_ = 0.0f;

    instance->is_initialized_ = true;
    instance->is_first_update_ = true;

    return true;
}

static bool calculatePItem(algorithmPID_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    instance->P_item_ = instance->k_P_ * instance->error_;

    return true;
}

static bool calculateIItem(algorithmPID_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    // dt太大，本次不计算I项
    if (instance->dt_s_ > ALGORITHM_PID_DT_MAX_S) {
        // 保持上一次的积分值，正常返回
        return true;
    }

    float integral = instance->integral_;

    switch (instance->integral_method_) {
        case ALGORITHM_PID_INTEGRAL_METHOD_TRAPEZOID: // 梯形积分，更平滑
            integral += 0.5f * (instance->error_ + instance->error_last_) * instance->dt_s_; 
            break;
        case ALGORITHM_PID_INTEGRAL_METHOD_RECTANGLE: // 默认矩形积分
        default:
            integral += instance->error_ * instance->dt_s_;   
            break;
    }

    if (mathClampf(integral, instance->integral_lower_bound_, instance->integral_upper_bound_, &instance->integral_) == false) {
        return false;
    }

    instance->I_item_ = instance->k_I_ * instance->integral_;

    return true;
}

static bool calculateDItem(algorithmPID_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    // dt太小，本次不计算D项
    if (instance->dt_s_ < ALGORITHM_PID_DT_MIN_S) {
        // 清空残留的微分项，正常返回
        instance->derivative_ = 0.0f;
        instance->D_item_ = 0.0f;
        return true;
    }

    float derivative = instance->derivative_;

    switch (instance->derivative_method_) {
        case ALGORITHM_PID_DIFFERENTIAL_METHOD_ESTIMATE: // 对测量值微分
            // 在target不变时，d_err = (err - err_last) / dt = ((tar - est) - (tar - est_last)) / dt = (est_last - est) / dt
            derivative = (instance->estimate_last_ - instance->estimate_) / instance->dt_s_;
            break;
        case ALGORITHM_PID_DIFFERENTIAL_METHOD_ERROR: // 默认对误差微分
        default:
            derivative = (instance->error_ - instance->error_last_) / instance->dt_s_;
            break;
    }

    // 微分滤波
    if (instance->derivative_filter_ != NULL) {
        if (filterOneOrderLPFUpdate(instance->derivative_filter_, derivative, &derivative) == false) {
            // 一般滤波update函数返回false是滤波器未初始化 或者 滤波器内部时间戳异常
            // 滤波器实例指针不为空，说明需要滤波，如果滤波失败就认为本次D项计算失败
            return false;
        }
    }

    instance->derivative_ = derivative;
    instance->D_item_ = instance->k_D_ * instance->derivative_;

    return true;
}

bool algorithmPIDUpdate(algorithmPID_t *instance, float target, float estimate)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    instance->target_ = target;
    instance->estimate_ = estimate;
    instance->error_ = target - estimate;

    uint64_t timestamp_us = instance->get_time_us_callback_();
    float dt_s = (float)(timestamp_us - instance->timestamp_last_us_) * 1.0e-6f;
    instance->dt_s_ = dt_s < 0.0f ? 0.0f : dt_s;
    instance->timestamp_last_us_ = timestamp_us;

    // 第一次更新，跳过id计算
    if (instance->is_first_update_ == true) {
        calculatePItem(instance);
        float pid_total = instance->P_item_;
        mathClampf(pid_total, instance->output_lower_bound_, instance->output_upper_bound_, &instance->PID_output_);

        instance->estimate_last_ = instance->estimate_;
        instance->error_last_ = instance->error_;
        instance->is_first_update_ = false;

        return true;
    }

    calculatePItem(instance);
    calculateIItem(instance);
    if (calculateDItem(instance) == false) {
        // D项计算失败，降级
        instance->derivative_ = 0.0f;
        instance->D_item_ = 0.0f;
    }
    
    float pid_total = instance->P_item_ + instance->I_item_ + instance->D_item_;
    mathClampf(pid_total, instance->output_lower_bound_, instance->output_upper_bound_, &instance->PID_output_);

    instance->estimate_last_ = instance->estimate_;
    instance->error_last_ = instance->error_;

    return true;
}

bool algorithmPIDReset(algorithmPID_t *instance)
{
    // 没初始化不允许reset
    if (instance == NULL || instance->is_initialized_ == false || instance->get_time_us_callback_ == NULL) {
        return false;
    }

    instance->integral_ = 0.0f;
    instance->derivative_ = 0.0f;
    instance->P_item_ = 0.0f;
    instance->I_item_ = 0.0f;
    instance->D_item_ = 0.0f;
    instance->PID_output_ = 0.0f;

    instance->timestamp_last_us_ = instance->get_time_us_callback_();

    instance->is_first_update_ = true;

    return true;
}
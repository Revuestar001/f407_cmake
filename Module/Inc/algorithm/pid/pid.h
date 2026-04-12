#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "1st_order_lp_filter.h"

#define ALGORITHM_PID_DT_MAX_S 0.1f
#define ALGORITHM_PID_DT_MIN_S 0.0001f

typedef enum
{
    ALGORITHM_PID_INTEGRAL_METHOD_RECTANGLE = 0, // 矩形积分
    ALGORITHM_PID_INTEGRAL_METHOD_TRAPEZOID, // 梯形积分
} algorithmPIDIntegralMethod_t; // 积分方法

typedef enum
{
    ALGORITHM_PID_DIFFERENTIAL_METHOD_ERROR = 0, // 使用误差微分
    ALGORITHM_PID_DIFFERENTIAL_METHOD_ESTIMATE, // 使用测量值微分
} algorithmPIDDifferentialMethod_t; // 微分方法

typedef uint64_t (*algorithmPIDGetAbsTimeUs_f)(void);

typedef struct algorithm_pid_config
{
    float k_P_;
    float k_I_;
    float k_D_;
    float integral_upper_bound_;
    float integral_lower_bound_;
    float output_upper_bound_; // 输出限制
    float output_lower_bound_;

    algorithmPIDIntegralMethod_t integral_method_;
    algorithmPIDDifferentialMethod_t differential_method_;
    filterOneOrderLPF_t *differential_filter_; // NULL 时不启用微分滤波

    algorithmPIDGetAbsTimeUs_f get_time_us_callback_;
} algorithmPIDConfig_t;

typedef struct algorithm_pid
{
    float k_P_;
    float k_I_;
    float k_D_;
    float integral_upper_bound_;
    float integral_lower_bound_;
    float output_upper_bound_; // 输出限制
    float output_lower_bound_;

    float target_; // 目标/期望值
    float estimate_; // 估计/测量值
    float estimate_last_; // 上次估计/测量值
    float error_; // 误差，统一为 (期望 - 测量)
    float error_last_; // 上次误差

    float integral_; // 积分项
    float differential_; // 微分项

    float P_item_;
    float I_item_;
    float D_item_;

    float PID_output_;

    algorithmPIDIntegralMethod_t integral_method_;
    algorithmPIDDifferentialMethod_t differential_method_;
    filterOneOrderLPF_t *differential_filter_; // NULL 时不启用微分滤波

    algorithmPIDGetAbsTimeUs_f get_time_us_callback_;
    uint64_t timestamp_last_us_;

    float dt_s_;

    bool is_initialized_;
    bool is_first_update_;
} algorithmPID_t;

bool algorithmPIDInit(algorithmPID_t *instance, algorithmPIDConfig_t *config);
bool algorithmPIDUpdate(algorithmPID_t *instance, float target, float estimate);
// 未初始化不允许reset
bool algorithmPIDReset(algorithmPID_t *instance);
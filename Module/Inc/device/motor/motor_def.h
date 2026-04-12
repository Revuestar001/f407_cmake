#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    MOTOR_OK = 0,
    MOTOR_TX_FAILURE,
    MOTOR_NO_NEW_DATA,
    MOTOR_STOP,
    MOTOR_ERROR,
} motorStatus_e;

typedef enum
{
    MOTOR_WORK_STATUS_DISABLE = 0,
    MOTOR_WORK_STATUS_ENABLE,
} motorWorkStatus_e;

typedef struct motor_ref
{
    float angle_ref_rad_;
    float angular_velocity_ref_rads_;
    float current_ref_A_;
} motorReferenceData_t;

typedef struct motor_feedback
{
    float angle_fb_rad_;
    float angle_fb_total_rad_; // 多圈总角度
    float angular_velocity_fb_rads_;
    float current_fb_A_;
    float temperature_c_;

    // 减速后数据
    float angle_fb_total_reduced_rad_;
    float angular_velocity_fb_reduced_rads_;

    uint64_t timestamp_us_;
} motorFeedBackData_t;

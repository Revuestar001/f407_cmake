#pragma once

#include <stdint.h>

typedef struct msg_ins
{
    uint64_t timestamp_;
    
    float quat_[4];
    float euler_zyx_deg_[3]; // ENU-FLU下,roll - x, pitch - y, yaw - z
    float gyro_bias_rads_[3];
} msgINS_t;
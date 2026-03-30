#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "app_def.h"

#define APP_IMU_TRY_INIT_MAX_COUNT 4U
#define APP_IMU_TRY_INIT_DELAY_MS 100U
#define APP_IMU_MAX_UPDATE_WAIT_MS 10U // 最低更新频率100Hz

typedef struct app_imu_data
{
    float accel_ms2_[3];
    float gyro_rads_[3];

    uint64_t timestamp_;
} appIMUData_t;

appState_e appIMUGetState();

// 任务循环
void appIMUTaskEntry(void *argument);
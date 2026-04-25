#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "app_def.h"

#define APP_INS_TRY_INIT_MAX_COUNT 4U
#define APP_INS_TRY_INIT_DELAY_MS 100U
#define APP_INS_MAX_UPDATE_WAIT_MS 10U // 最低更新频率 100Hz

typedef struct app_INS_data
{
    // INS 对外输出
    bool valid_;
    uint64_t timestamp_;
    float dt_s_;
    float quat_[4];
    float euler_zyx_rad_[3]; // ENU-FLU下,roll - x, pitch - y, yaw - z
    float euler_zyx_deg_[3];
    float gyro_bias_rads_[3];
} appINSData_t;

appState_e appINSGetAPPState(void);
// 在 INS 正常运行后手动进入 accel 六面标定模式
bool appINSStartAccelSixFaceCalibrate(void);
// accel 六面标定参数是否已经就绪
bool appINSGetAccelCalibrateReady(void);

// INS 任务入口
void appINSTaskEntry(void *argument);

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "arm_math.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "bsp_board.h"
#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bmi088.h"
#include "ist8310.h"
#include "app_ins.h"
#include "app_def.h"
#include "user_def.h"

#define APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2 (0.1f * STANDARD_GRAVITY_M_S2) // 用于判断IMU是否静止
#define APP_INS_CALIB_IMU_WARM_UP_TIME_MS 500U
#define APP_INS_CALIB_IMU_GYRO_BIAS_SAMPLE_COUNT 1200U
#define APP_INS_CALIB_GYRO_TOLERANCE_RADS 0.05f

#define APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 (0.1f * STANDARD_GRAVITY_M_S2) // 用于判断那个面平放
#define APP_INS_ACCEL_6FACE_STABLE_SAMPLE_COUNT 1200U // 认为哪个面静止的样本数
#define APP_INS_ACCEL_6FACE_SAMPLE_COUNT 4000U // accel六面校准采集样本数

#define APP_INS_ACCEL_BIAS_X_MS2_OFFLINE 0.128432751f
#define APP_INS_ACCEL_BIAS_Y_MS2_OFFLINE 0.140794277f
#define APP_INS_ACCEL_BIAS_Z_MS2_OFFLINE 0.0130887032f
#define APP_INS_ACCEL_SCALE_X_OFFLINE 1.00562215f
#define APP_INS_ACCEL_SCALE_Y_OFFLINE 1.00878429f
#define APP_INS_ACCEL_SCALE_Z_OFFLINE 1.0059551f

// app_ins 的工作模式：
// 1. 正常运行
// 2. accel 六面标定
typedef enum
{
    APP_INS_SERVICE_NORMAL = 0,
    APP_INS_SERVICE_ACCEL_6FACE_CALIB,
} appINSMode_e;

// 六面标定时识别当前是哪一个朝上面
typedef enum
{
    APP_INS_ACCEL_FACE_NONE = 0,
    APP_INS_ACCEL_FACE_F_UP,
    APP_INS_ACCEL_FACE_F_DOWN,
    APP_INS_ACCEL_FACE_L_UP,
    APP_INS_ACCEL_FACE_L_DOWN,
    APP_INS_ACCEL_FACE_U_UP,
    APP_INS_ACCEL_FACE_U_DOWN,
} appINSAccelCalibrateFace_e;

// accel 六面标定上下文
typedef struct app_ins_accel_6_face_calibrate
{
    bool active_;
    bool face_done_[6];
    appINSAccelCalibrateFace_e current_face_;
    uint32_t stable_count_;
    uint32_t face_sample_count_[6];
    float face_measure_accumulate_[6][3];
} appINSAccelSixFaceCalibrate_t; // accel六面标定中间数据

// app_ins 内部启动状态
typedef enum
{
    APP_INS_IMU_MAG_INIT = 0,
    APP_INS_IMU_WARM_UP,
    APP_INS_IMU_CALIB_GYRO_BIAS,
    APP_INS_IMU_CALIB_ACCEL,
    APP_INS_WAIT_MAG_SAMPLE,
    APP_INS_EKF_INIT,
    APP_INS_RUNNING,
    APP_INS_DEGRADED,
    APP_INS_REINIT,
} appINSStage_e; // INS内部状态枚举

typedef struct app_ins_calibrate_data
{
    TickType_t warm_up_tick_;

    // gyro 上电静止 bias 校准
    float gyro_bias_rads_[3];
    float gyro_bias_accumulate_[3];
    uint32_t gyro_bias_sample_count_;
    bool gyro_bias_ready_;

    // accel 六面标定参数与过程上下文
    appINSAccelSixFaceCalibrate_t accel_calib_;
    float accel_bias_ms2_[3];
    float accel_scale_[3];
    bool accel_bias_scale_ready_;
} appINSCalibrateData_t; // 包含标定校准数据

typedef struct app_ins_observation_data
{
    // 给 EKF 使用的校准后数据
    float accel_corrected_ms2_[3];
    float gyro_corrected_rads_[3];
    float mag_ut_[3];

    uint64_t imu_timestamp_us_; // imu时间戳
    uint64_t mag_timestamp_us_; // mag时间戳
    bool mag_is_new_; // 表示这个mag_timestamp_us_时间戳下的mag数据是否是新来的，用于ekf的mag update
} appINSObservationData_t; // ekf输入数据

typedef struct app_ins_mag_record_data
{
    uint32_t update_count_;
    uint64_t latest_timestamp_us_;

    uint8_t chip_id_;

    int16_t mag_raw_before_transform_[3];
    float mag_ut_[3];
    float mag_norm_ut_;
    float mag_horizontal_norm_ut_; // 仅看 FL 平面内的模长，方便平放转圈观察

    bool min_max_initialized_;
    float mag_min_ut_[3];
    float mag_max_ut_[3];
} appINSMagRecordData_t; // 给 live watch/调试器观察的磁力计记录数据

typedef struct app_ins
{
    // 当前 INS 任务句柄，在 task entry 中绑定
    TaskHandle_t task_handle_;

    deviceBMI088Instance_t *bmi088_instance_;
    deviceIST8310Instance_t *ist8310_instance_;

    // EXTI 中断引脚
    bspGPIOInstance_t *accel_int_;
    bspGPIOInstance_t *gyro_int_;
    bspGPIOInstance_t *mag_int_;

    appINSMode_e mode_;
    appINSData_t data_; // 输出用，姿态解算后的数据
    appINSStage_e stage_;

    // 最新传感器数据，不在这里原地做 app 层校准
    deviceBMI088Data_t latest_imu_data_;
    deviceIST8310Data_t latest_mag_data_;
    // MAG 中断到来，只置位，不直接在 ISR 里读 I2C
    bool mag_new_data_ready_;
    // 表示至少已经成功拿到一帧有效 MAG 数据
    bool mag_data_valid_;

    appINSCalibrateData_t calibrate_data_;
    appINSObservationData_t observation_data_; // 校正过后的九轴数据，给ekf使用
    appINSMagRecordData_t mag_record_data_; // 磁力计调试记录数据

    uint8_t error_count_;
} appINSInstance_t;

static appINSInstance_t app_ins_ = {0};
static uint32_t ins_loop_time = 0;
static uint32_t ins_loop_time_max = 0;

// 3 维向量模长
static float appINSGetVectorNorm3f(const float vector[3])
{
    float norm_square = vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2];
    float norm = 0.0f;

    if (arm_sqrt_f32(norm_square, &norm) != ARM_MATH_SUCCESS) {
        return 0.0f;
    }

    return norm;
}

// gyro 校准应用点：只减 bias
static void appINSApplyGyroBias(const float gyro[3], const float gyro_bias[3], float gyro_out[3])
{
    gyro_out[0] = gyro[0] - gyro_bias[0];
    gyro_out[1] = gyro[1] - gyro_bias[1];
    gyro_out[2] = gyro[2] - gyro_bias[2];
}

// accel 校准应用点：每轴 bias + scale
static void appINSApplyAccelBiasScale(const float accel[3], const float accel_bias[3], const float accel_scale[3], float accel_out[3])
{
    accel_out[0] = (accel[0] - accel_bias[0]) * accel_scale[0];
    accel_out[1] = (accel[1] - accel_bias[1]) * accel_scale[1];
    accel_out[2] = (accel[2] - accel_bias[2]) * accel_scale[2];
}

// 记录一份给调试器直接观察的磁力计数据：
// 1. 原始值（交叉轴补偿前）
// 2. 当前输出的 mag_ut_
// 3. 三轴模长与平面模长
// 4. 旋转过程中的 min/max，方便看点云范围是否合理
static void appINSUpdateMAGRecordData(void)
{
    float horizontal_norm_square = 0.0f;
    float horizontal_norm = 0.0f;

    app_ins_.mag_record_data_.update_count_++;
    app_ins_.mag_record_data_.latest_timestamp_us_ = app_ins_.observation_data_.mag_timestamp_us_;
    app_ins_.mag_record_data_.chip_id_ = app_ins_.latest_mag_data_.chip_id_;

    for (uint8_t i = 0; i < 3U; i++) {
        app_ins_.mag_record_data_.mag_raw_before_transform_[i] = app_ins_.latest_mag_data_.mag_raw_before_transform[i];
        app_ins_.mag_record_data_.mag_ut_[i] = app_ins_.latest_mag_data_.mag_ut_[i];
    }

    app_ins_.mag_record_data_.mag_norm_ut_ = appINSGetVectorNorm3f(app_ins_.latest_mag_data_.mag_ut_);

    horizontal_norm_square =
        app_ins_.latest_mag_data_.mag_ut_[0] * app_ins_.latest_mag_data_.mag_ut_[0] +
        app_ins_.latest_mag_data_.mag_ut_[1] * app_ins_.latest_mag_data_.mag_ut_[1];

    if (arm_sqrt_f32(horizontal_norm_square, &horizontal_norm) != ARM_MATH_SUCCESS) {
        horizontal_norm = 0.0f;
    }
    app_ins_.mag_record_data_.mag_horizontal_norm_ut_ = horizontal_norm;

    if (app_ins_.mag_record_data_.min_max_initialized_ == false) {
        for (uint8_t i = 0; i < 3U; i++) {
            app_ins_.mag_record_data_.mag_min_ut_[i] = app_ins_.latest_mag_data_.mag_ut_[i];
            app_ins_.mag_record_data_.mag_max_ut_[i] = app_ins_.latest_mag_data_.mag_ut_[i];
        }
        app_ins_.mag_record_data_.min_max_initialized_ = true;
        return;
    }

    for (uint8_t i = 0; i < 3U; i++) {
        if (app_ins_.latest_mag_data_.mag_ut_[i] < app_ins_.mag_record_data_.mag_min_ut_[i]) {
            app_ins_.mag_record_data_.mag_min_ut_[i] = app_ins_.latest_mag_data_.mag_ut_[i];
        }
        if (app_ins_.latest_mag_data_.mag_ut_[i] > app_ins_.mag_record_data_.mag_max_ut_[i]) {
            app_ins_.mag_record_data_.mag_max_ut_[i] = app_ins_.latest_mag_data_.mag_ut_[i];
        }
    }
}

// 静止判据：
// 1. gyro 足够小
// 2. accel 模长接近 1g
static bool appINSIsIMUStationary(void)
{
    float accel_norm = appINSGetVectorNorm3f(app_ins_.latest_imu_data_.accel_ms2_);

    return fabsf(app_ins_.latest_imu_data_.gyro_rads_[0]) <= APP_INS_CALIB_GYRO_TOLERANCE_RADS &&
           fabsf(app_ins_.latest_imu_data_.gyro_rads_[1]) <= APP_INS_CALIB_GYRO_TOLERANCE_RADS &&
           fabsf(app_ins_.latest_imu_data_.gyro_rads_[2]) <= APP_INS_CALIB_GYRO_TOLERANCE_RADS &&
           fabsf(accel_norm - STANDARD_GRAVITY_M_S2) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2;
}

// 清空一次 accel 六面标定过程上下文
static void appINSResetAccelSixFaceCalibrate(void)
{
    memset(&app_ins_.calibrate_data_.accel_calib_, 0, sizeof(appINSAccelSixFaceCalibrate_t));
}

// 根据当前未校准 accel 数据判断是哪个面朝上
static appINSAccelCalibrateFace_e appINSDetectAccelCalibrateFace(void)
{
    const float *accel = app_ins_.latest_imu_data_.accel_ms2_;

    if (fabsf(accel[0] - STANDARD_GRAVITY_M_S2) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[1]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[2]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2) {
        return APP_INS_ACCEL_FACE_F_UP;
    }

    if (fabsf(accel[0] + STANDARD_GRAVITY_M_S2) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[1]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[2]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2) {
        return APP_INS_ACCEL_FACE_F_DOWN;
    }

    if (fabsf(accel[1] - STANDARD_GRAVITY_M_S2) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[0]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[2]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2) {
        return APP_INS_ACCEL_FACE_L_UP;
    }

    if (fabsf(accel[1] + STANDARD_GRAVITY_M_S2) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[0]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[2]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2) {
        return APP_INS_ACCEL_FACE_L_DOWN;
    }

    if (fabsf(accel[2] - STANDARD_GRAVITY_M_S2) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[0]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[1]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2) {
        return APP_INS_ACCEL_FACE_U_UP;
    }

    if (fabsf(accel[2] + STANDARD_GRAVITY_M_S2) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[0]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2 &&
        fabsf(accel[1]) <= APP_INS_ACCEL_6FACE_AXIS_TOLERANCE_MS2) {
        return APP_INS_ACCEL_FACE_U_DOWN;
    }

    return APP_INS_ACCEL_FACE_NONE;
}

// 六个面是否都已经采满
static bool appINSIsAccelAllSixFaceDone(void)
{
    for (uint8_t i = 0; i < 6U; i++) {
        if (app_ins_.calibrate_data_.accel_calib_.face_done_[i] == false) {
            return false;
        }
    }

    return true;
}

// 六个面采满后，按每轴独立 bias + scale 计算参数
static void appINSFinishAccelSixFaceCalibrate(void)
{
    float face_average[6][3] = {{0.0f}};

    for (uint8_t face = 0; face < 6U; face++) {
        if (app_ins_.calibrate_data_.accel_calib_.face_sample_count_[face] == 0U) {
            return;
        }

        for (uint8_t axis = 0; axis < 3U; axis++) {
            face_average[face][axis] = app_ins_.calibrate_data_.accel_calib_.face_measure_accumulate_[face][axis] /
                                       (float)app_ins_.calibrate_data_.accel_calib_.face_sample_count_[face];
        }
    }

    // face_average 的顺序固定为：
    // [0]/[1] -> +F / -F
    // [2]/[3] -> +L / -L
    // [4]/[5] -> +U / -U
    //
    // 六面法的一轴模型可以写成：
    // meas = true / scale + bias
    // 当该轴正向朝上时，true = +g；反向朝上时，true = -g
    // 因此：
    // pos = +g / scale + bias
    // neg = -g / scale + bias
    //
    // 两式相加可得 bias = (pos + neg) / 2
    app_ins_.calibrate_data_.accel_bias_ms2_[0] = (face_average[0][0] + face_average[1][0]) * 0.5f;
    app_ins_.calibrate_data_.accel_bias_ms2_[1] = (face_average[2][1] + face_average[3][1]) * 0.5f;
    app_ins_.calibrate_data_.accel_bias_ms2_[2] = (face_average[4][2] + face_average[5][2]) * 0.5f;

    // 两式相减可得：
    // pos - neg = 2g / scale
    // 所以 scale = 2g / (pos - neg)
    if (fabsf(face_average[0][0] - face_average[1][0]) > 1e-6f) {
        app_ins_.calibrate_data_.accel_scale_[0] = (2.0f * STANDARD_GRAVITY_M_S2) / (face_average[0][0] - face_average[1][0]);
    }
    if (fabsf(face_average[2][1] - face_average[3][1]) > 1e-6f) {
        app_ins_.calibrate_data_.accel_scale_[1] = (2.0f * STANDARD_GRAVITY_M_S2) / (face_average[2][1] - face_average[3][1]);
    }
    if (fabsf(face_average[4][2] - face_average[5][2]) > 1e-6f) {
        app_ins_.calibrate_data_.accel_scale_[2] = (2.0f * STANDARD_GRAVITY_M_S2) / (face_average[4][2] - face_average[5][2]);
    }

    app_ins_.calibrate_data_.accel_bias_scale_ready_ = true;
    app_ins_.calibrate_data_.accel_calib_.active_ = false;
    app_ins_.calibrate_data_.accel_calib_.current_face_ = APP_INS_ACCEL_FACE_NONE;
    app_ins_.calibrate_data_.accel_calib_.stable_count_ = 0U;
    app_ins_.mode_ = APP_INS_SERVICE_NORMAL; // 退出校准模式
}

// 在 IMU 更新链里单步推进六面标定
static void appINSProcessAccelSixFaceCalibrate(void)
{
    appINSAccelCalibrateFace_e face;
    uint8_t face_index;

    if (app_ins_.mode_ != APP_INS_SERVICE_ACCEL_6FACE_CALIB || app_ins_.calibrate_data_.accel_calib_.active_ == false) {
        return;
    }
    // 如果imu没有静止，重来
    if (appINSIsIMUStationary() == false) {
        app_ins_.calibrate_data_.accel_calib_.current_face_ = APP_INS_ACCEL_FACE_NONE;
        app_ins_.calibrate_data_.accel_calib_.stable_count_ = 0U;
        return;
    }
    // 如果判定不到任何一面可以校准，可能是没放平，重来
    face = appINSDetectAccelCalibrateFace();
    if (face == APP_INS_ACCEL_FACE_NONE) {
        app_ins_.calibrate_data_.accel_calib_.current_face_ = APP_INS_ACCEL_FACE_NONE;
        app_ins_.calibrate_data_.accel_calib_.stable_count_ = 0U;
        return;
    }
    // 找到一面可以校准，开始记录静止保持
    if (app_ins_.calibrate_data_.accel_calib_.current_face_ != face) {
        app_ins_.calibrate_data_.accel_calib_.current_face_ = face;
        app_ins_.calibrate_data_.accel_calib_.stable_count_ = 1U;
        return;
    }
    // 该面保持静止直到样本数超过设定值
    if (app_ins_.calibrate_data_.accel_calib_.stable_count_ < APP_INS_ACCEL_6FACE_STABLE_SAMPLE_COUNT) {
        app_ins_.calibrate_data_.accel_calib_.stable_count_++;
        return;
    }
    // 该面开始校准
    face_index = (uint8_t)face - 1U;
    if (app_ins_.calibrate_data_.accel_calib_.face_done_[face_index] == true) {
        return;
    }

    app_ins_.calibrate_data_.accel_calib_.face_measure_accumulate_[face_index][0] += app_ins_.latest_imu_data_.accel_ms2_[0];
    app_ins_.calibrate_data_.accel_calib_.face_measure_accumulate_[face_index][1] += app_ins_.latest_imu_data_.accel_ms2_[1];
    app_ins_.calibrate_data_.accel_calib_.face_measure_accumulate_[face_index][2] += app_ins_.latest_imu_data_.accel_ms2_[2];
    app_ins_.calibrate_data_.accel_calib_.face_sample_count_[face_index]++;
    // 该面样本数够了，标记记录数据完成
    if (app_ins_.calibrate_data_.accel_calib_.face_sample_count_[face_index] >= APP_INS_ACCEL_6FACE_SAMPLE_COUNT) {
        app_ins_.calibrate_data_.accel_calib_.face_done_[face_index] = true;
        app_ins_.calibrate_data_.accel_calib_.current_face_ = APP_INS_ACCEL_FACE_NONE;
        app_ins_.calibrate_data_.accel_calib_.stable_count_ = 0U;

        if (appINSIsAccelAllSixFaceDone()) { // 检查是否所有6个面数据采集完成
            appINSFinishAccelSixFaceCalibrate(); // 计算bias和scale
        }
    }
}

// IMU 中断只负责唤醒 INS 任务
static void appINSIMUDataReadyInterruptCallback(void *owner, bspGPIOInstance_t *gpio_instance)
{
    (void)gpio_instance;
    appINSInstance_t *instance = (appINSInstance_t *)owner;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(instance->task_handle_, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// MAG 中断只置位，真正读数放在任务上下文里做
static void appINSMAGDataReadyInterruptCallback(void *owner, bspGPIOInstance_t *gpio_instance)
{
    (void)gpio_instance;
    appINSInstance_t *instance = (appINSInstance_t *)owner;

    instance->mag_new_data_ready_ = true;
}

// 注册 IMU drdy中断回调
static bool appINSIMUDataReadyInterruptRegister(bspGPIOInstance_t *accel_int, bspGPIOInstance_t *gyro_int)
{
    if (accel_int == NULL && gyro_int == NULL) {
        return false;
    }

    if (accel_int != NULL) {
        bspGPIOIsrCallbackRegister(accel_int, &app_ins_, appINSIMUDataReadyInterruptCallback);
    }

    if (gyro_int != NULL) {
        bspGPIOIsrCallbackRegister(gyro_int, &app_ins_, appINSIMUDataReadyInterruptCallback);
    }

    return true;
}

// 注册 MAG drdy中断回调
static bool appINSMAGDataReadyInterruptRegister(bspGPIOInstance_t *mag_int)
{
    if (mag_int == NULL) {
        return false;
    }

    bspGPIOIsrCallbackRegister(mag_int, &app_ins_, appINSMAGDataReadyInterruptCallback);

    return true;
}

// 对外粗粒度状态
appState_e appINSGetAPPState(void)
{
    if (app_ins_.stage_ == APP_INS_DEGRADED) {
        return APP_STATE_DEGRADED;
    }

    if (app_ins_.stage_ == APP_INS_REINIT) {
        return APP_STATE_REINIT;
    }

    if (app_ins_.mode_ == APP_INS_SERVICE_ACCEL_6FACE_CALIB) {
        return APP_STATE_CALIBRATING;
    }

    switch (app_ins_.stage_) {
    case APP_INS_RUNNING:
        return APP_STATE_NORMAL;
    case APP_INS_IMU_MAG_INIT:
        return APP_STATE_UNINIT;
    case APP_INS_IMU_WARM_UP:
    case APP_INS_IMU_CALIB_GYRO_BIAS:
    case APP_INS_IMU_CALIB_ACCEL:
    case APP_INS_WAIT_MAG_SAMPLE:
    case APP_INS_EKF_INIT:
        return APP_STATE_CALIBRATING;
    default:
        return APP_STATE_UNINIT;
    }
}

// app_ins 轻初始化：
// 只完成 device bring-up、回调注册和上下文初始化
static bool appINSInit(void)
{
    deviceBMI088Config_t bmi088_config;
    deviceIST8310Config_t ist8310_config;

    memset(&app_ins_, 0, sizeof(appINSInstance_t));

    app_ins_.task_handle_ = xTaskGetCurrentTaskHandle();
    app_ins_.mode_ = APP_INS_SERVICE_NORMAL;
    app_ins_.stage_ = APP_INS_IMU_MAG_INIT;
    // 初始化时使用离线标定数据
    app_ins_.calibrate_data_.accel_bias_ms2_[0] = APP_INS_ACCEL_BIAS_X_MS2_OFFLINE;
    app_ins_.calibrate_data_.accel_bias_ms2_[1] = APP_INS_ACCEL_BIAS_Y_MS2_OFFLINE;
    app_ins_.calibrate_data_.accel_bias_ms2_[2] = APP_INS_ACCEL_BIAS_Z_MS2_OFFLINE;
    app_ins_.calibrate_data_.accel_scale_[0] = APP_INS_ACCEL_SCALE_X_OFFLINE;
    app_ins_.calibrate_data_.accel_scale_[1] = APP_INS_ACCEL_SCALE_Y_OFFLINE;
    app_ins_.calibrate_data_.accel_scale_[2] = APP_INS_ACCEL_SCALE_Z_OFFLINE;
    appINSResetAccelSixFaceCalibrate();

    // 初始化 BMI088
    bmi088_config.spi_instance_ = bspBoardGetSPIInstance(BSP_SPI_IMU);
    bmi088_config.accel_cs_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_CS1_ACCEL);
    bmi088_config.gyro_cs_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_CS1_GYRO);
    bmi088_config.delay_us_callback_ = bspDWTDelayUs;
    bmi088_config.mode_ = DEVICE_BMI088_EXTI;
    bmi088_config.name_ = "IMU";

    app_ins_.bmi088_instance_ = deviceBMI088InstanceInit(&bmi088_config);
    if (app_ins_.bmi088_instance_ == NULL) {
        app_ins_.stage_ = APP_INS_REINIT;
        return false;
    }

    app_ins_.accel_int_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_INT1_ACCEL);
    app_ins_.gyro_int_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_INT1_GYRO);

    if (deviceBMI088Init(app_ins_.bmi088_instance_) == DEVICE_BMI088_OK) {
        deviceBMI088Mode_e bmi088_mode;
        deviceBMI088GetMode(app_ins_.bmi088_instance_, &bmi088_mode);
        if (bmi088_mode == DEVICE_BMI088_EXTI || bmi088_mode == DEVICE_BMI088_EXTI_DMA) {
            if (appINSIMUDataReadyInterruptRegister(app_ins_.accel_int_, app_ins_.gyro_int_) == false) {
                app_ins_.stage_ = APP_INS_REINIT;
                return false;
            }

            // BMI088 的 gyro 数据就绪中断需要显式配置
            if (deviceBMI088ConfigGyroDataReadyIT(app_ins_.bmi088_instance_) != DEVICE_BMI088_OK) {
                app_ins_.stage_ = APP_INS_REINIT;
                return false;
            }
        }
    } else {
        app_ins_.stage_ = APP_INS_REINIT;
        return false;
    }

    // 初始化 IST8310
    ist8310_config.i2c_instance_ = bspBoardGetI2CInstance(BSP_I2C_MAG);
    ist8310_config.reset_pin_ = bspBoardGetGPIOInstance(BSP_GPIO_MAG_RSTN);
    ist8310_config.mode_ = DEVICE_IST8310_EXTI;
    ist8310_config.delay_us_callback_ = bspDWTDelayUs;
    ist8310_config.name_ = "MAG";

    app_ins_.ist8310_instance_ = deviceIST8310InstanceInit(&ist8310_config);
    if (app_ins_.ist8310_instance_ == NULL) {
        app_ins_.stage_ = APP_INS_REINIT;
        return false;
    }

    app_ins_.mag_int_ = bspBoardGetGPIOInstance(BSP_GPIO_MAG_DRDY);

    if (deviceIST8310Init(app_ins_.ist8310_instance_) == DEVICE_IST8310_OK) {
        deviceIST8310Mode_e ist8310_mode;
        deviceIST8310GetMode(app_ins_.ist8310_instance_, &ist8310_mode);
        if (ist8310_mode == DEVICE_IST8310_EXTI || ist8310_mode == DEVICE_IST8310_EXTI_DMA) {
            if (appINSMAGDataReadyInterruptRegister(app_ins_.mag_int_) == false) {
                app_ins_.stage_ = APP_INS_REINIT;
                return false;
            }
        }
    } else {
        app_ins_.stage_ = APP_INS_REINIT;
        return false;
    }

    // 初始化完成后先手动触发一次单次测量
    if (deviceIST8310SetSingleMeasureMode(app_ins_.ist8310_instance_) != DEVICE_IST8310_OK) {
        app_ins_.stage_ = APP_INS_REINIT;
        return false;
    }

    // 进入启动阶段
    app_ins_.stage_ = APP_INS_IMU_WARM_UP;
    app_ins_.calibrate_data_.warm_up_tick_ = xTaskGetTickCount();
    return true;
}

// 更新 IMU latest snapshot，并生成一份校准后的 observation
static bool appINSUpdateIMUSample(void)
{
    if (app_ins_.stage_ == APP_INS_REINIT) {
        return false;
    }

    if (deviceBMI088UpdateData(app_ins_.bmi088_instance_) != DEVICE_BMI088_OK) {
        app_ins_.error_count_++;
        app_ins_.stage_ = APP_INS_DEGRADED;
        return false;
    }

    if (deviceBMI088GetDataByOutputFrame(app_ins_.bmi088_instance_, &app_ins_.latest_imu_data_, DEVICE_BMI088_OUTPUT_FRAME_FLU) != DEVICE_BMI088_OK) {
        app_ins_.error_count_++;
        app_ins_.stage_ = APP_INS_DEGRADED;
        return false;
    }

    // 六面标定复用同一条 IMU 更新链推进，只有在accel六面校准模式下才进行校准，否则跳过
    appINSProcessAccelSixFaceCalibrate();

    appINSApplyGyroBias(app_ins_.latest_imu_data_.gyro_rads_, app_ins_.calibrate_data_.gyro_bias_rads_, app_ins_.observation_data_.gyro_corrected_rads_);
    appINSApplyAccelBiasScale(app_ins_.latest_imu_data_.accel_ms2_, app_ins_.calibrate_data_.accel_bias_ms2_, app_ins_.calibrate_data_.accel_scale_, app_ins_.observation_data_.accel_corrected_ms2_);
    app_ins_.observation_data_.imu_timestamp_us_ = bspDWTGetAbsTimeUs();

    return true;
}

// 对于 IST8310 的单次测量模式：
// DRDY -> 任务读数据 -> 再次触发下一次单次测量
static bool appINSUpdateMAGSample(void)
{
    if (app_ins_.stage_ == APP_INS_REINIT || app_ins_.mag_new_data_ready_ == false) {
        return false;
    }

    if (deviceIST8310UpdateData(app_ins_.ist8310_instance_) != DEVICE_IST8310_OK) {
        app_ins_.error_count_++;
        app_ins_.stage_ = APP_INS_DEGRADED;
        return false;
    }

    if (deviceIST8310GetData(app_ins_.ist8310_instance_, &app_ins_.latest_mag_data_) != DEVICE_IST8310_OK) {
        app_ins_.error_count_++;
        app_ins_.stage_ = APP_INS_DEGRADED;
        return false;
    }
    // 暂时没有mag标定
    app_ins_.observation_data_.mag_ut_[0] = app_ins_.latest_mag_data_.mag_ut_[0];
    app_ins_.observation_data_.mag_ut_[1] = app_ins_.latest_mag_data_.mag_ut_[1];
    app_ins_.observation_data_.mag_ut_[2] = app_ins_.latest_mag_data_.mag_ut_[2];
    app_ins_.observation_data_.mag_timestamp_us_ = bspDWTGetAbsTimeUs();
    app_ins_.observation_data_.mag_is_new_ = true;
    appINSUpdateMAGRecordData();

    if (deviceIST8310SetSingleMeasureMode(app_ins_.ist8310_instance_) != DEVICE_IST8310_OK) {
        app_ins_.error_count_++;
        app_ins_.stage_ = APP_INS_DEGRADED;
        return false;
    }

    app_ins_.mag_new_data_ready_ = false;
    app_ins_.mag_data_valid_ = true;
    return true;
}

// INS 启动阶段推进：
// warm-up -> gyro bias -> 等第一帧 mag -> EKF init
static void appINSProcessStartupStage(void)
{
    switch (app_ins_.stage_) {
    case APP_INS_IMU_WARM_UP:
        if (xTaskGetTickCount() - app_ins_.calibrate_data_.warm_up_tick_ <= pdMS_TO_TICKS(APP_INS_CALIB_IMU_WARM_UP_TIME_MS)) {
            return;
        }
        app_ins_.stage_ = APP_INS_IMU_CALIB_GYRO_BIAS;
        // fallthrough
    case APP_INS_IMU_CALIB_GYRO_BIAS:
        if (appINSIsIMUStationary()) {
            app_ins_.calibrate_data_.gyro_bias_sample_count_++;
            app_ins_.calibrate_data_.gyro_bias_accumulate_[0] += app_ins_.latest_imu_data_.gyro_rads_[0];
            app_ins_.calibrate_data_.gyro_bias_accumulate_[1] += app_ins_.latest_imu_data_.gyro_rads_[1];
            app_ins_.calibrate_data_.gyro_bias_accumulate_[2] += app_ins_.latest_imu_data_.gyro_rads_[2];
        } else {
            app_ins_.calibrate_data_.gyro_bias_sample_count_ = 0U;
            memset(app_ins_.calibrate_data_.gyro_bias_accumulate_, 0, sizeof(app_ins_.calibrate_data_.gyro_bias_accumulate_));
        }

        if (app_ins_.calibrate_data_.gyro_bias_sample_count_ >= APP_INS_CALIB_IMU_GYRO_BIAS_SAMPLE_COUNT) {
            app_ins_.calibrate_data_.gyro_bias_rads_[0] = app_ins_.calibrate_data_.gyro_bias_accumulate_[0] / (float)app_ins_.calibrate_data_.gyro_bias_sample_count_;
            app_ins_.calibrate_data_.gyro_bias_rads_[1] = app_ins_.calibrate_data_.gyro_bias_accumulate_[1] / (float)app_ins_.calibrate_data_.gyro_bias_sample_count_;
            app_ins_.calibrate_data_.gyro_bias_rads_[2] = app_ins_.calibrate_data_.gyro_bias_accumulate_[2] / (float)app_ins_.calibrate_data_.gyro_bias_sample_count_;
            app_ins_.calibrate_data_.gyro_bias_ready_ = true;
            app_ins_.stage_ = APP_INS_WAIT_MAG_SAMPLE;
        }
        break;
    case APP_INS_WAIT_MAG_SAMPLE:
        if (app_ins_.mag_data_valid_ == true) {
            app_ins_.stage_ = APP_INS_EKF_INIT;
        }
        break;
    case APP_INS_EKF_INIT:
        app_ins_.stage_ = APP_INS_RUNNING;
        break;
    case APP_INS_RUNNING:
    default:
        break;
    }
}

// EKF 预留入口
static bool appINSRunEKF(void)
{
    return true;
}

// 仅允许在 INS 已经正常运行时手动进入六面标定
bool appINSStartAccelSixFaceCalibrate(void)
{
    if (app_ins_.stage_ != APP_INS_RUNNING || app_ins_.mode_ != APP_INS_SERVICE_NORMAL) {
        return false;
    }

    appINSResetAccelSixFaceCalibrate();
    app_ins_.calibrate_data_.accel_calib_.active_ = true;
    app_ins_.calibrate_data_.accel_bias_scale_ready_ = false;
    app_ins_.mode_ = APP_INS_SERVICE_ACCEL_6FACE_CALIB;
    return true;
}

// accel 标定参数是否已就绪
bool appINSGetAccelCalibrateReady(void)
{
    return app_ins_.calibrate_data_.accel_bias_scale_ready_;
}

// INS 任务主循环：
// IMU 为主时基，MAG 作为异步低频观测
void appINSTaskEntry(void *argument)
{
    (void)argument;

    while (appINSInit() == false) {
        osDelay(pdMS_TO_TICKS(APP_INS_TRY_INIT_DELAY_MS));
    }

    for (;;) {
        uint32_t start_cnt = bspDWTGetCount();
        uint32_t notify_count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(APP_INS_MAX_UPDATE_WAIT_MS));

        if (notify_count > 0U) {
            // 收到 IMU 通知，更新一帧 IMU 数据
            appINSUpdateIMUSample();
        } else {
            // 没有收到 IMU 通知，先记一次超时
            app_ins_.error_count_++;
        }

        // 无论是否收到 IMU 通知，都继续推进 MAG 与 startup
        appINSUpdateMAGSample();
        appINSProcessStartupStage();

        // 触发一次accel标定，为true时不进行accel校准，使用离线值
        static bool accel_calib_test_started = true;
        if (app_ins_.stage_ == APP_INS_RUNNING) {        
            if (app_ins_.stage_ == APP_INS_RUNNING && accel_calib_test_started == false) { 
                appINSStartAccelSixFaceCalibrate();
                accel_calib_test_started = true;
            }
        }

        if (app_ins_.stage_ == APP_INS_RUNNING && app_ins_.mode_ == APP_INS_SERVICE_NORMAL) {
            appINSRunEKF();
        }

        ins_loop_time = bspDWTGetElapsedTimeUs(start_cnt);
        ins_loop_time_max = (ins_loop_time > ins_loop_time_max) ? ins_loop_time : ins_loop_time_max;
    }
}

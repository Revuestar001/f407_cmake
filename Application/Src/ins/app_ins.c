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

#define APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2 (0.1f * STANDARD_GRAVITY_M_S2)
#define APP_INS_CALIB_IMU_WARM_UP_TIME_MS 500U
#define APP_INS_CALIB_IMU_GYRO_BIAS_SAMPLE_COUNT 1200U
#define APP_INS_CALIB_GYRO_TOLERANCE_RADS 0.05f

typedef enum 
{ 
    APP_INS_SERVICE_NORMAL = 0, // 正常运行
    APP_INS_SERVICE_ACCEL_6FACE_CALIB, // 用于accel六面标定
} appINSMode_e;

typedef enum
{
    APP_INS_ACCEL_FACE_NONE = 0,
    APP_INS_ACCEL_FACE_F_UP,
    APP_INS_ACCEL_FACE_F_DOWN,
    APP_INS_ACCEL_FACE_L_UP,
    APP_INS_ACCEL_FACE_L_DOWN,
    APP_INS_ACCEL_FACE_U_UP,
    APP_INS_ACCEL_FACE_U_DOWN,
} appINSAccelCalibrateFace_e; // 仅用于accel六面标定

typedef struct app_ins_accel_6_face_calibrate 
{
    bool active_;
    bool face_done_[6];
    appINSAccelCalibrateFace_e current_face_;
    uint32_t face_sample_count_[6];
    float face_measure_accumulate_[6][3];
} appINSAccelSixFaceCalibrate_t;

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
} appINSStage_e; // app内部状态机

typedef struct app_ins_calibrate_data
{
    TickType_t warm_up_tick_;

    // gyro在线bias校准
    float gyro_bias_rads_[3];
    float gyro_bias_accumulate_[3]; // bias校准累加值
    uint32_t gyro_bias_sample_count_;
    bool gyro_bias_ready_;

    // accel离线校准
    appINSAccelSixFaceCalibrate_t accel_calib_;
    float accel_bias_ms2_[3];
    float accel_scale_[3];
    bool accel_bias_scale_ready_;
} appINSCalibrateData_t;

typedef struct app_ins_observation_data
{
    float accel_corrected_ms2_[3]; // 校准过后的数据
    float gyro_corrected_rads_[3];
    float mag_ut_[3];

    uint64_t timestamp_;
} appINSObservationData_t;

typedef struct app_ins 
{
    // 当前 INS 任务句柄，在 task entry 中绑定
    TaskHandle_t task_handle_;

    deviceBMI088Instance_t *bmi088_instance_;
    deviceIST8310Instance_t *ist8310_instance_;

    // EXTI中断引脚
    bspGPIOInstance_t *accel_int_; 
    bspGPIOInstance_t *gyro_int_;
    bspGPIOInstance_t *mag_int_;

    appINSMode_e mode_;

    // ins数据
    appINSData_t data_;

    // app内部状态
    appINSStage_e stage_;

    // app对外状态
    // appState_e state_;

    // 最新传感器数据，不进行校正
    deviceBMI088Data_t latest_imu_data_;
    deviceIST8310Data_t latest_mag_data_;
    bool mag_sample_valid_; // 表示是否有新的mag数据准备完成
    bool mag_data_ready_; // 表示是否真的拿到至少一帧mag数据

    appINSCalibrateData_t calibrate_data_;
    appINSObservationData_t observation_data_; // 校准后的数据，给EKF使用

    uint8_t error_count_;
} appINSInstance_t;

static appINSInstance_t app_ins_ = {0};
static uint32_t ins_loop_time = 0; 
static uint32_t ins_loop_time_max = 0; 

//
// for calib
//
static void appINSApplyGyroBias(const float gyro[3], const float gyro_bias[3], float gyro_out[3])
{
    gyro_out[0] = gyro[0] - gyro_bias[0];
    gyro_out[1] = gyro[1] - gyro_bias[1];
    gyro_out[2] = gyro[2] - gyro_bias[2];
}

static void appINSApplyAccelBiasScale(const float accel[3], const float accel_bias[3], const float accel_scale[3], float accel_out[3])
{
    accel_out[0] = (accel[0] - accel_bias[0]) * accel_scale[0];
    accel_out[1] = (accel[1] - accel_bias[1]) * accel_scale[1];
    accel_out[2] = (accel[2] - accel_bias[2]) * accel_scale[2];
}

static bool appINSIsIMUStationary()
{
    // 各轴gyro是否足够小
    bool is_stationary = fabsf(app_ins_.latest_imu_data_.gyro_rads_[0]) <= APP_INS_CALIB_GYRO_TOLERANCE_RADS && 
                        fabsf(app_ins_.latest_imu_data_.gyro_rads_[1]) <= APP_INS_CALIB_GYRO_TOLERANCE_RADS &&
                        fabsf(app_ins_.latest_imu_data_.gyro_rads_[2]) <= APP_INS_CALIB_GYRO_TOLERANCE_RADS;
    // 总模长是否接近1g
    is_stationary &= ((powf(app_ins_.latest_imu_data_.accel_ms2_[0], 2.0f) + powf(app_ins_.latest_imu_data_.accel_ms2_[1], 2.0f) + powf(app_ins_.latest_imu_data_.accel_ms2_[2], 2.0f)) <= (powf(APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2, 2.0f) + powf(STANDARD_GRAVITY_M_S2, 2.0f))) && 
                    ((powf(app_ins_.latest_imu_data_.accel_ms2_[0], 2.0f) + powf(app_ins_.latest_imu_data_.accel_ms2_[1], 2.0f) + powf(app_ins_.latest_imu_data_.accel_ms2_[2], 2.0f)) <= (-powf(APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2, 2.0f) + powf(STANDARD_GRAVITY_M_S2, 2.0f)));
    // 某一轴绝对值最大且接近 1g且另外两轴接近 0
    is_stationary &= ((fabsf(app_ins_.latest_imu_data_.accel_ms2_[0] - STANDARD_GRAVITY_M_S2) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2) && (fabsf(app_ins_.latest_imu_data_.accel_ms2_[1]) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2) && (fabsf(app_ins_.latest_imu_data_.accel_ms2_[2]) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2)) ||
                    ((fabsf(app_ins_.latest_imu_data_.accel_ms2_[1] - STANDARD_GRAVITY_M_S2) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2) && (fabsf(app_ins_.latest_imu_data_.accel_ms2_[0]) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2) && (fabsf(app_ins_.latest_imu_data_.accel_ms2_[2]) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2)) || 
                    ((fabsf(app_ins_.latest_imu_data_.accel_ms2_[2] - STANDARD_GRAVITY_M_S2) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2) && (fabsf(app_ins_.latest_imu_data_.accel_ms2_[0]) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2) && (fabsf(app_ins_.latest_imu_data_.accel_ms2_[1]) <= APP_INS_CALIB_IMU_ACCEL_TOLERANCE_MS2));
    return is_stationary;
}

static void appINSIMUDataReadyInterruptCallback(void *owner, bspGPIOInstance_t *gpio_instance)
{
    // 这里暂时不考虑是gyro 还是accel中断
    (void)gpio_instance;
    appINSInstance_t *instance = (appINSInstance_t *)owner;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 唤醒当前任务
    vTaskNotifyGiveFromISR(instance->task_handle_, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void appINSMAGDataReadyInterruptCallback(void *owner, bspGPIOInstance_t *gpio_instance)
{
    (void)gpio_instance;
    appINSInstance_t *instance = (appINSInstance_t *)owner;

    // ins任务应该不以mag的drdy为准进行调度，那这个回调应该干什么？也许就是置一个标志位？
    instance->mag_sample_valid_ = true;
}

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

static bool appINSMAGDataReadyInterruptRegister(bspGPIOInstance_t *mag_int)
{
    if (mag_int == NULL) {
        return false;
    }

    bspGPIOIsrCallbackRegister(mag_int, &app_ins_, appINSMAGDataReadyInterruptCallback);

    return true;
}

appState_e appINSGetAPPState()
{
    switch (app_ins_.stage_) {
    case APP_INS_RUNNING:
        return APP_STATE_NORMAL;
    case APP_INS_DEGRADED:
        return APP_STATE_DEGRADED;
    case APP_INS_REINIT:
        return APP_STATE_REINIT;
    case APP_INS_IMU_MAG_INIT:
        return APP_STATE_UNINIT;
    case APP_INS_IMU_WARM_UP:
    case APP_INS_IMU_CALIB_GYRO_BIAS:
    case APP_INS_IMU_CALIB_ACCEL:
    case APP_INS_EKF_INIT:
        return APP_STATE_CALIBRATING;
    default:
        return APP_STATE_UNINIT;
    }
}

static bool appINSInit()
{   
    memset(&app_ins_, 0, sizeof(appINSInstance_t));

    app_ins_.task_handle_ = xTaskGetCurrentTaskHandle();
    app_ins_.stage_ = APP_INS_IMU_MAG_INIT;

    // 初始化bmi088
    deviceBMI088Config_t bmi088_config;
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
            // 请注意，bmi088的gyro数据就绪中断必须手动配置并开启
            if (deviceBMI088ConfigGyroDataReadyIT(app_ins_.bmi088_instance_) != DEVICE_BMI088_OK) {
                app_ins_.stage_ = APP_INS_REINIT;
                return false;
            }
        }
    } else {
        app_ins_.stage_ = APP_INS_REINIT;
        return false;
    }

    // 初始化ist8310
    deviceIST8310Config_t ist8310_config;
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

    // 初始化完成，手动触发一次ist8310单次测量
    if (deviceIST8310SetSingleMeasureMode(app_ins_.ist8310_instance_) != DEVICE_IST8310_OK) {
        app_ins_.stage_ = APP_INS_REINIT;
        return false;
    }

    // 进入校准
    app_ins_.stage_ = APP_INS_IMU_WARM_UP;
    app_ins_.calibrate_data_.warm_up_tick_ = xTaskGetTickCount();
    return true;
}

static bool appINSUpdateIMUSample()
{
    if (app_ins_.stage_ == APP_INS_REINIT) {
        return false;
    }

    deviceBMI088Status_e bmi088_state;
    bmi088_state = deviceBMI088UpdateData(app_ins_.bmi088_instance_);
    if (bmi088_state != DEVICE_BMI088_OK) {
        app_ins_.error_count_ ++;
        app_ins_.stage_ = APP_INS_DEGRADED;
        return false;
    }

    deviceBMI088GetDataByOutputFrame(app_ins_.bmi088_instance_, &app_ins_.latest_imu_data_, DEVICE_BMI088_OUTPUT_FRAME_FLU);
    appINSApplyGyroBias(app_ins_.latest_imu_data_.gyro_rads_, app_ins_.calibrate_data_.gyro_bias_rads_, app_ins_.observation_data_.gyro_corrected_rads_);
    appINSApplyAccelBiasScale(app_ins_.latest_imu_data_.accel_ms2_, app_ins_.calibrate_data_.accel_bias_ms2_, app_ins_.calibrate_data_.accel_scale_, app_ins_.observation_data_.accel_corrected_ms2_);

    return true;
}

static bool appINSUpdateMAGSample()
{
    if (app_ins_.stage_ == APP_INS_REINIT || app_ins_.mag_sample_valid_ == false) {
        return false;
    }

    if (deviceIST8310UpdateData(app_ins_.ist8310_instance_) != DEVICE_IST8310_OK) {
        app_ins_.error_count_ ++;
        app_ins_.stage_ = APP_INS_DEGRADED;
        return false;
    }

    deviceIST8310GetData(app_ins_.ist8310_instance_, &app_ins_.latest_mag_data_);
    // 暂时没有mag标定
    app_ins_.observation_data_.mag_ut_[0] = app_ins_.latest_mag_data_.mag_ut_[0];
    app_ins_.observation_data_.mag_ut_[1] = app_ins_.latest_mag_data_.mag_ut_[1];
    app_ins_.observation_data_.mag_ut_[2] = app_ins_.latest_mag_data_.mag_ut_[2];

    // 这个处理可能不好
    if (deviceIST8310SetSingleMeasureMode(app_ins_.ist8310_instance_) != DEVICE_IST8310_OK) {
        app_ins_.error_count_ ++;
        app_ins_.stage_ = APP_INS_DEGRADED;
        return false;
    }

    app_ins_.mag_sample_valid_ = false;
    app_ins_.mag_data_ready_ = true;
    return true;
}

static void appINSProcessStartupStage()
{
    switch (app_ins_.stage_) {
        case APP_INS_IMU_WARM_UP:
            if (xTaskGetTickCount() - app_ins_.calibrate_data_.warm_up_tick_ <= pdMS_TO_TICKS(APP_INS_CALIB_IMU_WARM_UP_TIME_MS)) {
                return;
            }
            app_ins_.stage_ = APP_INS_IMU_CALIB_GYRO_BIAS;
            // fallthrough，立刻切换到一下状态
        case APP_INS_IMU_CALIB_GYRO_BIAS:
            if (appINSIsIMUStationary()) {
                app_ins_.calibrate_data_.gyro_bias_sample_count_ ++;
                app_ins_.calibrate_data_.gyro_bias_accumulate_[0] += app_ins_.latest_imu_data_.gyro_rads_[0];
                app_ins_.calibrate_data_.gyro_bias_accumulate_[1] += app_ins_.latest_imu_data_.gyro_rads_[1];
                app_ins_.calibrate_data_.gyro_bias_accumulate_[2] += app_ins_.latest_imu_data_.gyro_rads_[2];
            } else {
                app_ins_.calibrate_data_.gyro_bias_sample_count_ = 0;
                memset(app_ins_.calibrate_data_.gyro_bias_accumulate_, 0, sizeof(float) * 3);
            }

            if (app_ins_.calibrate_data_.gyro_bias_sample_count_ >= APP_INS_CALIB_IMU_GYRO_BIAS_SAMPLE_COUNT) {
                app_ins_.calibrate_data_.gyro_bias_rads_[0] = app_ins_.calibrate_data_.gyro_bias_accumulate_[0] / app_ins_.calibrate_data_.gyro_bias_sample_count_;
                app_ins_.calibrate_data_.gyro_bias_rads_[1] = app_ins_.calibrate_data_.gyro_bias_accumulate_[1] / app_ins_.calibrate_data_.gyro_bias_sample_count_;
                app_ins_.calibrate_data_.gyro_bias_rads_[2] = app_ins_.calibrate_data_.gyro_bias_accumulate_[2] / app_ins_.calibrate_data_.gyro_bias_sample_count_;
                app_ins_.calibrate_data_.gyro_bias_ready_ = true;

                app_ins_.stage_ = APP_INS_WAIT_MAG_SAMPLE;
            }
            break;
        case APP_INS_WAIT_MAG_SAMPLE:
            // 不是已经采集了mag数据吗？这个状态是什么意思？可能给之后的mag校准之类的有用？
            // 等待至少一帧有效mag
            if (app_ins_.mag_data_ready_ == true) {
                app_ins_.stage_ = APP_INS_EKF_INIT;
            }
            break;
        case APP_INS_EKF_INIT:
            // 还没实现
            app_ins_.stage_ = APP_INS_RUNNING;
            break;
        case APP_INS_RUNNING:
        default:
            break;
    }

    return;
}

static bool appINSRunEKF()
{
    // 之后再做
    return true;
}

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
            // 收到任务通知，获取imu数据
            appINSUpdateIMUSample();
        } else {
            // 没有收到imu任务通知，等待中断超时
            app_ins_.error_count_ ++;
        }
        // 无论是否收到通知，都继续接下来的流程
        appINSUpdateMAGSample();

        // 处理启动/初始化流程
        appINSProcessStartupStage();

        // 只有app ins正常运行时，才开始进行EKF计算
        if (app_ins_.stage_ == APP_INS_RUNNING) {
            appINSRunEKF();
        }
        ins_loop_time = bspDWTGetElapsedTimeUs(start_cnt);
        ins_loop_time_max = (ins_loop_time > ins_loop_time_max) ? ins_loop_time : ins_loop_time_max;
    }

}

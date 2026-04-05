#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_board.h"
#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bmi088.h"
#include "app_imu.h"
#include "app_def.h"

#define APP_IMU_CALIBRATE_TIME_MS 3000U
#define APP_IMU_CALIBRATE_GYRO_TOLERANCE_RADS 0.1f

typedef struct app_imu_calibrate_data
{
    float gyro_bias_rads_[3];
} appIMUCalibrateData_t;

typedef struct app_imu 
{
    // 当前 IMU 任务句柄，在 task entry 中绑定
    TaskHandle_t task_handle_;

    deviceBMI088Instance_t *bmi088_instance_;

    // 两个EXTI中断引脚
    bspGPIOInstance_t *accel_int_; 
    bspGPIOInstance_t *gyro_int_;

    appIMUCalibrateData_t calibrate_data_;

    // imu数据
    appIMUData_t data_;

    appState_e state_;
    uint8_t error_count_;
} appIMUInstance_t;

static appIMUInstance_t app_imu_instance_ = {0};

static void appIMUDataReadyInterruptCallback(void *owner, bspGPIOInstance_t *gpio_instance)
{
    // 这里暂时不考虑是gyro 还是accel中断
    (void)gpio_instance;
    appIMUInstance_t *instance = (appIMUInstance_t *)owner;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 唤醒当前任务
    vTaskNotifyGiveFromISR(instance->task_handle_, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static bool appIMUDataReadyInterruptRegister(bspGPIOInstance_t *accel_int, bspGPIOInstance_t *gyro_int)
{
    if (accel_int == NULL && gyro_int == NULL) {
        return false;
    }

    if (accel_int != NULL) {
        bspGPIOIsrCallbackRegister(accel_int, &app_imu_instance_, appIMUDataReadyInterruptCallback);
    }

    if (gyro_int != NULL) {
        bspGPIOIsrCallbackRegister(gyro_int, &app_imu_instance_, appIMUDataReadyInterruptCallback);
    }

    return true;
}

static appState_e appIMUInit()
{   
    // 初始化函数不做memset清零

    deviceBMI088Config_t config;
    config.spi_instance_ = bspBoardGetSPIInstance(BSP_SPI_IMU);
    config.accel_cs_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_CS1_ACCEL);
    config.gyro_cs_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_CS1_GYRO);
    config.delay_us_callback_ = bspDWTDelayUs;
    config.mode_ = DEVICE_BMI088_EXTI;
    config.name_ = "IMU"; 

    app_imu_instance_.bmi088_instance_ = deviceBMI088InstanceInit(&config);
    app_imu_instance_.accel_int_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_INT1_ACCEL);
    app_imu_instance_.gyro_int_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_INT1_GYRO);
    app_imu_instance_.state_ = APP_STATE_UNINIT;
    app_imu_instance_.error_count_ = 0U;

    deviceBMI088Mode_e bmi088_mode;
    // bmi088初始化
    if (deviceBMI088Init(app_imu_instance_.bmi088_instance_) == DEVICE_BMI088_OK) { 
        deviceBMI088GetMode(app_imu_instance_.bmi088_instance_, &bmi088_mode);
        if (bmi088_mode == DEVICE_BMI088_EXTI || bmi088_mode == DEVICE_BMI088_EXTI_DMA) {
            if (appIMUDataReadyInterruptRegister(app_imu_instance_.accel_int_, app_imu_instance_.gyro_int_) == false) {
                app_imu_instance_.state_ = APP_STATE_REINIT;
                return app_imu_instance_.state_;
            }
            // 请注意，bmi088的gyro数据就绪中断必须手动配置并开启
            if (deviceBMI088ConfigGyroDataReadyIT(app_imu_instance_.bmi088_instance_) != DEVICE_BMI088_OK) {
                app_imu_instance_.state_ = APP_STATE_REINIT;
                return app_imu_instance_.state_;
            }
        }
    } else {
        app_imu_instance_.state_ = APP_STATE_REINIT;
        return app_imu_instance_.state_;
    }

    // 进入校准模式
    app_imu_instance_.state_ = APP_STATE_CALIBRATING;
    return app_imu_instance_.state_;
}

static appState_e appIMUCalibrate()
{
    if (app_imu_instance_.state_ != APP_STATE_CALIBRATING) {
        return app_imu_instance_.state_;
    }

    memset(&app_imu_instance_.calibrate_data_, 0, sizeof(appIMUCalibrateData_t));

    uint32_t calibrate_count = 0;
    deviceBMI088Status_e bmi088_state;
    deviceBMI088Data_t bmi088_data;
    uint32_t start_cnt = bspDWTGetCount();
    while (true) {
        bmi088_state = deviceBMI088UpdateData(app_imu_instance_.bmi088_instance_);
        if (bmi088_state != DEVICE_BMI088_OK) {
            app_imu_instance_.error_count_ ++;
            continue;
        }

        deviceBMI088GetDataByOutputFrame(app_imu_instance_.bmi088_instance_, &bmi088_data, DEVICE_BMI088_OUTPUT_FRAME_FLU);
        if (bmi088_data.gyro_rads_[0] > -APP_IMU_CALIBRATE_GYRO_TOLERANCE_RADS && 
            bmi088_data.gyro_rads_[0] < APP_IMU_CALIBRATE_GYRO_TOLERANCE_RADS && 
            bmi088_data.gyro_rads_[1] > -APP_IMU_CALIBRATE_GYRO_TOLERANCE_RADS && 
            bmi088_data.gyro_rads_[1] < APP_IMU_CALIBRATE_GYRO_TOLERANCE_RADS && 
            bmi088_data.gyro_rads_[2] > -APP_IMU_CALIBRATE_GYRO_TOLERANCE_RADS && 
            bmi088_data.gyro_rads_[2] < APP_IMU_CALIBRATE_GYRO_TOLERANCE_RADS) {
            calibrate_count ++;

            app_imu_instance_.calibrate_data_.gyro_bias_rads_[0] += bmi088_data.gyro_rads_[0];
            app_imu_instance_.calibrate_data_.gyro_bias_rads_[1] += bmi088_data.gyro_rads_[1];
            app_imu_instance_.calibrate_data_.gyro_bias_rads_[2] += bmi088_data.gyro_rads_[2];
        } else {
            calibrate_count = 0;
            memset(&app_imu_instance_.calibrate_data_, 0, sizeof(appIMUCalibrateData_t));
            start_cnt = bspDWTGetCount();
            continue;
        }

        if (bspDWTGetElapsedTimeUs(start_cnt) > APP_IMU_CALIBRATE_TIME_MS * 1000) {
            break;
        }
    }

    app_imu_instance_.calibrate_data_.gyro_bias_rads_[0] = app_imu_instance_.calibrate_data_.gyro_bias_rads_[0] / (float)calibrate_count;
    app_imu_instance_.calibrate_data_.gyro_bias_rads_[1] = app_imu_instance_.calibrate_data_.gyro_bias_rads_[1] / (float)calibrate_count;
    app_imu_instance_.calibrate_data_.gyro_bias_rads_[2] = app_imu_instance_.calibrate_data_.gyro_bias_rads_[2] / (float)calibrate_count;

    app_imu_instance_.state_ = APP_STATE_NORMAL;
    return app_imu_instance_.state_;
}

static appState_e appIMUUpdate(appIMUData_t *imu_data_output)
{
    if (app_imu_instance_.state_ != APP_STATE_NORMAL || imu_data_output == NULL) {
        return app_imu_instance_.state_;
    }

    deviceBMI088Status_e bmi088_state;
    bmi088_state = deviceBMI088UpdateData(app_imu_instance_.bmi088_instance_);
    if (bmi088_state != DEVICE_BMI088_OK) {
        app_imu_instance_.error_count_ ++;
        app_imu_instance_.state_ = APP_STATE_DEGRADED;
        return app_imu_instance_.state_;
    }

    deviceBMI088Data_t bmi088_data;
    deviceBMI088GetDataByOutputFrame(app_imu_instance_.bmi088_instance_, &bmi088_data, DEVICE_BMI088_OUTPUT_FRAME_FLU);

    // 请注意，拷贝的是float数组，不是三个字节！
    memcpy(imu_data_output->accel_ms2_, bmi088_data.accel_ms2_, sizeof(imu_data_output->accel_ms2_));
    memcpy(imu_data_output->gyro_rads_, bmi088_data.gyro_rads_, sizeof(imu_data_output->gyro_rads_));

    app_imu_instance_.state_ = APP_STATE_NORMAL;
    return app_imu_instance_.state_;
}

appState_e appIMUGetState()
{
    return app_imu_instance_.state_;
}

void appIMUTaskEntry(void *argument)
{
    (void)argument;

    memset(&app_imu_instance_, 0, sizeof(appIMUInstance_t));
    app_imu_instance_.task_handle_ = xTaskGetCurrentTaskHandle();

    (void)appIMUInit();

    (void)appIMUCalibrate();

    deviceBMI088Mode_e bmi088_mode;
    deviceBMI088GetMode(app_imu_instance_.bmi088_instance_, &bmi088_mode);

    for (;;) {
        switch (bmi088_mode) {
            case DEVICE_BMI088_BLOCKING:
                appIMUUpdate(&app_imu_instance_.data_);
                osDelay(pdMS_TO_TICKS(3U));
                break;
            case DEVICE_BMI088_EXTI:
            case DEVICE_BMI088_EXTI_DMA:
                // 阻塞等待任务通知，pdTRUE表示退出时把通知计数清零，即使中断发生多次（收到多次通知），也只响应一次
                // 返回通知次数
                uint32_t notify_count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(APP_IMU_MAX_UPDATE_WAIT_MS));
                if (notify_count == 0U) {
                    // 没有收到通知，等待中断超时
                    app_imu_instance_.error_count_ ++;
                    continue;
                }
                // 收到任务通知，获取数据
                (void)appIMUUpdate(&app_imu_instance_.data_);
                
                break;
            default:
                break;
        }
        
    }
}

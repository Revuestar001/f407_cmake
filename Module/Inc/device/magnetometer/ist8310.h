#pragma once

#include "bsp_gpio.h"
#include "bsp_i2c.h"

#define DEVICE_IST8310_MAX_INSTANCE_NUM 1U

typedef enum
{
    DEVICE_IST8310_OK = 0,
    DEVICE_IST8310_ERROR,
    DEVICE_IST8310_BUSY,
    DEVICE_IST8310_TIMEOUT,
} deviceIST8310Status_e;

typedef enum
{
    DEVICE_IST8310_BLOCKING = 0,
    DEVICE_IST8310_EXTI,
    DEVICE_IST8310_EXTI_DMA,
} deviceIST8310Mode_e;

typedef struct device_ist8310 deviceIST8310Instance_t;

typedef void (*deviceIST8310DelayUsCallback_f)(uint32_t time_us) ;

typedef struct device_ist8310_config
{
    bspI2CInstance_t *i2c_instance_;
    bspGPIOInstance_t *reset_pin_;

    deviceIST8310Mode_e mode_;

    deviceIST8310DelayUsCallback_f delay_us_callback_;

    const char *name_;
} deviceIST8310Config_t;

typedef struct device_ist8310_data
{
    uint8_t chip_id_;

    int16_t mag_raw_before_transform[3];

    float mag_ut_[3]; // 微特斯拉
} deviceIST8310Data_t;

deviceIST8310Instance_t *deviceIST8310InstanceInit(deviceIST8310Config_t *config);
deviceIST8310Status_e deviceIST8310Init(deviceIST8310Instance_t *instance);
// 请注意，ist8310的数据不会自动连续准备好，必须手动发起测量
deviceIST8310Status_e deviceIST8310SetSingleMeasureMode(deviceIST8310Instance_t *instance);
// 读取并更新一次数据，请注意，ist8310的数据不会自动连续准备好，必须手动发起测量
deviceIST8310Status_e deviceIST8310UpdateData(deviceIST8310Instance_t *instance);
deviceIST8310Status_e deviceIST8310IsDataReady(deviceIST8310Instance_t *instance);
deviceIST8310Status_e deviceIST8310GetData(deviceIST8310Instance_t *instance, deviceIST8310Data_t *data_out);
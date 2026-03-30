#pragma once

#include <stdint.h>

#include "bsp_gpio.h"
#include "bsp_spi.h"

#define DEVICE_BMI088_MAX_INSTANCE_NUM 1
#define DEVICE_BMI088_MAX_BURST_LENGTH 9

typedef enum
{
    DEVICE_BMI088_OK = 0,
    DEVICE_BMI088_ERROR,
    DEVICE_BMI088_BUSY,
    DEVICE_BMI088_TIMEOUT,
} deviceBMI088Status_e;

typedef enum
{
    DEVICE_BMI088_BLOCKING = 0,
    DEVICE_BMI088_EXTI,
    DEVICE_BMI088_EXTI_DMA,
} deviceBMI088Mode_e;

typedef enum
{
    DEVICE_BMI088_NO_ERROR = 0x00,
    DEVICE_BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    DEVICE_BMI088_ACC_PWR_CONF_ERROR = 0x02,
    DEVICE_BMI088_ACC_CONF_ERROR = 0x03,
    DEVICE_BMI088_ACC_SELF_TEST_ERROR = 0x04,
    DEVICE_BMI088_ACC_RANGE_ERROR = 0x05,
    DEVICE_BMI088_INT1_IO_CTRL_ERROR = 0x06,
    DEVICE_BMI088_INT_MAP_DATA_ERROR = 0x07,
    DEVICE_BMI088_GYRO_RANGE_ERROR = 0x08,
    DEVICE_BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    DEVICE_BMI088_GYRO_LPM1_ERROR = 0x0A,
    DEVICE_BMI088_GYRO_CTRL_ERROR = 0x0B,
    DEVICE_BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    DEVICE_BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    DEVICE_BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    DEVICE_BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    DEVICE_BMI088_NO_SENSOR = 0xFF,
} deviceBMI088ErrorCode_e;

typedef struct device_bmi088 deviceBMI088Instance_t;

typedef void (*deviceBMI088DelayUsCallback_f)(uint32_t time_us);

typedef struct device_bmi088_config
{
    bspSPIInstance_t *spi_instance_;
    bspGPIOInstance_t *accel_cs_;
    bspGPIOInstance_t *gyro_cs_;

    deviceBMI088Mode_e mode_;

    deviceBMI088DelayUsCallback_f delay_us_callback_;

    const char *name_;
} deviceBMI088Config_t;

typedef struct device_bmi088_data
{
    uint8_t accel_chip_id_;
    uint8_t gyro_chip_id_;

    int16_t accel_raw_[3];
    int16_t gyro_raw_[3];

    float accel_ms2_[3];
    float gyro_rads_[3];
} deviceBMI088Data_t;

// 只是初始化实例，绑定板上资源
deviceBMI088Instance_t *deviceBMI088InstanceInit(const deviceBMI088Config_t *config);
// 初始化对应的BMI088设备
deviceBMI088Status_e deviceBMI088Init(deviceBMI088Instance_t *instance);
// 读取并更新一次数据
deviceBMI088Status_e deviceBMI088UpdateData(deviceBMI088Instance_t *instance);
// 获取当前传感器数据
deviceBMI088Status_e deviceBMI088GetData(const deviceBMI088Instance_t *instance, deviceBMI088Data_t *data_out);
// 配置bmi088的gyro数据就绪中断
deviceBMI088Status_e deviceBMI088ConfigGyroDataReadyIT(deviceBMI088Instance_t *instance);
deviceBMI088Status_e deviceBMI088GetMode(deviceBMI088Instance_t *instance, deviceBMI088Mode_e *mode_out);
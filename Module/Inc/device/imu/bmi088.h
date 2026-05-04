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

typedef enum
{
    DEVICE_BMI088_OUTPUT_FRAME_FLU = 0,
    DEVICE_BMI088_OUTPUT_FRAME_FRD,
} deviceBMI088OutputFrame_e;

typedef struct device_bmi088 deviceBMI088Instance_t;

typedef void (*deviceBMI088DelayUsCallback_f)(uint32_t time_us);
typedef void (*deviceBMI088DMAXferCpltNotifyCallback_f)(void);
typedef void (*deviceBMI088DMAXferErrorNotifyCallback_f)(void);

typedef struct device_bmi088_config
{
    bspSPIInstance_t *spi_instance_;
    bspGPIOInstance_t *accel_cs_;
    bspGPIOInstance_t *gyro_cs_;

    deviceBMI088Mode_e mode_;

    deviceBMI088DelayUsCallback_f delay_us_callback_;

    // 请注意这两个回调是在ISR中使用的，请务必保证只做最简单的处理！！
    deviceBMI088DMAXferCpltNotifyCallback_f dma_xfer_cplt_notify_callback_from_isr_;
    deviceBMI088DMAXferErrorNotifyCallback_f dma_xfer_error_notify_callback_from_isr_;

    const char *name_;
} deviceBMI088Config_t;

typedef struct device_bmi088_data
{
    int16_t accel_raw_[3]; // bmi088原生轴数据！
    int16_t gyro_raw_[3]; // bmi088原生轴数据！

    float accel_ms2_[3]; // 经过安装映射到FLU后的数据
    float gyro_rads_[3]; // 经过安装映射到FLU后的数据
} deviceBMI088Data_t;

// 只是初始化实例，绑定板上资源
deviceBMI088Instance_t *deviceBMI088InstanceInit(const deviceBMI088Config_t *config);
// 初始化对应的BMI088设备
deviceBMI088Status_e deviceBMI088Init(deviceBMI088Instance_t *instance);

// 读取并更新一次数据
// 同步读取，只允许在DEVICE_BMI088_BLOCKING 和DEVICE_BMI088_EXTI模式下使用！
deviceBMI088Status_e deviceBMI088UpdateData(deviceBMI088Instance_t *instance);
// 使用DMA传输更新数据,开启一次DMA传输
deviceBMI088Status_e deviceBMI088UpdateDataDMAStart(deviceBMI088Instance_t *instance);
// 开启DMA传输后，调用该函数以推进DMA传输流程，若传输完成则更新IMU数据
deviceBMI088Status_e deviceBMI088UpdateDataDMAProcess(deviceBMI088Instance_t *instance, bool *new_data_ready);

// 获取当前传感器数据，对于c板来说，bmi088直接输出的accel和gyro本来就定义在FLU下，注意F是robomaster字体为正时指向前部，即uart2标识-->uart1标识
deviceBMI088Status_e deviceBMI088GetData(const deviceBMI088Instance_t *instance, deviceBMI088Data_t *data_out);
// 获取转到对应机体坐标系下的当前传感器数据，默认bmi088与FLU对齐
deviceBMI088Status_e deviceBMI088GetDataByOutputFrame(const deviceBMI088Instance_t *instance, deviceBMI088Data_t *data_out, deviceBMI088OutputFrame_e frame);
// 配置bmi088的gyro数据就绪中断
deviceBMI088Status_e deviceBMI088ConfigGyroDataReadyIT(deviceBMI088Instance_t *instance);
deviceBMI088Status_e deviceBMI088GetMode(deviceBMI088Instance_t *instance, deviceBMI088Mode_e *mode_out);
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ist8310.h"
#include "ist8310_register_address.h"
#include "bsp_gpio.h"
#include "bsp_i2c.h"

#define DEVICE_IST8310_CROSS_AXIS_COMP_M (3.0f / 20.0f)
#define DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT 50.0f
#define DEVICE_IST8310_MILLI_GAUSS_TO_UT 0.1f

typedef struct device_ist8310 
{
    bspI2CInstance_t *i2c_instance_;
    bspGPIOInstance_t *reset_pin_;

    deviceIST8310Mode_e mode_;

    deviceIST8310DelayUsCallback_f delay_us_callback_;

    const char *name_;

    bool is_initialized_; // 这个指的是具体设备是否初始化，不是指实例
    bool in_single_measure_mode_; // ist8310不会连续测量，必须手动进入单次测量模式手动发起一次测量

    float A_ij_[IST8310_CROSS_AXIS_DATA_LEN / 2]; // 交叉轴补偿矩阵元素

    deviceIST8310Data_t data_;
} deviceIST8310Instance_t;

static uint8_t ist8310_memory_index_ = 0;
static deviceIST8310Instance_t ist8310_instance_memory_[DEVICE_IST8310_MAX_INSTANCE_NUM];

// 计算交叉轴变换矩阵A
static deviceIST8310Status_e deviceIST8310ComputeCrossAxisTransformationMatrix(deviceIST8310Instance_t *instance, uint8_t *data_raw)
{
    if (instance == NULL || data_raw == NULL) {
        return DEVICE_IST8310_ERROR;
    }

    float data_decoded[IST8310_CROSS_AXIS_DATA_LEN / 2];
    for (size_t i = 0; i < (IST8310_CROSS_AXIS_DATA_LEN / 2); i ++) {
        data_decoded[i] = (float)(((int16_t)(((uint16_t)data_raw[2 * i + 1] << 8) | data_raw[2 * i])) * DEVICE_IST8310_CROSS_AXIS_COMP_M);
    }

    const float x11 = data_decoded[0];
    const float x12 = data_decoded[1];
    const float x13 = data_decoded[2];
    const float x21 = data_decoded[3];
    const float x22 = data_decoded[4];
    const float x23 = data_decoded[5];
    const float x31 = data_decoded[6];
    const float x32 = data_decoded[7];
    const float x33 = data_decoded[8];

    const float det =
        x11 * (x22 * x33 - x23 * x32) -
        x12 * (x21 * x33 - x23 * x31) +
        x13 * (x21 * x32 - x22 * x31);

    if (det > -1.0e-6f && det < 1.0e-6f) {
        return DEVICE_IST8310_ERROR;
    }

    const float inv_det = 1.0f / det;
    instance->A_ij_[0] = (x22 * x33 - x23 * x32) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;
    instance->A_ij_[1] = (x13 * x32 - x12 * x33) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;
    instance->A_ij_[2] = (x12 * x23 - x13 * x22) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;
    instance->A_ij_[3] = (x23 * x31 - x21 * x33) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;
    instance->A_ij_[4] = (x11 * x33 - x13 * x31) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;
    instance->A_ij_[5] = (x13 * x21 - x11 * x23) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;
    instance->A_ij_[6] = (x21 * x32 - x22 * x31) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;
    instance->A_ij_[7] = (x12 * x31 - x11 * x32) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;
    instance->A_ij_[8] = (x11 * x22 - x12 * x21) * inv_det * DEVICE_IST8310_CROSS_AXIS_TRANSFERMATION_COEFFICIENT;

    return DEVICE_IST8310_OK;
}

deviceIST8310Instance_t *deviceIST8310InstanceInit(deviceIST8310Config_t *config)
{
    if (config == NULL || 
        config->i2c_instance_ == NULL || 
        config->reset_pin_ == NULL || 
        config->delay_us_callback_ == NULL) {
        return NULL;
    }

    if (ist8310_memory_index_ >= DEVICE_IST8310_MAX_INSTANCE_NUM) {
        return NULL;
    }

    deviceIST8310Instance_t *instance = &ist8310_instance_memory_[ist8310_memory_index_];
    memset(instance, 0, sizeof(deviceIST8310Instance_t));
    instance->i2c_instance_ = config->i2c_instance_;
    instance->reset_pin_ = config->reset_pin_;
    instance->mode_ = config->mode_;
    instance->delay_us_callback_ = config->delay_us_callback_;
    instance->name_ = config->name_;

    instance->is_initialized_ = false;
    instance->in_single_measure_mode_ = false;

    ist8310_memory_index_ ++;

    return instance;
}

deviceIST8310Status_e deviceIST8310Init(deviceIST8310Instance_t *instance)
{
    if (instance == NULL) {
        return DEVICE_IST8310_ERROR;
    }

    instance->is_initialized_ = false;
    instance->in_single_measure_mode_ = false;

    // 硬复位确保进入 stand-by 模式
    bspGPIOWriteLogic(instance->reset_pin_, true);
    instance->delay_us_callback_(50000U);
    bspGPIOWriteLogic(instance->reset_pin_, false);
    instance->delay_us_callback_(50000U);

    // 检查芯片 ID
    uint8_t chip_id;
    if (bspI2CMemoryRead(instance->i2c_instance_, 
        IST8310_I2C_ADDRESS_7BIT, 
        IST8310_WHO_AM_I_ADD, 
        BSP_I2C_MEM_ADD_SIZE_8BIT, 
        &chip_id, 
        1U) != BSP_I2C_OK || 
        chip_id != IST8310_CHIP_ID_VAL) {
        return DEVICE_IST8310_ERROR;
    }
    instance->data_.chip_id_ = chip_id;

    // 配置平均滤波 16 采样，低噪声
    uint8_t config = IST8310_AVGCNTL_XYZ_SIXTEEN_SAMPLE;
    if (bspI2CMemoryWrite(instance->i2c_instance_, IST8310_I2C_ADDRESS_7BIT, IST8310_AVGCNTL_ADD, BSP_I2C_MEM_ADD_SIZE_8BIT, &config, 1U) != BSP_I2C_OK) {
        return DEVICE_IST8310_ERROR;
    }

    // 配置脉冲宽度
    config = IST8310_PDCNTL_NORMAL_PULSE;
    if (bspI2CMemoryWrite(instance->i2c_instance_, IST8310_I2C_ADDRESS_7BIT, IST8310_PDCNTL_ADD, BSP_I2C_MEM_ADD_SIZE_8BIT, &config, 1U) != BSP_I2C_OK) {
        return DEVICE_IST8310_ERROR;
    }

    if (instance->mode_ == DEVICE_IST8310_EXTI || instance->mode_ == DEVICE_IST8310_EXTI_DMA) {
        config = IST8310_CNTL2_DRDY_ENABLE_ACTIVE_HIGH;
    } else {
        config = IST8310_CNTL2_DRDY_DISABLE;
    }
    // 配置 DRDY 中断和中断引脚电平
    // 其实默认已配置
    if (bspI2CMemoryWrite(instance->i2c_instance_, IST8310_I2C_ADDRESS_7BIT, IST8310_CNTL2_ADD, BSP_I2C_MEM_ADD_SIZE_8BIT, &config, 1U) != BSP_I2C_OK) {
        return DEVICE_IST8310_ERROR;
    }

    // 读取交叉轴补偿原始数据，初始化时读一次即可
    uint8_t cross_axis_compensation_raw_data[IST8310_CROSS_AXIS_DATA_LEN];
    if (bspI2CMemoryRead(instance->i2c_instance_, 
        IST8310_I2C_ADDRESS_7BIT, 
        IST8310_CROSS_AXIS_X11L_ADD, 
        BSP_I2C_MEM_ADD_SIZE_8BIT, 
        cross_axis_compensation_raw_data, 
        IST8310_CROSS_AXIS_DATA_LEN) != BSP_I2C_OK) {
        return DEVICE_IST8310_ERROR;
    }

    // 计算交叉轴变换矩阵 A
    if (deviceIST8310ComputeCrossAxisTransformationMatrix(instance, cross_axis_compensation_raw_data) != DEVICE_IST8310_OK) {
        return DEVICE_IST8310_ERROR;
    }

    instance->is_initialized_ = true;
    return DEVICE_IST8310_OK;
}

// 请注意，ist8310的数据不会自动连续准备好，必须手动发起测量
deviceIST8310Status_e deviceIST8310SetSingleMeasureMode(deviceIST8310Instance_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false || instance->in_single_measure_mode_ == true) {
        return DEVICE_IST8310_ERROR;
    }

    uint8_t config = IST8310_CNTL1_SINGLE_MEASURE_MODE;
    if (bspI2CMemoryWrite(instance->i2c_instance_, IST8310_I2C_ADDRESS_7BIT, IST8310_CNTL1_ADD, BSP_I2C_MEM_ADD_SIZE_8BIT, &config, 1U) != BSP_I2C_OK) {
        return DEVICE_IST8310_ERROR;
    }

    instance->in_single_measure_mode_ = true;
    return DEVICE_IST8310_OK;
}

// 读取并更新一次数据，请注意，ist8310的数据不会自动连续准备好，必须手动发起测量
deviceIST8310Status_e deviceIST8310UpdateData(deviceIST8310Instance_t *instance)
{
    // 不在单次测量模式下，不允许读取数据
    if (instance == NULL || instance->is_initialized_ == false || instance->in_single_measure_mode_ == false) {
        return DEVICE_IST8310_ERROR;
    }

    // 只判断in_single_measure_mode_不够，因为数据需要5-6ms时间来准备
    // blocking模式需要检查数据是否准备完成，exti模式由drdy中断提示
    if (instance->mode_ == DEVICE_IST8310_BLOCKING) {
        deviceIST8310Status_e status = deviceIST8310IsDataReady(instance);
        if (status != DEVICE_IST8310_OK) {
            return status;
        }
    }
    
    uint8_t data_raw[IST8310_DATA_LEN];
    if (bspI2CMemoryRead(instance->i2c_instance_, 
        IST8310_I2C_ADDRESS_7BIT, 
        IST8310_DATAXL_ADD, 
        BSP_I2C_MEM_ADD_SIZE_8BIT, 
        data_raw, 
        IST8310_DATA_LEN) != BSP_I2C_OK) {
        return DEVICE_IST8310_ERROR;
    }
    // 读取完成后清除标志位
    instance->in_single_measure_mode_ = false;

    for (size_t i = 0; i < (IST8310_DATA_LEN / 2); i ++) {
        instance->data_.mag_raw_before_transform[i] = (int16_t)(((uint16_t)data_raw[2 * i + 1] << 8) | data_raw[2 * i]);
    }

    float mag_after_transform_mg[3] = {0.0f};
    for (size_t row = 0; row < 3U; row++) {
        mag_after_transform_mg[row] =
            instance->A_ij_[row * 3U] * (float)instance->data_.mag_raw_before_transform[0] +
            instance->A_ij_[row * 3U + 1U] * (float)instance->data_.mag_raw_before_transform[1] +
            instance->A_ij_[row * 3U + 2U] * (float)instance->data_.mag_raw_before_transform[2];
    }

    for (size_t i = 0; i < 3U; i++) {
        instance->data_.mag_ut_[i] = mag_after_transform_mg[i] * DEVICE_IST8310_MILLI_GAUSS_TO_UT;
    }

    return DEVICE_IST8310_OK;
}

// 这个函数内部不修改in_single_measure_mode_，虽然我不知道这样好不好
deviceIST8310Status_e deviceIST8310IsDataReady(deviceIST8310Instance_t *instance)
{
    if (instance == NULL) {
        return DEVICE_IST8310_ERROR;
    }

    uint8_t data;
    if (bspI2CMemoryRead(instance->i2c_instance_, 
        IST8310_I2C_ADDRESS_7BIT, 
        IST8310_STAT1_ADD, 
        BSP_I2C_MEM_ADD_SIZE_8BIT, 
        &data, 
        1U) != BSP_I2C_OK) {
        return DEVICE_IST8310_ERROR;
    }

    if ((data & IST8310_STAT1_DATA_READY_VAL) == 0U) {
        // STAT1 按位判断，DRDY bit 没置位就说明数据还没准备好
        return DEVICE_IST8310_BUSY;
    }

    return DEVICE_IST8310_OK;
}

deviceIST8310Status_e deviceIST8310GetData(deviceIST8310Instance_t *instance, deviceIST8310Data_t *data_out)
{
    if (instance == NULL || instance->is_initialized_ == false || data_out == NULL) {
        return DEVICE_IST8310_ERROR;
    }

    *data_out = instance->data_;

    return DEVICE_IST8310_OK;
}


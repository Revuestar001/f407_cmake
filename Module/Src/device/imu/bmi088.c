#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bmi088.h"
#include "bmi088_register_address.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "general_math.h"

// bmi088读写，读的时候地址高位一定为1，写的时候地址高位一定为0
#define DEVICE_BMI088_REG_TO_WRITE_CMD(reg)  ((reg) & 0x7FU)
#define DEVICE_BMI088_REG_TO_READ_CMD(reg)   ((reg) | 0x80U)

#define DEVICE_BMI088_BUS_IDLE_DELAY_US           10U
#define DEVICE_BMI088_POWER_ON_DELAY_US           30000U
#define DEVICE_BMI088_POWER_MODE_SWITCH_DELAY_US  450U
#define DEVICE_BMI088_ACCEL_CONF_DEFAULT          0xAAU // 注意这个是normal滤波+400Hz更新，不是默认配置
#define DEVICE_BMI088_ACCEL_SENSITIVITY_6G        (32768.0f / 6.0f)
#define DEVICE_BMI088_GYRO_SENSITIVITY_2000DPS    16.384f

typedef enum
{
    DEVICE_BMI088_AXIS_X = 0,
    DEVICE_BMI088_AXIS_Y,
    DEVICE_BMI088_AXIS_Z,
} deviceBMI088Axis_e; // 枚举bmi088的x y z轴

typedef struct device_bmi088_install_transform
{
    deviceBMI088Axis_e axis_map[3]; // 坐标轴映射，表示机体坐标系的每一个轴，对应到bmi088的哪一个轴
    int8_t axis_sign[3]; // 轴的正负
} deviceBMI088InstallTransform_t; // 定义bmi088安装变换,暂时先不暴露，因为这个其实和板子固定了，从外部传入容易出错

typedef enum
{
    DEVICE_BMI088_TARGET_NONE = 0,
    DEVICE_BMI088_TARGET_ACCEL,
    DEVICE_BMI088_TARGET_GYRO,
} deviceBMI088Target_e;

typedef enum
{
    DEVICE_BMI088_DMA_XFER_IDLE = 0, // 空闲，无DMA传输任务
    DEVICE_BMI088_DMA_XFER_ACCESS_ACCEL,
    DEVICE_BMI088_DMA_XFER_ACCESS_GYRO,
    DEVICE_BMI088_DMA_XFER_RAW_DATA_READY,
    DEVICE_BMI088_DMA_XFER_ERROR,
} deviceBMI088DMAXferState_e;

// 隐藏结构体
typedef struct device_bmi088
{
    bspSPIInstance_t *spi_instance_;
    bspGPIOInstance_t *accel_cs_;
    bspGPIOInstance_t *gyro_cs_;

    deviceBMI088Mode_e mode_;
    deviceBMI088InstallTransform_t install_transform_;

    deviceBMI088DelayUsCallback_f delay_us_callback_;

    const char *name_;

    bool is_initialized_; // 这个指的是具体设备是否初始化，不是指实例

    deviceBMI088Target_e selected_target_; // 被选中的设备,用这个判断或许不可靠

    volatile deviceBMI088DMAXferState_e dma_xfer_state_;
    uint8_t dma_xfer_tx_buffer_[DEVICE_BMI088_MAX_BURST_LENGTH + 2U];
    uint8_t dma_xfer_rx_buffer_[DEVICE_BMI088_MAX_BURST_LENGTH + 2U];
    uint8_t accel_raw_uint8_buffer[ACC_XYZ_LEN];
    uint8_t gyro_raw_uint8_buffer[GYRO_XYZ_LEN];
    deviceBMI088DMAXferCpltNotifyCallback_f dma_xfer_cplt_notify_callback_from_isr_;
    deviceBMI088DMAXferErrorNotifyCallback_f dma_xfer_error_notify_callback_from_isr_;

    deviceBMI088Data_t data_;
} deviceBMI088Instance_t;

static uint8_t bmi088_memory_index_ = 0;
static deviceBMI088Instance_t bmi088_instance_memory_[DEVICE_BMI088_MAX_INSTANCE_NUM];

static bool isModeValid(deviceBMI088Mode_e mode)
{   
    switch (mode) {
        case DEVICE_BMI088_BLOCKING:
        case DEVICE_BMI088_EXTI:
        case DEVICE_BMI088_EXTI_DMA:
            return true;
        default:
            return false;
    }
}

static bool changeSPIWorkModeByBMI088Config(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    if (isModeValid(instance->mode_) == false) {
        return false;
    }

    switch (instance->mode_) {
        case DEVICE_BMI088_BLOCKING:
        case DEVICE_BMI088_EXTI:
            if (bspSPIChangeWorkMode(instance->spi_instance_, BSP_SPI_WORK_MODE_BLOCKING) != BSP_SPI_OK) {
                return false;
            }
            break;
        case DEVICE_BMI088_EXTI_DMA:
            if (bspSPIChangeWorkMode(instance->spi_instance_, BSP_SPI_WORK_MODE_DMA) != BSP_SPI_OK) {
                return false;
            }
            break;
        default:
            return false;
    }

    return true;
}

// 请注意，最好不要在非初始化阶段调用延时函数
static void deviceBMI088DelayUs(deviceBMI088Instance_t *instance, uint32_t time_us)
{
    if (instance == NULL || instance->delay_us_callback_ == NULL || time_us == 0U) {
        return;
    }

    instance->delay_us_callback_(time_us);
}

static int16_t parseRawDataOneAxis(uint8_t data_lsb, uint8_t data_msb)
{
    return (int16_t)(((uint16_t)data_msb << 8U) | data_lsb);
}

//
// 操作CS的helper函数
//
static void deviceBMI088DeselectAllTarget(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return;
    }

    // 拉高所有CS
    bspGPIOWriteLogic(instance->accel_cs_, false);
    bspGPIOWriteLogic(instance->gyro_cs_, false);

    instance->selected_target_ = DEVICE_BMI088_TARGET_NONE;
}

static void deviceBMI088DeselectTarget(deviceBMI088Instance_t *instance, deviceBMI088Target_e target)
{
    if (instance == NULL || target == DEVICE_BMI088_TARGET_NONE) {
        return;
    }

    switch (target) {
        case DEVICE_BMI088_TARGET_ACCEL:
            bspGPIOWriteLogic(instance->accel_cs_, false);
            break;
        case DEVICE_BMI088_TARGET_GYRO:
            bspGPIOWriteLogic(instance->gyro_cs_, false);
            break;
        default:
            break;
    }

    instance->selected_target_ = DEVICE_BMI088_TARGET_NONE;
}

// 操作选中CS
static void deviceBMI088SelectTarget(deviceBMI088Instance_t *instance, deviceBMI088Target_e target)
{
    if (instance == NULL || target == DEVICE_BMI088_TARGET_NONE) {
        return;
    }

    switch (target) {
        case DEVICE_BMI088_TARGET_ACCEL:
            bspGPIOWriteLogic(instance->accel_cs_, true);
            break;
        case DEVICE_BMI088_TARGET_GYRO:
            bspGPIOWriteLogic(instance->gyro_cs_, true);
            break;
        default:
            break;
    }

    instance->selected_target_ = target;
}

//
// 按target读写寄存器
//
// 写单个字节,请注意，这里会操作CS
static bool deviceBMI088WriteSingleRegister(deviceBMI088Instance_t *instance,
                                            deviceBMI088Target_e target,
                                            uint8_t register_address,
                                            uint8_t register_value)
{
    if (instance == NULL || target == DEVICE_BMI088_TARGET_NONE) {
        return false;
    }

    uint8_t tx_buffer[2] = {0};
    tx_buffer[0] = DEVICE_BMI088_REG_TO_WRITE_CMD(register_address);
    tx_buffer[1] = register_value;

    bspSPIStatus_e spi_status;

    deviceBMI088DeselectAllTarget(instance);
    // 选中
    deviceBMI088SelectTarget(instance, target);

    spi_status = bspSPITransmit(instance->spi_instance_, tx_buffer, sizeof(tx_buffer));

    // 这个是阻塞SPI才能这么干，一定要注意！！！！！！别忘记改
    deviceBMI088DeselectTarget(instance, target);

    return spi_status == BSP_SPI_OK;
}

// 读多个字节，地址自增读寄存器，因为accel gyro分别需要读六个寄存器
static bool deviceBMI088ReadRegisters(deviceBMI088Instance_t *instance,
                                      deviceBMI088Target_e target,
                                      uint8_t register_address,
                                      uint8_t *rx_buffer,
                                      uint16_t data_length)
{
    if (instance == NULL ||
        target == DEVICE_BMI088_TARGET_NONE ||
        rx_buffer == NULL ||
        data_length == 0 ||
        data_length > DEVICE_BMI088_MAX_BURST_LENGTH) {
        return false;
    }

    // 多出两个字节，分别给register_address和accel的dummy byte预留空间
    uint8_t tx_buffer[DEVICE_BMI088_MAX_BURST_LENGTH + 2U] = {0};
    memset(tx_buffer, DUMMY_BYTE, sizeof(tx_buffer));
    tx_buffer[0] = DEVICE_BMI088_REG_TO_READ_CMD(register_address);
    uint8_t rx_buffer_temp[DEVICE_BMI088_MAX_BURST_LENGTH + 2U] = {0};

    uint16_t transfer_length = 0;
    if (target == DEVICE_BMI088_TARGET_ACCEL) {
        // 读取ACCEL，会在寄存器地址后多一个无效字节，然后才是有效数据
        transfer_length = data_length + 1U + 1U;
    } else if (target == DEVICE_BMI088_TARGET_GYRO) {
        // 读取GYRO，在寄存器地址后就是有效数据
        transfer_length = data_length + 1U;
    }

    bspSPIStatus_e spi_status;

    deviceBMI088DeselectAllTarget(instance);
    // 选中
    deviceBMI088SelectTarget(instance, target);

    spi_status = bspSPITransmitReceive(instance->spi_instance_, tx_buffer, rx_buffer_temp, transfer_length);

    // 这个是阻塞SPI才能这么干，一定要注意！！！！！！别忘记改
    deviceBMI088DeselectTarget(instance, target);

    if (spi_status != BSP_SPI_OK) {
        return false;
    }

    if (target == DEVICE_BMI088_TARGET_ACCEL) {
        // 读取ACCEL，会在寄存器地址后多一个无效字节，然后才是有效数据
        memcpy(rx_buffer, &rx_buffer_temp[2], data_length);
    } else if (target == DEVICE_BMI088_TARGET_GYRO) {
        // 读取GYRO，在寄存器地址后就是有效数据
        memcpy(rx_buffer, &rx_buffer_temp[1], data_length);
    }

    return true;
}

//
// 初始化配置
//
static bool deviceBMI088GetChipID(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    bool state = true;

    uint8_t chip_id[2] = {0};
    state &= deviceBMI088ReadRegisters(instance,
                                       DEVICE_BMI088_TARGET_ACCEL,
                                       ACC_CHIP_ID_ADDR,
                                       &chip_id[0],
                                       1U);
    if (state == false || chip_id[0] != ACC_CHIP_ID_VAL) {
        return false;
    }

    // 仍然是阻塞才能这么写！！！！！！！！！
    state &= deviceBMI088ReadRegisters(instance,
                                       DEVICE_BMI088_TARGET_GYRO,
                                       GYRO_CHIP_ID_ADDR,
                                       &chip_id[1],
                                       1U);

    if (state == false || chip_id[1] != GYRO_CHIP_ID_VAL) {
        return false;
    }

    return true;
}

static bool deviceBMI088AccelSwitchToSPI(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    uint8_t dummy_data = 0;

    return deviceBMI088ReadRegisters(instance,
                                     DEVICE_BMI088_TARGET_ACCEL,
                                     ACC_CHIP_ID_ADDR,
                                     &dummy_data,
                                     1U);
}

static bool deviceBMI088ConfigAccel(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    bool state = true;

    // 把 accel 的 power-save / filter配置切到 active
    state &= deviceBMI088WriteSingleRegister(instance, DEVICE_BMI088_TARGET_ACCEL, ACC_PWR_CONF_ADDR, ACC_PWR_CONF_ACT);
    deviceBMI088DelayUs(instance, DEVICE_BMI088_POWER_MODE_SWITCH_DELAY_US);
    // 让 accel 进入正常工作、开始正常出数
    state &= deviceBMI088WriteSingleRegister(instance, DEVICE_BMI088_TARGET_ACCEL, ACC_PWR_CTRL_ADDR, ACC_PWR_CTRL_ON);
    deviceBMI088DelayUs(instance, DEVICE_BMI088_POWER_MODE_SWITCH_DELAY_US);

    // ACC_CONF 需要把保留位一起按 datasheet 要求写进去
    state &= deviceBMI088WriteSingleRegister(instance, DEVICE_BMI088_TARGET_ACCEL, ACC_CONF_ADDR, DEVICE_BMI088_ACCEL_CONF_DEFAULT);
    state &= deviceBMI088WriteSingleRegister(instance, DEVICE_BMI088_TARGET_ACCEL, ACC_RANGE_ADDR, ACC_RANGE_6G);

    return state;
}

static bool deviceBMI088ConfigGyro(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    bool state = true;

    state &= deviceBMI088WriteSingleRegister(instance, DEVICE_BMI088_TARGET_GYRO, GYRO_RANGE_ADDR, GYRO_RANGE_2000_DEG_S);
    state &= deviceBMI088WriteSingleRegister(instance, DEVICE_BMI088_TARGET_GYRO, GYRO_BANDWIDTH_ADDR, GYRO_ODR_400Hz_BANDWIDTH_47Hz);

    return state;
}

static void deviceBMI088MapDataByInstallTransform(deviceBMI088Instance_t  *instance, const float data_raw[3], float data_mapped[3])
{
    if (instance == NULL) {
        return;
    }

    data_mapped[0] = (float)instance->install_transform_.axis_sign[0] * data_raw[instance->install_transform_.axis_map[0]];
    data_mapped[1] = (float)instance->install_transform_.axis_sign[1] * data_raw[instance->install_transform_.axis_map[1]];
    data_mapped[2] = (float)instance->install_transform_.axis_sign[2] * data_raw[instance->install_transform_.axis_map[2]];
}

static bool deviceBMI088ParseRawData(deviceBMI088Instance_t  *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    int16_t accel_raw[3] = {0};
    int16_t gyro_raw[3] = {0};

    accel_raw[0] = parseRawDataOneAxis(instance->accel_raw_uint8_buffer[0], instance->accel_raw_uint8_buffer[1]);
    accel_raw[1] = parseRawDataOneAxis(instance->accel_raw_uint8_buffer[2], instance->accel_raw_uint8_buffer[3]);
    accel_raw[2] = parseRawDataOneAxis(instance->accel_raw_uint8_buffer[4], instance->accel_raw_uint8_buffer[5]);

    gyro_raw[0] = parseRawDataOneAxis(instance->gyro_raw_uint8_buffer[0], instance->gyro_raw_uint8_buffer[1]);
    gyro_raw[1] = parseRawDataOneAxis(instance->gyro_raw_uint8_buffer[2], instance->gyro_raw_uint8_buffer[3]);
    gyro_raw[2] = parseRawDataOneAxis(instance->gyro_raw_uint8_buffer[4], instance->gyro_raw_uint8_buffer[5]);

    instance->data_.accel_raw_[0] = accel_raw[0];
    instance->data_.accel_raw_[1] = accel_raw[1];
    instance->data_.accel_raw_[2] = accel_raw[2];

    instance->data_.gyro_raw_[0] = gyro_raw[0];
    instance->data_.gyro_raw_[1] = gyro_raw[1];
    instance->data_.gyro_raw_[2] = gyro_raw[2];

    float data[3];

    // 当前换算系数对应 init() 里固定写入的量程配置，accel_单位为m/s^2，gyro_单位为rad/s
    data[0] = ((float)accel_raw[0] / DEVICE_BMI088_ACCEL_SENSITIVITY_6G) * STANDARD_GRAVITY_M_S2;
    data[1] = ((float)accel_raw[1] / DEVICE_BMI088_ACCEL_SENSITIVITY_6G) * STANDARD_GRAVITY_M_S2;
    data[2] = ((float)accel_raw[2] / DEVICE_BMI088_ACCEL_SENSITIVITY_6G) * STANDARD_GRAVITY_M_S2;
    deviceBMI088MapDataByInstallTransform(instance, data, instance->data_.accel_ms2_);

    data[0] = ((float)gyro_raw[0] / DEVICE_BMI088_GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data[1] = ((float)gyro_raw[1] / DEVICE_BMI088_GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data[2] = ((float)gyro_raw[2] / DEVICE_BMI088_GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    deviceBMI088MapDataByInstallTransform(instance, data, instance->data_.gyro_rads_);

    return true;
}

static bool resetDMAXfer(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    memset(instance->dma_xfer_tx_buffer_, 0, sizeof(instance->dma_xfer_tx_buffer_));
    memset(instance->dma_xfer_rx_buffer_, 0, sizeof(instance->dma_xfer_rx_buffer_));
    instance->dma_xfer_state_ = DEVICE_BMI088_DMA_XFER_IDLE;

    return true;
}

// 在ISR中
static void deviceBMI088DMAXferCallback(void *owner, uint8_t *tx_buffer_ptr, uint8_t *rx_buffer_ptr, uint16_t data_size, bool *xfer_continue)
{
    if (owner == NULL || tx_buffer_ptr == NULL || rx_buffer_ptr == NULL || data_size == 0U || xfer_continue == NULL) {
        return;
    }

    deviceBMI088Instance_t *instance = (deviceBMI088Instance_t *)owner;
    *xfer_continue = false;

    if (instance->is_initialized_ == false || instance->mode_ != DEVICE_BMI088_EXTI_DMA) {
        return;
    }

    switch (instance->dma_xfer_state_) {
        case DEVICE_BMI088_DMA_XFER_ACCESS_ACCEL:
            deviceBMI088DeselectAllTarget(instance);
            // 拷贝数据
            memcpy(instance->accel_raw_uint8_buffer, rx_buffer_ptr + 2, ACC_XYZ_LEN);

            deviceBMI088SelectTarget(instance, DEVICE_BMI088_TARGET_GYRO);
            memset(instance->dma_xfer_tx_buffer_, DUMMY_BYTE, sizeof(instance->dma_xfer_tx_buffer_));
            memset(instance->dma_xfer_rx_buffer_, 0, sizeof(instance->dma_xfer_rx_buffer_));
            instance->dma_xfer_tx_buffer_[0] = DEVICE_BMI088_REG_TO_READ_CMD(GYRO_RATE_X_LSB_ADDR);
            // 开启DMA传输
            // 先切换状态，防止立刻进入SPI TXRX CPLT回调，导致状态切换错误
            instance->dma_xfer_state_ = DEVICE_BMI088_DMA_XFER_ACCESS_GYRO;
            bspSPIStatus_e spi_status;
            spi_status = bspSPITransmitReceive(instance->spi_instance_, 
                                                instance->dma_xfer_tx_buffer_, 
                                                instance->dma_xfer_rx_buffer_, 
                                                GYRO_XYZ_LEN + 1U);
            if (spi_status != BSP_SPI_OK) {
                deviceBMI088DeselectAllTarget(instance);
                resetDMAXfer(instance);
                if (instance->dma_xfer_error_notify_callback_from_isr_ != NULL) {
                    instance->dma_xfer_error_notify_callback_from_isr_();
                }
                return;
            }

            *xfer_continue = true;
            break;

        case DEVICE_BMI088_DMA_XFER_ACCESS_GYRO:
            deviceBMI088DeselectAllTarget(instance);
            // 拷贝数据
            memcpy(instance->gyro_raw_uint8_buffer, rx_buffer_ptr + 1, GYRO_XYZ_LEN);
            instance->dma_xfer_state_ = DEVICE_BMI088_DMA_XFER_RAW_DATA_READY;

            if (instance->dma_xfer_cplt_notify_callback_from_isr_ != NULL) {
                instance->dma_xfer_cplt_notify_callback_from_isr_();
            }
            break;
        
        case DEVICE_BMI088_DMA_XFER_ERROR:
            resetDMAXfer(instance);

        case DEVICE_BMI088_DMA_XFER_IDLE:
        case DEVICE_BMI088_DMA_XFER_RAW_DATA_READY:
            return;
    }
}

static void deviceBMI088DMAXferErrorCallback(void *owner)
{
    if (owner == NULL) {
        return;
    }

    deviceBMI088Instance_t *instance = (deviceBMI088Instance_t *)owner;

    deviceBMI088DeselectAllTarget(instance);

    instance->dma_xfer_state_ = DEVICE_BMI088_DMA_XFER_ERROR;

    if (instance->dma_xfer_error_notify_callback_from_isr_ != NULL) {
        instance->dma_xfer_error_notify_callback_from_isr_();
    }
}

// 这里是初始化实例，也许改为register更好？
deviceBMI088Instance_t *deviceBMI088InstanceInit(const deviceBMI088Config_t *config)
{
    if (config == NULL ||
        config->spi_instance_ == NULL ||
        config->accel_cs_ == NULL ||
        config->gyro_cs_ == NULL ||
        config->delay_us_callback_ == NULL) {
        return NULL;
    }

    if (bmi088_memory_index_ >= DEVICE_BMI088_MAX_INSTANCE_NUM) {
        return NULL;
    }

    if (isModeValid(config->mode_) == false) {
        return NULL;
    }

    deviceBMI088Instance_t *instance = &bmi088_instance_memory_[bmi088_memory_index_];
    memset(instance, 0, sizeof(deviceBMI088Instance_t));
    instance->spi_instance_ = config->spi_instance_;
    instance->accel_cs_ = config->accel_cs_;
    instance->gyro_cs_ = config->gyro_cs_;
    instance->mode_ = config->mode_;
    // 默认都是转到FLU下，根据bmi088的安装情况，转到FLU下
    deviceBMI088InstallTransform_t install_transform;
    install_transform.axis_map[0] = DEVICE_BMI088_AXIS_X;
    install_transform.axis_map[1] = DEVICE_BMI088_AXIS_Y;
    install_transform.axis_map[2] = DEVICE_BMI088_AXIS_Z;
    install_transform.axis_sign[0] = 1;
    install_transform.axis_sign[1] = 1;
    install_transform.axis_sign[2] = 1;
    instance->install_transform_ = install_transform;
    instance->delay_us_callback_ = config->delay_us_callback_;
    instance->name_ = config->name_;
    instance->is_initialized_ = false;

    instance->selected_target_ = DEVICE_BMI088_TARGET_NONE;

    instance->dma_xfer_state_ = DEVICE_BMI088_DMA_XFER_IDLE;
    if (instance->mode_ == DEVICE_BMI088_EXTI_DMA) {
        // 也可以为NULL，即不通知
        instance->dma_xfer_cplt_notify_callback_from_isr_ = config->dma_xfer_cplt_notify_callback_from_isr_;
        instance->dma_xfer_error_notify_callback_from_isr_ = config->dma_xfer_error_notify_callback_from_isr_;
        bspSPITxRxCpltCallbackRegister(instance->spi_instance_, instance, deviceBMI088DMAXferCallback);
        bspSPIErrorCallbackRegister(instance->spi_instance_, instance, deviceBMI088DMAXferErrorCallback);
    }

    bmi088_memory_index_ ++;

    return instance;
}

// 设备初始化
deviceBMI088Status_e deviceBMI088Init(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return DEVICE_BMI088_ERROR;
    }

    bool state = true;

    instance->is_initialized_ = false;

    // BMI088初始化使用阻塞读取！
    bspSPIStatus_e spi_status;
    spi_status = bspSPIChangeWorkMode(instance->spi_instance_, BSP_SPI_WORK_MODE_BLOCKING);
    if (spi_status != BSP_SPI_OK) {
        return DEVICE_BMI088_ERROR;
    }

    // 释放所有CS
    deviceBMI088DeselectAllTarget(instance);
    // 延迟一小段时间，等待总线空闲
    deviceBMI088DelayUs(instance, DEVICE_BMI088_BUS_IDLE_DELAY_US);

    // 等待上电完成后再开始访问寄存器
    deviceBMI088DelayUs(instance, DEVICE_BMI088_POWER_ON_DELAY_US);

    // 使用一次dummy read使得accel切换到spi
    state &= deviceBMI088AccelSwitchToSPI(instance);
    if (state == false) {
        return DEVICE_BMI088_ERROR;
    }

    // 正式检查accel和gyro的chip id
    state &= deviceBMI088GetChipID(instance);
    if (state == false) {
        return DEVICE_BMI088_ERROR;
    }

    state &= deviceBMI088ConfigAccel(instance);
    if (state == false) {
        return DEVICE_BMI088_ERROR;
    }

    state &= deviceBMI088ConfigGyro(instance);
    if (state == false) {
        return DEVICE_BMI088_ERROR;
    }

    // 切换到对应的SPI模式
    deviceBMI088DelayUs(instance, DEVICE_BMI088_BUS_IDLE_DELAY_US);
    if (changeSPIWorkModeByBMI088Config(instance) == false) {
        return DEVICE_BMI088_ERROR;
    }

    instance->is_initialized_ = true;

    return DEVICE_BMI088_OK;
}

// 同步读取，只允许在DEVICE_BMI088_BLOCKING 和DEVICE_BMI088_EXTI模式下使用！
deviceBMI088Status_e deviceBMI088UpdateData(deviceBMI088Instance_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return DEVICE_BMI088_ERROR;
    }

    if (instance->mode_ == DEVICE_BMI088_EXTI_DMA) {
        return DEVICE_BMI088_ERROR;
    }

    bool state = true;

    state &= deviceBMI088ReadRegisters(instance,
                                       DEVICE_BMI088_TARGET_ACCEL,
                                       ACC_X_LSB_ADDR,
                                       instance->accel_raw_uint8_buffer,
                                       ACC_XYZ_LEN);
    state &= deviceBMI088ReadRegisters(instance,
                                       DEVICE_BMI088_TARGET_GYRO,
                                       GYRO_RATE_X_LSB_ADDR,
                                       instance->gyro_raw_uint8_buffer,
                                       GYRO_XYZ_LEN);
    if (state == false) {
        return DEVICE_BMI088_ERROR;
    }

    if (deviceBMI088ParseRawData(instance) == false) {
        return DEVICE_BMI088_ERROR;
    }

    return DEVICE_BMI088_OK;
}

// 使用DMA传输更新数据,开启一次DMA传输
deviceBMI088Status_e deviceBMI088UpdateDataDMAStart(deviceBMI088Instance_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return DEVICE_BMI088_ERROR;
    }

    if (instance->mode_ != DEVICE_BMI088_EXTI_DMA) {
        return DEVICE_BMI088_ERROR;
    }

    switch (instance->dma_xfer_state_) {
        case DEVICE_BMI088_DMA_XFER_IDLE:
            deviceBMI088DeselectAllTarget(instance);
            deviceBMI088SelectTarget(instance, DEVICE_BMI088_TARGET_ACCEL);
            memset(instance->dma_xfer_tx_buffer_, DUMMY_BYTE, sizeof(instance->dma_xfer_tx_buffer_));
            memset(instance->dma_xfer_rx_buffer_, 0, sizeof(instance->dma_xfer_rx_buffer_));
            instance->dma_xfer_tx_buffer_[0] = DEVICE_BMI088_REG_TO_READ_CMD(ACC_X_LSB_ADDR);
            // 开启DMA传输
            // 先切换状态，防止立刻进入SPI TXRX CPLT回调，导致状态切换错误
            instance->dma_xfer_state_ = DEVICE_BMI088_DMA_XFER_ACCESS_ACCEL;
            bspSPIStatus_e spi_status;
            spi_status = bspSPITransmitReceive(instance->spi_instance_, 
                                                instance->dma_xfer_tx_buffer_, 
                                                instance->dma_xfer_rx_buffer_, 
                                                ACC_XYZ_LEN + 2U); // ACC_XYZ_LEN + 2U是因为读取ACCEL时，在寄存器地址后，还会多一个无意义字节
            if (spi_status != BSP_SPI_OK) {
                deviceBMI088DeselectAllTarget(instance);
                resetDMAXfer(instance);
                return DEVICE_BMI088_ERROR;
            }
            break;

        case DEVICE_BMI088_DMA_XFER_ERROR:
            resetDMAXfer(instance);

        case DEVICE_BMI088_DMA_XFER_ACCESS_ACCEL:
        case DEVICE_BMI088_DMA_XFER_ACCESS_GYRO:
        case DEVICE_BMI088_DMA_XFER_RAW_DATA_READY:
            return DEVICE_BMI088_ERROR;
    }

    return DEVICE_BMI088_OK;
}

// 当DMA传输一次完整数据完成时，调用该函数以更新IMU数据
deviceBMI088Status_e deviceBMI088UpdateDataDMAProcess(deviceBMI088Instance_t *instance, bool *new_data_ready)
{
    if (instance == NULL || new_data_ready == NULL || instance->is_initialized_ == false) {
        return DEVICE_BMI088_ERROR;
    }

    *new_data_ready = false;

    if (instance->mode_ != DEVICE_BMI088_EXTI_DMA) {
        return DEVICE_BMI088_ERROR;
    }

    switch (instance->dma_xfer_state_) {
        case DEVICE_BMI088_DMA_XFER_RAW_DATA_READY:
            if (deviceBMI088ParseRawData(instance) == false) {
                return DEVICE_BMI088_ERROR;
            }
            *new_data_ready = true;
            instance->dma_xfer_state_ = DEVICE_BMI088_DMA_XFER_IDLE;
            break;
        
        case DEVICE_BMI088_DMA_XFER_ERROR:
            resetDMAXfer(instance);

        case DEVICE_BMI088_DMA_XFER_IDLE:
        case DEVICE_BMI088_DMA_XFER_ACCESS_ACCEL:
        case DEVICE_BMI088_DMA_XFER_ACCESS_GYRO:
            return DEVICE_BMI088_BUSY;
    }

    return DEVICE_BMI088_OK;
}

deviceBMI088Status_e deviceBMI088GetData(const deviceBMI088Instance_t *instance, deviceBMI088Data_t *data_out)
{
    if (instance == NULL || instance->is_initialized_ == false || data_out == NULL) {
        return DEVICE_BMI088_ERROR;
    }

    *data_out = instance->data_;

    return DEVICE_BMI088_OK;
}

deviceBMI088Status_e deviceBMI088GetDataByOutputFrame(const deviceBMI088Instance_t *instance, deviceBMI088Data_t *data_out, deviceBMI088OutputFrame_e frame)
{
    if (instance == NULL || instance->is_initialized_ == false || data_out == NULL) {
        return DEVICE_BMI088_ERROR;
    }

    // 默认instance中的data数据(data_raw不是！)是在FLU下的
    switch (frame) {
        case DEVICE_BMI088_OUTPUT_FRAME_FLU:
            *data_out = instance->data_;
            break;
        case DEVICE_BMI088_OUTPUT_FRAME_FRD:
            data_out->accel_ms2_[0] = instance->data_.accel_ms2_[0];
            data_out->accel_ms2_[1] = -instance->data_.accel_ms2_[1];
            data_out->accel_ms2_[2] = -instance->data_.accel_ms2_[2];
            data_out->gyro_rads_[0] = instance->data_.gyro_rads_[0];
            data_out->gyro_rads_[1] = -instance->data_.gyro_rads_[1];
            data_out->gyro_rads_[2] = -instance->data_.gyro_rads_[2];

            memcpy(data_out->accel_raw_, instance->data_.accel_raw_, sizeof(int16_t) * 3U);
            memcpy(data_out->gyro_raw_, instance->data_.gyro_raw_, sizeof(int16_t) * 3U);
            break;
        default:
            return DEVICE_BMI088_ERROR;
    }

    return DEVICE_BMI088_OK;
}

// 配置bmi088的gyro数据就绪中断
deviceBMI088Status_e deviceBMI088ConfigGyroDataReadyIT(deviceBMI088Instance_t *instance)
{
    if (instance == NULL) {
        return DEVICE_BMI088_ERROR;
    }

    if (instance->mode_ == DEVICE_BMI088_BLOCKING) {
        return DEVICE_BMI088_ERROR;
    }

    // BMI088数据就绪中断配置也使用阻塞读取！
    bspSPIStatus_e spi_status;
    spi_status = bspSPIChangeWorkMode(instance->spi_instance_, BSP_SPI_WORK_MODE_BLOCKING);
    if (spi_status != BSP_SPI_OK) {
        return DEVICE_BMI088_ERROR;
    }

    bool state = true;

    // 官方流程是先映射再配置电气化，最后使能中断
    // 不过经过测试，先配置电气化再映射也可以
    // 将gyro的数据准备完成中断映射到INT3
    state &= deviceBMI088WriteSingleRegister(instance, 
                                            DEVICE_BMI088_TARGET_GYRO, 
                                            GYRO_INT3_INT4_IO_MAP_ADDR, 
                                            GYRO_INT3_INT4_IO_MAP_IT_INT3_VAL);
    if (state == false) {
        return DEVICE_BMI088_ERROR;
    }
    deviceBMI088DelayUs(instance, 2U);

    // 将gyro的数据准备完成中断配置为INT3推挽输出、低电平有效
    state &= deviceBMI088WriteSingleRegister(instance, 
                                            DEVICE_BMI088_TARGET_GYRO, 
                                            GYRO_INT3_INT4_IO_CONF_ADDR, 
                                            GYRO_INT3_INT4_IO_CONF_INT3_PP_ACTIVE_LOW_VAL);
    if (state == false) {
        return DEVICE_BMI088_ERROR;
    }
    deviceBMI088DelayUs(instance, 2U);

    // 使能gyro的数据准备完成中断
    state &= deviceBMI088WriteSingleRegister(instance, 
                                            DEVICE_BMI088_TARGET_GYRO, 
                                            GYRO_INT_CTRL_ADDR, 
                                            GYRO_INT_CTRL_IT_ENABLE_VAL);
    if (state == false) {
        return DEVICE_BMI088_ERROR;
    }

    // 切换到对应的SPI模式
    deviceBMI088DelayUs(instance, DEVICE_BMI088_BUS_IDLE_DELAY_US);
    if (changeSPIWorkModeByBMI088Config(instance) == false) {
        return DEVICE_BMI088_ERROR;
    }

    return DEVICE_BMI088_OK;
}

deviceBMI088Status_e deviceBMI088GetMode(deviceBMI088Instance_t *instance, deviceBMI088Mode_e *mode_out)
{
    if (instance == NULL) {
        return DEVICE_BMI088_ERROR;
    }

    *mode_out = instance->mode_;

    return DEVICE_BMI088_OK;
}

#include "i2c.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "bsp_i2c.h"
#include "bsp_i2c_private.h"

#define BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(x) (((x) & 0x7FU) << 1) // 地址左移1位，bit 0为读写位

typedef struct i2c_transfer_context
{
    bspI2CTransferType_e xfer_type_; // 本次传输事务类型
    uint16_t slave_address_7bit_; // 从机地址，只支持7位
    uint16_t memory_address_; // 操作的从机寄存器地址,这里uin16_t是因为，从机的寄存器地址可能是8位或者16位的
    uint16_t memory_address_size_; // 从机寄存器地址的位数，8或16bit，因为同一个地址比如0x10，8bit下为0x10，16bit下为0x0010，含义不同,注意HAL库下I2C从机寄存器地址发送为MSB

    uint8_t *tx_buffer_;
    uint8_t *rx_buffer_;
    uint16_t tx_data_size_; // 本次传输事务数据大小
    uint16_t rx_data_size_;

    uint32_t xfer_option_seq_; // seq传输的设置
} bspI2CTransferContext_t; // 本次传输事务的上下文

typedef struct i2c_instance 
{
    I2C_HandleTypeDef *i2c_handle_;
    bspI2CWorkMode_e i2c_work_mode_;

    bspI2CTransferContext_t context_this_transfer_; // 保存本次传输事务所有的上下文信息，这里和spi的想法一致

    const char *name_;

    bspI2CTransferCpltCallback_f xfer_cplt_callback_;
    bspI2CErrorCallback_f error_callback_;

    void *owner_;
} bspI2CInstance_t;

static uint8_t i2c_memory_index_ = 0;
static bspI2CInstance_t i2c_instance_memory_[BSP_I2C_MAX_INSTANCE_NUM] = {0};

// 将bspI2CSeqOption_e枚举值转为hal库值,防止依赖泄露
static bool bspI2CGetHALSeqTransferOptionFromBSP(bspI2CSeqTransferOption_e bsp_xfer_option, uint32_t *out)
{                                                                                                                                                                                                
    switch (bsp_xfer_option) {                                                                                                                                                                            
        case BSP_I2C_SEQ_FIRST:                                                                                                                                                                  
            *out = I2C_FIRST_FRAME;
            break;                                                                                                                                                              
        case BSP_I2C_SEQ_FIRST_AND_NEXT:                                                                                                                                                         
            *out = I2C_FIRST_AND_NEXT_FRAME;
            break;                                                                                                                                                       
        case BSP_I2C_SEQ_NEXT:                                                                                                                                                                   
            *out = I2C_NEXT_FRAME;
            break;                                                                                                                                                                 
        case BSP_I2C_SEQ_FIRST_AND_LAST:                                                                                                                                                         
            *out = I2C_FIRST_AND_LAST_FRAME;
            break;                                                                                                                                                       
        case BSP_I2C_SEQ_LAST:                                                                                                                                                                   
            *out = I2C_LAST_FRAME;
            break;                                                                                                                                                                 
        case BSP_I2C_SEQ_LAST_NO_STOP:                                                                                                                                                           
            *out = I2C_LAST_FRAME_NO_STOP;
            break;                                                                                                                                                         
        default:                                                                                                                                                                                 
            return false;                                                                                                                                                     
    }
    
    return true;
}

static bool bspI2CGetHALMemoryAddressSizeFromBSP(bspI2CMemoryAddressSize_e bsp_memory_address_size, uint16_t *out)
{
     switch (bsp_memory_address_size) {                                                                                                                                                                            
        case BSP_I2C_MEM_ADD_SIZE_8BIT:                                                                                                                                                                  
            *out = I2C_MEMADD_SIZE_8BIT;
            break;                                                                                                                                                                
        case BSP_I2C_MEM_ADD_SIZE_16BIT:                                                                                                                                                         
            *out = I2C_MEMADD_SIZE_16BIT;
            break;                                                                                                                                                                                                                                                                                                              
        default:                                                                                                                                                                                 
            return false;                                                                                                                                                     
    }

    return true;
}

static bspI2CStatus_e bspI2CGetStatusFromHAL(HAL_StatusTypeDef status_hal)
{
    switch (status_hal) {
        case HAL_OK:
            return BSP_I2C_OK;
        case HAL_ERROR:
            return BSP_I2C_ERROR;
        case HAL_BUSY:
            return BSP_I2C_BUSY;
        case HAL_TIMEOUT:
            return BSP_I2C_TIMEOUT;
        default:
            break;
    }
    return BSP_I2C_ERROR;
}

static bspI2CInstance_t *bspI2CFindInstanceByHandle(I2C_HandleTypeDef *i2c_handle)
{
    for (size_t i = 0; i < i2c_memory_index_; i ++) {
        if (i2c_instance_memory_[i].i2c_handle_ == i2c_handle) {
            return &i2c_instance_memory_[i];
        }
    }
    return NULL;
}

bspI2CInstance_t *bspI2CInit(const bspI2CConfig_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    if (i2c_memory_index_ >= BSP_I2C_MAX_INSTANCE_NUM) {
        return NULL;
    }

    bspI2CInstance_t *instance = &i2c_instance_memory_[i2c_memory_index_];
    memset(instance, 0, sizeof(bspI2CInstance_t));
    instance->i2c_handle_ = config->i2c_handle_;
    instance->i2c_work_mode_ = config->i2c_work_mode_;
    instance->name_ = config->name_;

    i2c_memory_index_ ++;

    return instance;
}

void bspI2CTransferCpltCallbackRegister(bspI2CInstance_t *instance, void *owner_ptr, bspI2CTransferCpltCallback_f callback)
{
    if (instance == NULL) {
        return;
    }

    instance->owner_ = owner_ptr;
    instance->xfer_cplt_callback_ = callback;
}

void bspI2CErrorCallbackRegister(bspI2CInstance_t *instance, void *owner_ptr, bspI2CErrorCallback_f callback)
{
    if (instance == NULL) {
        return;
    }

    instance->owner_ = owner_ptr;
    instance->error_callback_ = callback;
}

// 这里语义上其实更接近该i2c外设是否处于ready下，是否可以发起一次新事务
bool bspI2CIsBusy(bspI2CInstance_t *instance)
{
    if (instance == NULL || instance->i2c_handle_ == NULL) {
        return true;
    }
    return instance->i2c_handle_->State != HAL_I2C_STATE_READY;
}

// 请注意，传入时从机地址不需要移位
bspI2CStatus_e bspI2CMasterTransmit(bspI2CInstance_t *instance, uint16_t slave_address, uint8_t *tx_buffer, uint16_t data_size)
{
    if (instance == NULL || instance->i2c_handle_ == NULL || tx_buffer == NULL || data_size == 0 || slave_address > 0x7FU) {
        return BSP_I2C_ERROR;
    }

    if (instance->i2c_handle_->State != HAL_I2C_STATE_READY) {
        return BSP_I2C_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    switch (instance->i2c_work_mode_) {
        case BSP_I2C_WORK_MODE_BLOCKING:
            status_hal = HAL_I2C_Master_Transmit(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), tx_buffer, data_size, BSP_I2C_BLOCKING_TIMEOUT);
            return bspI2CGetStatusFromHAL(status_hal);
        case BSP_I2C_WORK_MODE_IT:
            status_hal = HAL_I2C_Master_Transmit_IT(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), tx_buffer, data_size);
            break;
        case BSP_I2C_WORK_MODE_DMA:
            status_hal = HAL_I2C_Master_Transmit_DMA(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), tx_buffer, data_size);
            break;
        default:
            return BSP_I2C_ERROR;            
    }

    if (status_hal == HAL_OK) {
        // 异步传输启动成功，先清空旧的事务上下文信息，不可以在一个事务结束后清除
        memset(&instance->context_this_transfer_, 0, sizeof(bspI2CTransferContext_t));

        instance->context_this_transfer_.xfer_type_ = BSP_I2C_XFER_TYPE_MASTER_TX;
        instance->context_this_transfer_.slave_address_7bit_ = slave_address;
        instance->context_this_transfer_.tx_buffer_ = tx_buffer;
        instance->context_this_transfer_.tx_data_size_ = data_size;
    }

    return bspI2CGetStatusFromHAL(status_hal);
}

bspI2CStatus_e bspI2CMasterReceive(bspI2CInstance_t *instance, uint16_t slave_address, uint8_t *rx_buffer, uint16_t data_size)
{
    if (instance == NULL || instance->i2c_handle_ == NULL || rx_buffer == NULL || data_size == 0 || slave_address > 0x7FU) {
        return BSP_I2C_ERROR;
    }

    if (instance->i2c_handle_->State != HAL_I2C_STATE_READY) {
        return BSP_I2C_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    switch (instance->i2c_work_mode_) {
        case BSP_I2C_WORK_MODE_BLOCKING:
            status_hal = HAL_I2C_Master_Receive(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), rx_buffer, data_size, BSP_I2C_BLOCKING_TIMEOUT);
            return bspI2CGetStatusFromHAL(status_hal);
        case BSP_I2C_WORK_MODE_IT:
            status_hal = HAL_I2C_Master_Receive_IT(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), rx_buffer, data_size);
            break;
        case BSP_I2C_WORK_MODE_DMA:
            status_hal = HAL_I2C_Master_Receive_DMA(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), rx_buffer, data_size);
            break;
        default:
            return BSP_I2C_ERROR;            
    }

    if (status_hal == HAL_OK) {
        memset(&instance->context_this_transfer_, 0, sizeof(bspI2CTransferContext_t));

        instance->context_this_transfer_.xfer_type_ = BSP_I2C_XFER_TYPE_MASTER_RX;
        instance->context_this_transfer_.slave_address_7bit_ = slave_address;
        instance->context_this_transfer_.rx_buffer_ = rx_buffer;
        instance->context_this_transfer_.rx_data_size_ = data_size;
    }

    return bspI2CGetStatusFromHAL(status_hal);
}

// 按从机寄存器地址写入
bspI2CStatus_e bspI2CMemoryWrite(bspI2CInstance_t *instance, uint16_t slave_address, uint16_t memory_address, bspI2CMemoryAddressSize_e memory_address_size, uint8_t *tx_buffer, uint16_t data_size)
{
    if (instance == NULL || instance->i2c_handle_ == NULL || tx_buffer == NULL || data_size == 0 || slave_address > 0x7FU) {
        return BSP_I2C_ERROR;
    }

    if (instance->i2c_handle_->State != HAL_I2C_STATE_READY) {
        return BSP_I2C_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    uint16_t memory_address_size_hal;
    if (bspI2CGetHALMemoryAddressSizeFromBSP(memory_address_size, &memory_address_size_hal) == false) {
        return BSP_I2C_ERROR;
    }

    switch (instance->i2c_work_mode_) {
        case BSP_I2C_WORK_MODE_BLOCKING:
            status_hal = HAL_I2C_Mem_Write(instance->i2c_handle_, 
                                            BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), 
                                            memory_address, 
                                            memory_address_size_hal, 
                                            tx_buffer, 
                                            data_size, 
                                            BSP_I2C_BLOCKING_TIMEOUT);
            return bspI2CGetStatusFromHAL(status_hal);
        case BSP_I2C_WORK_MODE_IT:
            status_hal = HAL_I2C_Mem_Write_IT(instance->i2c_handle_, 
                                            BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), 
                                            memory_address, 
                                            memory_address_size_hal, 
                                            tx_buffer, 
                                            data_size);
            break;
        case BSP_I2C_WORK_MODE_DMA:
            status_hal = HAL_I2C_Mem_Write_DMA(instance->i2c_handle_, 
                                                BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), 
                                                memory_address, 
                                                memory_address_size_hal, 
                                                tx_buffer, 
                                                data_size);
            break;
        default:
            return BSP_I2C_ERROR;            
    }

    if (status_hal == HAL_OK) {
        memset(&instance->context_this_transfer_, 0, sizeof(bspI2CTransferContext_t));

        instance->context_this_transfer_.xfer_type_ = BSP_I2C_XFER_TYPE_MEMORY_WRITE;
        instance->context_this_transfer_.slave_address_7bit_ = slave_address;
        instance->context_this_transfer_.memory_address_ = memory_address;
        instance->context_this_transfer_.memory_address_size_ = memory_address_size;
        instance->context_this_transfer_.tx_buffer_ = tx_buffer;
        instance->context_this_transfer_.tx_data_size_ = data_size;
    }

    return bspI2CGetStatusFromHAL(status_hal);
}

bspI2CStatus_e bspI2CMemoryRead(bspI2CInstance_t *instance, uint16_t slave_address, uint16_t memory_address, bspI2CMemoryAddressSize_e memory_address_size, uint8_t *rx_buffer, uint16_t data_size)
{
    if (instance == NULL || instance->i2c_handle_ == NULL || rx_buffer == NULL || data_size == 0 || slave_address > 0x7FU) {
        return BSP_I2C_ERROR;
    }

    if (instance->i2c_handle_->State != HAL_I2C_STATE_READY) {
        return BSP_I2C_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    uint16_t memory_address_size_hal;
    if (bspI2CGetHALMemoryAddressSizeFromBSP(memory_address_size, &memory_address_size_hal) == false) {
        return BSP_I2C_ERROR;
    }

    switch (instance->i2c_work_mode_) {
        case BSP_I2C_WORK_MODE_BLOCKING:
            status_hal = HAL_I2C_Mem_Read(instance->i2c_handle_, 
                                        BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), 
                                        memory_address, 
                                        memory_address_size_hal, 
                                        rx_buffer, 
                                        data_size, 
                                        BSP_I2C_BLOCKING_TIMEOUT);
            return bspI2CGetStatusFromHAL(status_hal);
        case BSP_I2C_WORK_MODE_IT:
            status_hal = HAL_I2C_Mem_Read_IT(instance->i2c_handle_, 
                                            BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), 
                                            memory_address, 
                                            memory_address_size_hal, 
                                            rx_buffer, 
                                            data_size);
            break;
        case BSP_I2C_WORK_MODE_DMA:
            status_hal = HAL_I2C_Mem_Read_DMA(instance->i2c_handle_, 
                                            BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), 
                                            memory_address, 
                                            memory_address_size_hal, 
                                            rx_buffer, 
                                            data_size);
            break;
        default:
            return BSP_I2C_ERROR;            
    }

    if (status_hal == HAL_OK) {
        memset(&instance->context_this_transfer_, 0, sizeof(bspI2CTransferContext_t));

        instance->context_this_transfer_.xfer_type_ = BSP_I2C_XFER_TYPE_MEMORY_READ;
        instance->context_this_transfer_.slave_address_7bit_ = slave_address;
        instance->context_this_transfer_.memory_address_ = memory_address;
        instance->context_this_transfer_.memory_address_size_ = memory_address_size;
        instance->context_this_transfer_.rx_buffer_ = rx_buffer;
        instance->context_this_transfer_.rx_data_size_ = data_size;
    }

    return bspI2CGetStatusFromHAL(status_hal);
}

// 序列传输
bspI2CStatus_e bspI2CMasterSeqTransmit(bspI2CInstance_t *instance, uint16_t slave_address, uint8_t *tx_buffer, uint16_t data_size, bspI2CSeqTransferOption_e seq_option)
{
    if (instance == NULL || instance->i2c_handle_ == NULL || tx_buffer == NULL || data_size == 0 || seq_option == BSP_I2C_SEQ_NONE || slave_address > 0x7FU) {
        return BSP_I2C_ERROR;
    }

    if (instance->i2c_handle_->State != HAL_I2C_STATE_READY) {
        return BSP_I2C_BUSY;
    }

    uint32_t seq_option_hal;
    if (bspI2CGetHALSeqTransferOptionFromBSP(seq_option, &seq_option_hal) == false) {
        return BSP_I2C_ERROR;
    }

    HAL_StatusTypeDef status_hal;

    switch (instance->i2c_work_mode_) {
        case BSP_I2C_WORK_MODE_BLOCKING:
            // 请注意，seq传输不适用于blocking模式
            return BSP_I2C_ERROR;
        case BSP_I2C_WORK_MODE_IT:
            status_hal = HAL_I2C_Master_Seq_Transmit_IT(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), tx_buffer, data_size, seq_option_hal);
            break;
        case BSP_I2C_WORK_MODE_DMA:
            status_hal = HAL_I2C_Master_Seq_Transmit_DMA(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), tx_buffer, data_size, seq_option_hal);
            break;
        default:
            return BSP_I2C_ERROR;            
    }

    if (status_hal == HAL_OK) {
        memset(&instance->context_this_transfer_, 0, sizeof(bspI2CTransferContext_t));

        instance->context_this_transfer_.xfer_type_ = BSP_I2C_XFER_TYPE_MASTER_SEQ_TX;
        instance->context_this_transfer_.slave_address_7bit_ = slave_address;
        instance->context_this_transfer_.tx_buffer_ = tx_buffer;
        instance->context_this_transfer_.tx_data_size_ = data_size;
        instance->context_this_transfer_.xfer_option_seq_ = seq_option;
    }

    return bspI2CGetStatusFromHAL(status_hal);
}

bspI2CStatus_e bspI2CMasterSeqReceive(bspI2CInstance_t *instance, uint16_t slave_address, uint8_t *rx_buffer, uint16_t data_size, bspI2CSeqTransferOption_e seq_option)
{
    if (instance == NULL || instance->i2c_handle_ == NULL || rx_buffer == NULL || data_size == 0 || seq_option == BSP_I2C_SEQ_NONE || slave_address > 0x7FU) {
        return BSP_I2C_ERROR;
    }

    if (instance->i2c_handle_->State != HAL_I2C_STATE_READY) {
        return BSP_I2C_BUSY;
    }

    uint32_t seq_option_hal;
    if (bspI2CGetHALSeqTransferOptionFromBSP(seq_option, &seq_option_hal) == false) {
        return BSP_I2C_ERROR;
    }

    HAL_StatusTypeDef status_hal;

    switch (instance->i2c_work_mode_) {
        case BSP_I2C_WORK_MODE_BLOCKING:
            // 请注意，seq传输不适用于blocking模式
            return BSP_I2C_ERROR;
        case BSP_I2C_WORK_MODE_IT:
            status_hal = HAL_I2C_Master_Seq_Receive_IT(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), rx_buffer, data_size, seq_option_hal);
            break;
        case BSP_I2C_WORK_MODE_DMA:
            status_hal = HAL_I2C_Master_Seq_Receive_DMA(instance->i2c_handle_, BSP_I2C_SLAVE_ADDRESS_WITH_WRITE_READ_BIT(slave_address), rx_buffer, data_size, seq_option_hal);
            break;
        default:
            return BSP_I2C_ERROR;            
    }

    if (status_hal == HAL_OK) {
        memset(&instance->context_this_transfer_, 0, sizeof(bspI2CTransferContext_t));

        instance->context_this_transfer_.xfer_type_ = BSP_I2C_XFER_TYPE_MASTER_SEQ_RX;
        instance->context_this_transfer_.slave_address_7bit_ = slave_address;
        instance->context_this_transfer_.rx_buffer_ = rx_buffer;
        instance->context_this_transfer_.rx_data_size_ = data_size;
        instance->context_this_transfer_.xfer_option_seq_ = seq_option;
    }

    return bspI2CGetStatusFromHAL(status_hal);
}

static void bspI2CTransferCpltCallback(I2C_HandleTypeDef *hi2c)
{
    bspI2CInstance_t *instance = bspI2CFindInstanceByHandle(hi2c);
    if (instance == NULL) {
        return;
    }

    if (instance->xfer_cplt_callback_ != NULL) {
        switch (instance->context_this_transfer_.xfer_type_) {
            case BSP_I2C_XFER_TYPE_MASTER_TX:
            case BSP_I2C_XFER_TYPE_MEMORY_WRITE:
            case BSP_I2C_XFER_TYPE_MASTER_SEQ_TX:
                instance->xfer_cplt_callback_(instance->owner_, 
                                            instance->context_this_transfer_.tx_buffer_,
                                            instance->context_this_transfer_.tx_data_size_, 
                                            instance->context_this_transfer_.xfer_type_, 
                                            instance->context_this_transfer_.xfer_option_seq_);
                break;
            case BSP_I2C_XFER_TYPE_MASTER_RX:
            case BSP_I2C_XFER_TYPE_MEMORY_READ:
            case BSP_I2C_XFER_TYPE_MASTER_SEQ_RX:
                instance->xfer_cplt_callback_(instance->owner_, 
                                            instance->context_this_transfer_.rx_buffer_,
                                            instance->context_this_transfer_.rx_data_size_, 
                                            instance->context_this_transfer_.xfer_type_, 
                                            instance->context_this_transfer_.xfer_option_seq_);
                break;
            default:
                break;
        }
    }

    // 请注意，不可以在回调结束后清空上下文，因为i2c支持seq传输，回调中可以开启下一段传输事务，会写入上下文信息！！
    // memset(&instance->context_this_transfer_, 0, sizeof(bspI2CTransferContext_t));
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    bspI2CTransferCpltCallback(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    bspI2CTransferCpltCallback(hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    bspI2CTransferCpltCallback(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    bspI2CTransferCpltCallback(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    bspI2CInstance_t *instance = bspI2CFindInstanceByHandle(hi2c);
    if (instance == NULL) {
        return;
    }

    if (instance->error_callback_ != NULL) {
        instance->error_callback_(instance->owner_);
    }

    // 请注意，不可以在回调结束后清空上下文，因为i2c支持seq传输，回调中可以开启下一段传输事务，会写入上下文信息！！
    // memset(&instance->context_this_transfer_, 0, sizeof(bspI2CTransferContext_t));
}
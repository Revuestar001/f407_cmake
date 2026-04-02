#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "bsp_def.h"

typedef enum
{
    BSP_I2C_OK = 0,
    BSP_I2C_ERROR,
    BSP_I2C_BUSY,
    BSP_I2C_TIMEOUT
} bspI2CStatus_e;

typedef enum
{
    BSP_I2C_WORK_MODE_BLOCKING = 0,
    BSP_I2C_WORK_MODE_IT,
    BSP_I2C_WORK_MODE_DMA
} bspI2CWorkMode_e;

typedef enum
{
    BSP_I2C_MEM_ADD_SIZE_8BIT = 0,
    BSP_I2C_MEM_ADD_SIZE_16BIT,
} bspI2CMemoryAddressSize_e;

typedef enum
{
    BSP_I2C_XFER_TYPE_MASTER_TX = 0, // 主机发送，不带操作的寄存器地址
    BSP_I2C_XFER_TYPE_MASTER_RX, // 主机接收，不带操作的寄存器地址
    BSP_I2C_XFER_TYPE_MEMORY_WRITE, // 内存写，带操作的寄存器地址
    BSP_I2C_XFER_TYPE_MEMORY_READ, // 内存读，带操作的寄存器地址
    BSP_I2C_XFER_TYPE_MASTER_SEQ_TX, // 主机序列发送
    BSP_I2C_XFER_TYPE_MASTER_SEQ_RX, // 主机序列接收
} bspI2CTransferType_e; // 传输事务类型

typedef enum                                                                                                                                                                                     
{   
    BSP_I2C_SEQ_NONE = 0,                                                                                                                                                                                             
    BSP_I2C_SEQ_FIRST,                                                                                                                                                                       
    BSP_I2C_SEQ_FIRST_AND_NEXT,                                                                                                                                                                  
    BSP_I2C_SEQ_NEXT,                                                                                                                                                                            
    BSP_I2C_SEQ_FIRST_AND_LAST,                                                                                                                                                                  
    BSP_I2C_SEQ_LAST,                                                                                                                                                                            
    BSP_I2C_SEQ_LAST_NO_STOP,
    // 请注意OTHER字段很少使用，这里暂不支持                                                                                                                                                                     
} bspI2CSeqTransferOption_e; // 这是为了防止hal中seq 传输的xfer_option值泄漏到上层，只是转换hal的值而已

typedef struct i2c_instance bspI2CInstance_t;

// 传输事务完成回调，注意i2c是半双工，一般不会像spi那样同时用到tx_buffer和rx_buffer,这里transfer_buffer是为了统一传输事务的接口，在发送事务下表示为tx_buffer，接收事务下表示rx_buffer
typedef void (*bspI2CTransferCpltCallback_f)(void *owner, uint8_t *transfer_buffer, uint16_t transfer_size, bspI2CTransferType_e transfer_type, bspI2CSeqTransferOption_e seq_option);
typedef void (*bspI2CErrorCallback_f)(void *owner);

void bspI2CTransferCpltCallbackRegister(bspI2CInstance_t *instance, void *owner_ptr, bspI2CTransferCpltCallback_f callback);
void bspI2CErrorCallbackRegister(bspI2CInstance_t *instance, void *owner_ptr, bspI2CErrorCallback_f callback);
// 这里语义上其实更接近该i2c外设是否处于ready下，是否可以发起一次新事务
bool bspI2CIsBusy(bspI2CInstance_t *instance);
// 请注意，传入时从机地址不需要移位
bspI2CStatus_e bspI2CMasterTransmit(bspI2CInstance_t *instance, uint16_t slave_address, uint8_t *tx_buffer, uint16_t data_size);
bspI2CStatus_e bspI2CMasterReceive(bspI2CInstance_t *instance, uint16_t slave_address, uint8_t *rx_buffer, uint16_t data_size);
// 按从机寄存器地址写入
bspI2CStatus_e bspI2CMemoryWrite(bspI2CInstance_t *instance, uint16_t slave_address, uint16_t memory_address, bspI2CMemoryAddressSize_e memory_address_size, uint8_t *tx_buffer, uint16_t data_size);
bspI2CStatus_e bspI2CMemoryRead(bspI2CInstance_t *instance, uint16_t slave_address, uint16_t memory_address, bspI2CMemoryAddressSize_e memory_address_size, uint8_t *rx_buffer, uint16_t data_size);
// 序列传输
bspI2CStatus_e bspI2CMasterSeqTransmit(bspI2CInstance_t *instance, uint16_t slave_address, uint8_t *tx_buffer, uint16_t data_size, bspI2CSeqTransferOption_e seq_option);
bspI2CStatus_e bspI2CMasterSeqReceive(bspI2CInstance_t *instance, uint16_t slave_address, uint8_t *rx_buffer, uint16_t data_size, bspI2CSeqTransferOption_e seq_option);


#include "spi.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_def.h"
#include "bsp_spi.h"
#include "bsp_spi_private.h"

typedef struct spi_instance
{
    SPI_HandleTypeDef *spi_handle_;
    bspSPIWorkMode_e spi_work_mode_;

    // 一次传输由owner发起，spi实例只储存本次传输所用到的缓冲区信息
    uint8_t *tx_buffer_ptr_this_transfer_;
    uint8_t *rx_buffer_ptr_this_transfer_;
    uint16_t data_size_this_transfer_;

    const char *name_;

    bspSPITxRxCpltCallback_f txrx_cplt_callback_;
    bspSPIErrorCallback_f error_callback_;

    void *owner_;
} bspSPIInstance_t;

static uint8_t spi_memory_index_ = 0;
static bspSPIInstance_t spi_instance_memory_[BSP_SPI_MAX_INSTANCE_NUM] = {NULL};

static bspSPIStatus_e bspSPIGetStatusFromHAL(HAL_StatusTypeDef status_hal)
{
    switch (status_hal) {
        case HAL_OK:
            return BSP_SPI_OK;
        case HAL_ERROR:
            return BSP_SPI_ERROR;
        case HAL_BUSY:
            return BSP_SPI_BUSY;
        case HAL_TIMEOUT:
            return BSP_SPI_TIMEOUT;
        default:
            break;
    }
    return BSP_SPI_ERROR;
}

static bspSPIInstance_t *bspSPIFindInstanceByHandle(SPI_HandleTypeDef *spi_handle)
{
    for (size_t i = 0; i < spi_memory_index_; i ++) {
        if (spi_instance_memory_[i].spi_handle_ == spi_handle) {
            return &spi_instance_memory_[i];
        }
    }
    return NULL;
}

bspSPIInstance_t *bspSPIInit(const bspSPIConfig_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    if (spi_memory_index_ >= BSP_SPI_MAX_INSTANCE_NUM) {
        return NULL;
    }

    bspSPIInstance_t *instance = &spi_instance_memory_[spi_memory_index_];
    memset(instance, 0, sizeof(bspSPIInstance_t));
    instance->spi_handle_ = config->spi_handle_;
    instance->spi_work_mode_ = config->spi_work_mode_;
    instance->name_ = config->name_;

    spi_memory_index_ ++;

    return instance;
}

void bspSPITxRxCpltCallbackRegister(bspSPIInstance_t *instance, void *owner_ptr, bspSPITxRxCpltCallback_f callback)
{
    instance->owner_ = owner_ptr;
    instance->txrx_cplt_callback_ = callback;
}

void bspSPIErrorCallbackRegister(bspSPIInstance_t *instance, void *owner_ptr, bspSPIErrorCallback_f callback)
{
    instance->owner_ = owner_ptr;
    instance->error_callback_ = callback;
}

// IT 和DMA模式下，传输完成会触发回调函数
bspSPIStatus_e bspSPITransmitReceive(bspSPIInstance_t *instance, uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t data_size)
{
    if (instance == NULL || 
        instance->spi_handle_ == NULL || 
        tx_buffer == NULL || 
        rx_buffer == NULL || 
        data_size == 0) {
        return BSP_SPI_ERROR;
    }

    // 不为ready状态，认为该spi外设正忙
    if (instance->spi_handle_->State != HAL_SPI_STATE_READY) {
        return BSP_SPI_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    // CS信号由owner决定，这里不进行CS操作
    switch (instance->spi_work_mode_) {
        case BSP_SPI_WORK_MODE_BLOCKING:
            status_hal = HAL_SPI_TransmitReceive(instance->spi_handle_, tx_buffer, rx_buffer, data_size, BSP_SPI_BLOCKING_TIMEOUT);
            return bspSPIGetStatusFromHAL(status_hal);
        case BSP_SPI_WORK_MODE_IT:
            status_hal = HAL_SPI_TransmitReceive_IT(instance->spi_handle_, tx_buffer, rx_buffer, data_size);
            break;
        case BSP_SPI_WORK_MODE_DMA:
            status_hal = HAL_SPI_TransmitReceive_DMA(instance->spi_handle_, tx_buffer, rx_buffer, data_size);
            break;
        default:
            return BSP_SPI_ERROR;
    }

    if (status_hal == HAL_OK) {
        // 成功开启异步传输后，保存本次传输任务的缓冲区信息
        // IT DMA传输开启后，该函数返回
        // 一次传输完成后，触发回调，spi实例调用owner回调函数
        // 由于是异步传输，因此spi需要保存本次传输的缓冲区信息
        instance->tx_buffer_ptr_this_transfer_ = tx_buffer;
        instance->rx_buffer_ptr_this_transfer_ = rx_buffer;
        instance->data_size_this_transfer_ = data_size;
    }

    return bspSPIGetStatusFromHAL(status_hal);
}

// IT 和DMA模式下，传输完成会触发回调函数
bspSPIStatus_e bspSPITransmit(bspSPIInstance_t *instance, uint8_t *tx_buffer, uint16_t data_size)
{
    if (instance == NULL || 
        instance->spi_handle_ == NULL || 
        tx_buffer == NULL ||  
        data_size == 0) {
        return BSP_SPI_ERROR;
    }

    // 不为ready状态，认为该spi外设正忙
    if (instance->spi_handle_->State != HAL_SPI_STATE_READY) {
        return BSP_SPI_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    // CS信号由owner决定，这里不进行CS操作
    switch (instance->spi_work_mode_) {
        case BSP_SPI_WORK_MODE_BLOCKING:
            status_hal = HAL_SPI_Transmit(instance->spi_handle_, tx_buffer, data_size, BSP_SPI_BLOCKING_TIMEOUT);
            return bspSPIGetStatusFromHAL(status_hal);
        case BSP_SPI_WORK_MODE_IT:
            status_hal = HAL_SPI_Transmit_IT(instance->spi_handle_, tx_buffer, data_size);
            break;
        case BSP_SPI_WORK_MODE_DMA:
            status_hal = HAL_SPI_Transmit_DMA(instance->spi_handle_, tx_buffer, data_size);
            break;
        default:
            return BSP_SPI_ERROR;
    }

    if (status_hal == HAL_OK) {
        // 成功开启异步传输后，保存本次传输任务的缓冲区信息
        // IT DMA传输开启后，该函数返回
        // 一次传输完成后，触发回调，spi实例调用owner回调函数
        // 由于是异步传输，因此spi需要保存本次传输的缓冲区信息
        instance->tx_buffer_ptr_this_transfer_ = tx_buffer;
        instance->rx_buffer_ptr_this_transfer_ = NULL;
        instance->data_size_this_transfer_ = data_size;
    }

    return bspSPIGetStatusFromHAL(status_hal);
}

// IT 和DMA模式下，传输完成会触发回调函数
bspSPIStatus_e bspSPIReceive(bspSPIInstance_t *instance, uint8_t *rx_buffer, uint16_t data_size)
{
    if (instance == NULL || 
        instance->spi_handle_ == NULL || 
        rx_buffer == NULL || 
        data_size == 0) {
        return BSP_SPI_ERROR;
    }

    // 不为ready状态，认为该spi外设正忙
    if (instance->spi_handle_->State != HAL_SPI_STATE_READY) {
        return BSP_SPI_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    // CS信号由owner决定，这里不进行CS操作
    switch (instance->spi_work_mode_) {
        case BSP_SPI_WORK_MODE_BLOCKING:
            status_hal = HAL_SPI_Receive(instance->spi_handle_, rx_buffer, data_size, BSP_SPI_BLOCKING_TIMEOUT);
            return bspSPIGetStatusFromHAL(status_hal);
        case BSP_SPI_WORK_MODE_IT:
            status_hal = HAL_SPI_Receive_IT(instance->spi_handle_, rx_buffer, data_size);
            break;
        case BSP_SPI_WORK_MODE_DMA:
            status_hal = HAL_SPI_Receive_DMA(instance->spi_handle_, rx_buffer, data_size);
            break;
        default:
            return BSP_SPI_ERROR;
    }

    if (status_hal == HAL_OK) {
        // 成功开启异步传输后，保存本次传输任务的缓冲区信息
        // IT DMA传输开启后，该函数返回
        // 一次传输完成后，触发回调，spi实例调用owner回调函数
        // 由于是异步传输，因此spi需要保存本次传输的缓冲区信息
        instance->tx_buffer_ptr_this_transfer_ = NULL;
        instance->rx_buffer_ptr_this_transfer_ = rx_buffer;
        instance->data_size_this_transfer_ = data_size;
    }

    return bspSPIGetStatusFromHAL(status_hal);
}

bool bspSPITxRxIsBusy(bspSPIInstance_t *instance)
{
    return instance->spi_handle_->State == HAL_SPI_STATE_BUSY_TX_RX;
}

bspSPIStatus_e bspSPIChangeWorkMode(bspSPIInstance_t *instance, bspSPIWorkMode_e new_mode)
{
    if (instance == NULL) {
        return BSP_SPI_ERROR;
    }

    if (instance->spi_handle_->State != HAL_SPI_STATE_READY) {
        return BSP_SPI_BUSY;
    }

    instance->spi_work_mode_ = new_mode;

    return BSP_SPI_OK;
}

// IT 和DMA模式下，传输完成会触发回调函数
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    bspSPIInstance_t *instance = bspSPIFindInstanceByHandle(hspi);
    if (instance == NULL) {
        return;
    }

    if (instance->txrx_cplt_callback_ != NULL) {
        instance->txrx_cplt_callback_(instance->owner_, 
                                    instance->tx_buffer_ptr_this_transfer_,
                                    instance->rx_buffer_ptr_this_transfer_,
                                    instance->data_size_this_transfer_);
    }

    // 无论是否有回调，本次传输完成，清空保存的缓冲区信息
    instance->tx_buffer_ptr_this_transfer_ = NULL;
    instance->rx_buffer_ptr_this_transfer_ = NULL;
    instance->data_size_this_transfer_ = 0;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    bspSPIInstance_t *instance = bspSPIFindInstanceByHandle(hspi);
    if (instance == NULL) {
        return;
    }

    if (instance->error_callback_ != NULL) {
        instance->error_callback_(instance->owner_);
    }

    // 无论是否有回调，清空保存的缓冲区信息
    instance->tx_buffer_ptr_this_transfer_ = NULL;
    instance->rx_buffer_ptr_this_transfer_ = NULL;
    instance->data_size_this_transfer_ = 0;
}


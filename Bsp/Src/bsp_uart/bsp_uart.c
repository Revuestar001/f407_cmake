#include "usart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_def.h"
#include "bsp_uart.h"
#include "bsp_uart_private.h"

typedef struct uart_instance 
{
    UART_HandleTypeDef *uart_handle_;

    bspUARTTxMode_e tx_mode_;
    bspUARTRxMode_e rx_mode_;

    uint8_t *rx_buffer_ptr_; // 接收缓冲区指针
    uint16_t rx_buffer_size_;

    const char *name_;

    bspUARTRxEventCallback_f rx_event_callback_;
    bspUARTTxCpltCallback_f tx_cplt_callback_;
    bspUARTErrorCallback_f error_callback_;

    void *owner_;
} bspUARTInstance_t;

static uint8_t uart_memory_index_ = 0;
static bspUARTInstance_t uart_instance_memory_[BSP_UART_MAX_INSTANCE_NUM] = {NULL};

static bspUARTRxEventType_e bspUARTGetRxEventTypeFromHAL(HAL_UART_RxEventTypeTypeDef rx_event_hal)
{
    switch (rx_event_hal) {
        case HAL_UART_RXEVENT_TC:
            return BSP_UART_RX_EVENT_DMA_TC;
        case HAL_UART_RXEVENT_HT:
            return BSP_UART_RX_EVENT_DMA_HT;
        case HAL_UART_RXEVENT_IDLE:
            return BSP_UART_RX_EVENT_IDLE;
        default:
            break;
    }
    return BSP_UART_RX_EVENT_INVALID;
}

static bspUARTStatus_e bspUARTGetStatusFromHAL(HAL_StatusTypeDef status_hal)
{
    switch (status_hal) {
        case HAL_OK:
            return BSP_UART_OK;
        case HAL_ERROR:
            return BSP_UART_ERROR;
        case HAL_BUSY:
            return BSP_UART_BUSY;
        case HAL_TIMEOUT:
            return BSP_UART_TIMEOUT;
        default:
            break;
    }
    return BSP_UART_ERROR;
}

bspUARTInstance_t *bspUARTInit(const bspUARTConfig_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    if (uart_memory_index_ >= BSP_UART_MAX_INSTANCE_NUM) {
        return NULL;
    }

    bspUARTInstance_t *instance = &uart_instance_memory_[uart_memory_index_];
    memset(instance, 0, sizeof(bspUARTInstance_t));

    instance->uart_handle_ = config->uart_handle_;
    instance->tx_mode_ = config->tx_mode_;
    instance->rx_mode_ = config->rx_mode_;
    instance->rx_buffer_ptr_ = config->rx_buffer_ptr_;
    instance->rx_buffer_size_ = config->rx_buffer_size_;
    instance->name_ = config->name_;

    uart_memory_index_ ++;

    return instance;
}

void bspUARTRxEventCallbackRegister(bspUARTInstance_t *instance , void *owner_ptr, bspUARTRxEventCallback_f callback)
{
    instance->rx_event_callback_ = callback;
    instance->owner_ = owner_ptr;
}

void bspUARTTxCpltCallbackRegister(bspUARTInstance_t *instance , void *owner_ptr, bspUARTTxCpltCallback_f callback)
{
    instance->tx_cplt_callback_ = callback;
    instance->owner_ = owner_ptr;
}
void bspUARTErrorCallbackRegister(bspUARTInstance_t *instance , void *owner_ptr, bspUARTErrorCallback_f callback)
{
    instance->error_callback_ = callback;
    instance->owner_ = owner_ptr;
}

bspUARTStatus_e bspUARTRxStart(bspUARTInstance_t *instance)
{
    // 只有DMA传输满和空闲中断触发
    HAL_StatusTypeDef status_hal = HAL_UARTEx_ReceiveToIdle_DMA(instance->uart_handle_, instance->rx_buffer_ptr_, instance->rx_buffer_size_);
    __HAL_DMA_DISABLE_IT(instance->uart_handle_->hdmarx, DMA_IT_HT);

    return bspUARTGetStatusFromHAL(status_hal);
}

bspUARTStatus_e bspUARTTransimt(bspUARTInstance_t *instance, uint8_t *tx_buffer, uint16_t tx_size)
{   
    HAL_StatusTypeDef status_hal;
    switch (instance->tx_mode_) {
        case BSP_UART_TX_MODE_BLOCKING:
            status_hal = HAL_UART_Transmit(instance->uart_handle_, tx_buffer, tx_size, BSP_UART_TX_BLOCKING_TIMEOUT);
            break;
        case BSP_UART_TX_MODE_IT:
            status_hal = HAL_UART_Transmit_IT(instance->uart_handle_, tx_buffer, tx_size);
            break;
        case BSP_UART_TX_MODE_DMA:
            status_hal = HAL_UART_Transmit_DMA(instance->uart_handle_, tx_buffer, tx_size);
            break;
        default:
            return BSP_UART_ERROR;
            break;
    }
    return bspUARTGetStatusFromHAL(status_hal);
}

bool bspUARTTxIsBusy(bspUARTInstance_t *instance)
{
    return instance->uart_handle_->gState == HAL_UART_STATE_BUSY_TX;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t rx_buffer_cur_index = Size;
    bspUARTRxEventType_e rx_event = bspUARTGetRxEventTypeFromHAL(HAL_UARTEx_GetRxEventType(huart));

    for (size_t i = 0; i < uart_memory_index_; i ++) {
        if (uart_instance_memory_[i].uart_handle_ == huart) {
            if (uart_instance_memory_[i].rx_event_callback_ != NULL) {
                uart_instance_memory_[i].rx_event_callback_(uart_instance_memory_[i].owner_, 
                                                            uart_instance_memory_[i].rx_buffer_ptr_, 
                                                            rx_buffer_cur_index, 
                                                            rx_event);
            }
            // 注意这里实现很危险
            // 重新启动接收
            bspUARTRxStart(&uart_instance_memory_[i]);
            break;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (size_t i = 0; i < uart_memory_index_; i ++) {
        if (uart_instance_memory_[i].uart_handle_ == huart) {
            if (uart_instance_memory_[i].tx_cplt_callback_ != NULL) {
                uart_instance_memory_[i].tx_cplt_callback_(uart_instance_memory_[i].owner_);
            }
            break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (size_t i = 0; i < uart_memory_index_; i ++) {
        if (uart_instance_memory_[i].uart_handle_ == huart) {
            if (uart_instance_memory_[i].error_callback_ != NULL) {
                // 为什么不直接 bspUARTRxStart()?
                uart_instance_memory_[i].error_callback_(uart_instance_memory_[i].owner_);
            }
            break;
        }
    }
}

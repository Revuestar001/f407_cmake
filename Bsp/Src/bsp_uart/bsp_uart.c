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
    uint16_t rx_last_pos_; // 上次DMA指针在环形缓冲区的位置，不是数组下标

    const char *name_;

    bspUARTRxEventCallback_f rx_event_callback_;
    bspUARTTxCpltCallback_f tx_cplt_callback_;
    bspUARTErrorCallback_f error_callback_;

    void *rx_owner_;
    void *tx_owner_;
    void *error_owner_;
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
    instance->rx_owner_ = owner_ptr;
}

void bspUARTTxCpltCallbackRegister(bspUARTInstance_t *instance , void *owner_ptr, bspUARTTxCpltCallback_f callback)
{
    instance->tx_cplt_callback_ = callback;
    instance->tx_owner_ = owner_ptr;
}
void bspUARTErrorCallbackRegister(bspUARTInstance_t *instance , void *owner_ptr, bspUARTErrorCallback_f callback)
{
    instance->error_callback_ = callback;
    instance->error_owner_ = owner_ptr;
}

bspUARTStatus_e bspUARTRxStart(bspUARTInstance_t *instance)
{   
    HAL_StatusTypeDef status_hal = HAL_UARTEx_ReceiveToIdle_DMA(instance->uart_handle_, instance->rx_buffer_ptr_, instance->rx_buffer_size_);
    if (status_hal == HAL_OK) {
        // 成功启动接收时
        // 重置上一次DMA指向的位置为0
        instance->rx_last_pos_ = 0;
    }
    
    // DMA循环模式下，不再关闭DMA半满中断
    // __HAL_DMA_DISABLE_IT(instance->uart_handle_->hdmarx, DMA_IT_HT);

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

// DMA循环模式，此时uint16_t Size为DMA硬件指针指向的数组中的绝对偏移
// 也就是Size = 10时，数组[0, ..., 9]数据有效
// 也就是说，发生TC事件时，Size = sizeof(buffer)，作为下标会越界
// 配合rx_last_pos_可计算本次DMA触发中断所传输的字节数
// rx_last_pos：上次交付到上层的尾后位置
// rx_cur_pos：这次 DMA 可用数据的尾后位置
// 新增数据：
// [rx_last_pos, rx_cur_pos)
// 如果DMA绕回：
// [rx_last_pos, buffer_size) + [0, rx_cur_pos)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    bspUARTRxEventType_e rx_event = bspUARTGetRxEventTypeFromHAL(HAL_UARTEx_GetRxEventType(huart));

    for (size_t i = 0; i < uart_memory_index_; i ++) {
        if (uart_instance_memory_[i].uart_handle_ == huart) {
            bspUARTInstance_t *instance = &uart_instance_memory_[i];

            uint16_t rx_cur_pos = Size;
            uint16_t rx_last_pos = instance->rx_last_pos_;
            uint16_t rx_buffer_size = instance->rx_buffer_size_;

            if (rx_buffer_size == 0 || instance->rx_buffer_ptr_ == NULL) {
                return;
            }
            
            // 两个越界保护，不应该被触发
            if (rx_cur_pos > rx_buffer_size) {
                rx_cur_pos = rx_buffer_size;
            }
            if (rx_last_pos >= rx_buffer_size) {
                rx_last_pos = 0;
                instance->rx_last_pos_ = 0;
            }

            if (rx_cur_pos != rx_last_pos) {
                if (instance->rx_event_callback_ != NULL) {
                    // rx_cur_pos > rx_last_pos
                    // 正常收到连续的一段
                    // 或
                    // rx_cur_pos < rx_last_pos
                    // DMA从缓冲区绕回，数据分为两段
                    // owner自行拆为两段数据
                    bspUARTRxEventContext_t rx_context = {
                        .rx_buffer_ptr_ = instance->rx_buffer_ptr_,
                        .rx_buffer_size_ = rx_buffer_size,
                        .rx_data_start_index_ = rx_last_pos,
                        .rx_data_end_pos_ = rx_cur_pos,
                        .rx_data_len_ = (rx_cur_pos < rx_last_pos) ? ((rx_buffer_size - rx_last_pos) + rx_cur_pos) : (rx_cur_pos - rx_last_pos),
                        .rx_event_ = rx_event,
                    };
                    instance->rx_event_callback_(instance->rx_owner_, &rx_context);  
                }    
            } else {
                // rx_cur_pos = rx_last_pos
                // 可能没有新数据，也可能DMA绕了一整圈发生覆盖，这里开启了TC HT IDLE中断，默认处理速度足够
            }

            // 无论回调函数是否注册，都更新rx_last_pos_
            // 发生TC事件时，Size = sizeof(buffer)，作为下标会越界，取余
            instance->rx_last_pos_ = rx_cur_pos % rx_buffer_size;

            break;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (size_t i = 0; i < uart_memory_index_; i ++) {
        if (uart_instance_memory_[i].uart_handle_ == huart) {
            if (uart_instance_memory_[i].tx_cplt_callback_ != NULL) {
                uart_instance_memory_[i].tx_cplt_callback_(uart_instance_memory_[i].tx_owner_);
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
                uart_instance_memory_[i].error_callback_(uart_instance_memory_[i].error_owner_);
            }
            // 简单处理，重启接收
            bspUARTRxStart(&uart_instance_memory_[i]);
            break;
        }
    }
}

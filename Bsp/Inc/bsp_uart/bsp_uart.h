#pragma once

#include "bsp_def.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BSP_UART_OK = 0,
    BSP_UART_ERROR,
    BSP_UART_BUSY,
    BSP_UART_TIMEOUT
} bspUARTStatus_e;

typedef enum
{
    BSP_UART_RX_EVENT_DMA_TC = 0,
    BSP_UART_RX_EVENT_DMA_HT,
    BSP_UART_RX_EVENT_IDLE,
    BSP_UART_RX_EVENT_INVALID
} bspUARTRxEventType_e;

// 结构体前向声明
typedef struct uart_instance bspUARTInstance_t;
typedef void (*bspUARTRxEventCallback_f)(void *owner_ptr, uint8_t *rx_buffer_ptr, uint16_t rx_buffer_cur_index, bspUARTRxEventType_e rx_event) ; // 接受事件中断回调函数指针
typedef void (*bspUARTTxCpltCallback_f)(void *owner_ptr) ; // 发送完成中断回调函数指针
typedef void (*bspUARTErrorCallback_f)(void *owner_ptr) ; // 错误中断回调函数指针

void bspUARTRxEventCallbackRegister(bspUARTInstance_t *instance , void *owner_ptr, bspUARTRxEventCallback_f callback);
void bspUARTTxCpltCallbackRegister(bspUARTInstance_t *instance , void *owner_ptr, bspUARTTxCpltCallback_f callback);
void bspUARTErrorCallbackRegister(bspUARTInstance_t *instance , void *owner_ptr, bspUARTErrorCallback_f callback);

bspUARTStatus_e bspUARTRxStart(bspUARTInstance_t *instance);
bspUARTStatus_e bspUARTTransimt(bspUARTInstance_t *instance, uint8_t *tx_buffer, uint16_t tx_size);
bool bspUARTTxIsBusy(bspUARTInstance_t *instance);

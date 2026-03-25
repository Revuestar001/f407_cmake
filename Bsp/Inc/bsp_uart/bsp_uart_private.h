#pragma once
#include "usart.h"

#include <stdint.h>

#include "bsp_uart.h"
#include "bsp_def.h"

typedef enum
{
    BSP_UART_TX_MODE_BLOCKING = 0,
    BSP_UART_TX_MODE_IT,
    BSP_UART_TX_MODE_DMA,
} bspUARTTxMode_e;

typedef enum
{
    BSP_UART_RX_MODE_BLOCKING = 0,
    BSP_UART_RX_MODE_IT,
    BSP_UART_RX_MODE_DMA,
    BSP_UART_RX_MODE_DMA_IDLE // DMA + 空闲中断
} bspUARTRxMode_e;

typedef struct uart_config
{
    UART_HandleTypeDef *uart_handle_;

    bspUARTTxMode_e tx_mode_;
    bspUARTRxMode_e rx_mode_;

    uint8_t *rx_buffer_ptr_; // 接受缓冲区指针
    uint16_t rx_buffer_size_;

    const char *name_;
} bspUARTConfig_t;

bspUARTInstance_t *bspUARTInit(const bspUARTConfig_t *config);
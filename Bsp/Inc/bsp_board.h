#pragma once
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_def.h"

typedef enum 
{
    BSP_GPIO_USER_LED_BLUE = 0,
    BSP_GPIO_MAX
} bspGPIOId_e;

bspGPIOInstance_t *bspBoardGetGPIOInstance(bspGPIOId_e gpio_id);

typedef enum 
{
    BSP_UART_PRINT = 0,
    BSP_UART_MAX
} bspUARTId_e;

bspUARTInstance_t *bspBoardGetUARTInstance(bspUARTId_e uart_id);

void bspBoardInit();
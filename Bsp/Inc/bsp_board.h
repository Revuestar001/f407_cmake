#pragma once
#include "bsp_gpio.h"
#include "bsp_def.h"

typedef enum 
{
    BSP_GPIO_USER_LED_BLUE = 0,
    BSP_GPIO_MAX
} bspGPIOId_e;

bspGPIOInstance_t *bspBoardGetGPIOInstance(bspGPIOId_e gpio_id);

void bspBoardInit();
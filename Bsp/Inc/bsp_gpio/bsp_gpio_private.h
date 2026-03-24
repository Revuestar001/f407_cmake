#pragma once
#include <stdint.h>

#include "gpio.h"

#include "bsp_gpio.h"
#include "bsp_def.h"

typedef enum
{
    BSP_GPIO_ACTIVE_HIGH = 0, // 高电平时逻辑有效
    BSP_GPIO_ACTIVE_LOW
} bspGPIOActiveLevel_e;

typedef struct gpio_config
{
    GPIO_TypeDef *port_; // 端口，只保存指针
    uint16_t pin_; // 引脚
    
    bspGPIOActiveLevel_e active_level_; // 逻辑有效电平
    const char *name_; // 名称
} bspGPIOConfig_t;

// 线程不安全，必须在osKernelStart() 前调用
bspGPIOInstance_t *bspGPIOInit(const bspGPIOConfig_t *config);
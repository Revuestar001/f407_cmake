#include "gpio.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_gpio.h"
#include "bsp_gpio_private.h"
#include "bsp_def.h"

typedef struct gpio_instance
{
    GPIO_TypeDef *port_;
    uint16_t pin_;
    bspGPIOActiveLevel_e active_level_; // 逻辑有效电平
    const char *name_; // 名称

    bspGPIOIsrCallback_f isr_callback_; // EXIT中断回调函数指针

    void *owner_; // 指向这个GPIO实例拥有者的指针
} bspGPIOInstance_t;

static uint8_t gpio_memory_index_ = 0;
static bspGPIOInstance_t gpio_instance_memory_[BSP_GPIO_MAX_INSTANCE_NUM] = {0};

// 线程不安全，必须在osKernelStart() 前调用
bspGPIOInstance_t *bspGPIOInit(const bspGPIOConfig_t *config)
{
    if (config == NULL) {
        return NULL;
    }

    if (gpio_memory_index_ >= BSP_GPIO_MAX_INSTANCE_NUM) {
        return NULL;
    }

    bspGPIOInstance_t *instance = &gpio_instance_memory_[gpio_memory_index_];
    memset(instance, 0, sizeof(*instance));
    
    instance->port_ = config->port_;
    instance->pin_ = config->pin_;
    instance->active_level_ = config->active_level_;
    instance->name_ = config->name_;

    gpio_memory_index_++;

    return instance;
}

// 线程不安全，必须在osKernelStart() 前调用
void bspGPIOIsrCallbackRegister(bspGPIOInstance_t *instance , void *owner_ptr, bspGPIOIsrCallback_f callback)
{
    instance->isr_callback_ = callback;
    instance->owner_ = owner_ptr;
}

void bspGPIOWriteRaw(bspGPIOInstance_t *instance, bool write_state)
{
    HAL_GPIO_WritePin(instance->port_, instance->pin_, (GPIO_PinState)write_state);
}

void bspGPIOWriteLogic(bspGPIOInstance_t *instance, bool write_state)
{
    if (instance->active_level_ == BSP_GPIO_ACTIVE_LOW) {
        write_state = !write_state;
    }
    HAL_GPIO_WritePin(instance->port_, instance->pin_, (GPIO_PinState)write_state);
}

bool bspGPIOReadRaw(bspGPIOInstance_t *instance)
{
    return (bool)HAL_GPIO_ReadPin(instance->port_, instance->pin_);
}

bool bspGPIOReadLogic(bspGPIOInstance_t *instance)
{       
    bool pin_state = (bool)HAL_GPIO_ReadPin(instance->port_, instance->pin_);
    if (instance->active_level_ == BSP_GPIO_ACTIVE_LOW) {
        pin_state = !pin_state;
    }
    
    return pin_state;
}

void bspGPIOToggle(bspGPIOInstance_t *instance)
{
    HAL_GPIO_TogglePin(instance->port_, instance->pin_);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (size_t i = 0; i < gpio_memory_index_; i++)
    {
        if (gpio_instance_memory_[i].pin_ == GPIO_Pin && gpio_instance_memory_[i].isr_callback_ != NULL)
        {
            gpio_instance_memory_[i].isr_callback_((void *)gpio_instance_memory_[i].owner_, &gpio_instance_memory_[i]);
            return;
        }
    }
}


#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "bsp_def.h"

// 结构体前置声明
typedef struct gpio_instance bspGPIOInstance_t;
typedef void (*bspGPIOIsrCallback_f)(void *owner, bspGPIOInstance_t *instance) ; // EXIT中断回调函数指针

// 注册EXIT外部中断触发回调函数
// 线程不安全，必须在osKernelStart() 前调用
void bspGPIOIsrCallbackRegister(bspGPIOInstance_t *instance , void *owner_ptr, bspGPIOIsrCallback_f callback);

void bspGPIOWriteRaw(bspGPIOInstance_t *instance, bool write_state);
void bspGPIOWriteLogic(bspGPIOInstance_t *instance, bool write_state);
bool bspGPIOReadRaw(bspGPIOInstance_t *instance);
bool bspGPIOReadLogic(bspGPIOInstance_t *instance);
void bspGPIOToggle(bspGPIOInstance_t *instance);
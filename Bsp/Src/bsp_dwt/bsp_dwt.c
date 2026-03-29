#include "stm32f407xx.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_dwt.h"

static bool dwt_is_initialized_ = false;

void bspDWTInit()
{
    if (dwt_is_initialized_ == true) {
        return;
    }
    // 使能DWT，由DEMCR寄存器第24位控制
    CoreDebug->DEMCR |= (uint32_t)1 << 24;
    // 必须先清零CYCCNT寄存器
    DWT->CYCCNT = (uint32_t)0;
    // 使能CYCCNT寄存器，由DWT_CTRL(0xE0001000)的第0位控制
    DWT->CTRL = (uint32_t)1 << 0U;

    dwt_is_initialized_ = true;
}

// static void bspDWTResetCount(void)
// {
//     DWT->CYCCNT = 0U;
// }

uint32_t bspDWTGetCount(void)
{
    return DWT->CYCCNT;
}

// 84MHz下最大51s
uint32_t bspDWTGetElapsedTimeUs(uint32_t start_cnt)
{
    if (dwt_is_initialized_ == false) {
        return 0U;
    }

    uint32_t current_cnt = bspDWTGetCount();
    uint32_t elapsed_cnt = 0;

    // 假定只绕回一圈
    elapsed_cnt = current_cnt - start_cnt;

    return (uint32_t)(((uint64_t)elapsed_cnt * 1000000U) / SystemCoreClock);
}

// 84MHz下最大51s
void bspDWTDelayUs(uint32_t time_us)
{
    if (dwt_is_initialized_ == false || time_us == 0U) {
        return;
    }

    uint32_t start_cnt = DWT->CYCCNT;
    uint32_t elapsed_cnt = (uint32_t)(((uint64_t)SystemCoreClock * time_us) / 1000000U);

    while ((uint32_t)(DWT->CYCCNT - start_cnt) < elapsed_cnt) {
    }
}

void bspDWTDelayMs(uint32_t time_ms)
{
    for (uint32_t i = 0; i < time_ms; i++) {
        bspDWTDelayUs(1000U);
    }
}
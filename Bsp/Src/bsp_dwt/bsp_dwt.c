#include "stm32f407xx.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_dwt.h"

static bool dwt_is_initialized_ = false;
static uint32_t dwt_last_cnt_ = 0;
static uint64_t dwt_timestamp_us_ = 0;
static uint64_t dwt_remainder_ = 0; // 除法余数

// 进入临界区
static uint32_t bspDWTEnterCritical(void)
{   
    // 获取当前可屏蔽中断的状态
    // primask为1表示可屏蔽中断关闭
    // 在关中断前，先记录中断是否已经关闭
    uint32_t primask = __get_PRIMASK();
    __disable_irq(); // 关中断
    return primask;
}

// 退出临界区
static void bspDWTExitCritical(uint32_t primask)
{
    // 设置可屏蔽中断的状态来恢复到关中断前的状态
    // 防止在进入临界区前本来是关中断的情况下，退出临界区反而打开了中断
    // 所以有可能退出临界区时还是关中断状态
    __set_PRIMASK(primask);
}

void bspDWTInit(void)
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

    dwt_last_cnt_ = 0U;
    dwt_timestamp_us_ = 0ULL;
    dwt_remainder_ = 0ULL;

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

uint64_t bspDWTGetAbsTimeUs(void)
{   
    if (dwt_is_initialized_ == false) {
        return 0ULL;
    }

    // 对uint64_t读写不是原子操作，而且这个函数多个地方都要调用，因此进入临界区
    uint32_t primask = bspDWTEnterCritical();
    uint32_t now_cnt = bspDWTGetCount();
    uint32_t delta_cnt = now_cnt - dwt_last_cnt_;

    // 不直接用bspDWTGetElapsedTimeUs累加，是为了防止累积舍入误差
    // 加上除法产生的余数
    uint64_t with_remainder = (uint64_t)delta_cnt * 1000000ULL + dwt_remainder_;
    dwt_timestamp_us_ += with_remainder / SystemCoreClock; // 除法产生余数
    dwt_remainder_ = with_remainder % SystemCoreClock; // 保存本次除法的余数

    dwt_last_cnt_ = now_cnt;

    // 在临界区内保存时间戳，防止在临界区外返回静态全局变量
    uint64_t timestamp_us = dwt_timestamp_us_;

    // 退出临界区
    bspDWTExitCritical(primask);

    return timestamp_us;
}
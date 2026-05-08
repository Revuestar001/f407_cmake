#include "stm32g431xx.h"

#include <stdint.h>

#include "bsp_critical.h"

bspCriticalIRQState_t bspCriticalEnter(void)
{   
    // 获取当前可屏蔽中断的状态
    // primask为1表示可屏蔽中断关闭
    // 在关中断前，先记录中断是否已经关闭
    bspCriticalIRQState_t primask = __get_PRIMASK();
    __disable_irq(); // 关中断
    return primask;
}

void bspCriticalExit(bspCriticalIRQState_t primask)
{
    // 设置可屏蔽中断的状态来恢复到关中断前的状态
    // 防止在进入临界区前本来是关中断的情况下，退出临界区反而打开了中断
    // 所以有可能退出临界区时还是关中断状态
    __set_PRIMASK(primask);
}
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "app_def.h"

#define APP_INS_TRY_INIT_MAX_COUNT 4U
#define APP_INS_TRY_INIT_DELAY_MS 100U
#define APP_INS_MAX_UPDATE_WAIT_MS 10U // 最低更新频率100Hz

typedef struct app_INS_data
{
    // 包含输出状态，暂时留空因为没有ekf

    uint64_t timestamp_;
} appINSData_t;

// 任务循环
void appINSTaskEntry(void *argument);
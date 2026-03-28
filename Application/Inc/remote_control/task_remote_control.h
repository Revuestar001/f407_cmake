#pragma once
#include "FreeRTOS.h"

#include <stdbool.h>

#include "rc_mapper.h"

void taskRemoteControlInit();

// 返回是否成功更新一帧RC指令
bool taskRemoteControlUpdate(TickType_t timeout_tick);
const moduleRCMapper_t *taskRemoteControlGetRCMapped();

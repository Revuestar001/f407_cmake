#pragma once
#include "FreeRTOS.h"

#include <stdbool.h>

#include "rc_mapper.h"

void appRemoteControlInit();

// 返回是否成功更新一帧RC指令
bool appRemoteControlUpdate(TickType_t timeout_tick);
// 拷贝返回，防止读数据撕裂
// 注意，这里目前只输出摇杆值
moduleRCMapper_t appRemoteControlGetRCMapped();

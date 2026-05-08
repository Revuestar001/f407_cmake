#pragma once
#include "fdcan.h"

#include <stdint.h>

#include "bsp_def.h"
#include "bsp_fdcan.h"

typedef struct fdcan_config 
{
    FDCAN_HandleTypeDef *fdcan_handle_;

    const char *name_;
} bspFDCANConfig_t;

bspFDCANInstance_t *bspFDCANInit(const bspFDCANConfig_t *config);
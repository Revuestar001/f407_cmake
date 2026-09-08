#pragma once
#include "can.h"

#include <stdint.h>

#include "bsp_def.h"
#include "bsp_can.h"

typedef struct can_config 
{
    CAN_HandleTypeDef *can_handle_;

    uint8_t filter_bank_base_;

    const char *name_;
} bspCANConfig_t;

bspCANInstance_t *bspCANInit(const bspCANConfig_t *config);

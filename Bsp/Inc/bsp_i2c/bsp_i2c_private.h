#pragma once
#include "i2c.h"

#include "bsp_def.h"
#include "bsp_i2c.h"

typedef struct i2c_config
{
    I2C_HandleTypeDef *i2c_handle_;
    bspI2CWorkMode_e i2c_work_mode_;

    const char *name_;
} bspI2CConfig_t;

bspI2CInstance_t *bspI2CInit(const bspI2CConfig_t *config);
#pragma once

#include <stdint.h>

typedef enum
{
    BSP_FLASH_OK = 0,
    BSP_FLASH_ERROR,
    BSP_FLASH_INVALID_ADDR,
    BSP_FLASH_INVALID_SIZE,
    BSP_FLASH_HAL_ERROR,
} bspFlashStatus_e;

bspFlashStatus_e bspFlashRead(uint32_t address, void *data_out, uint32_t data_length);

bspFlashStatus_e bspFlashErase(uint32_t address, uint32_t data_length);

bspFlashStatus_e bspFlashWrite(uint32_t address, const void *data_in, uint32_t data_length);
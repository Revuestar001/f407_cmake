#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bsp_flash.h"
#include "bsp_flash_cfg.h"

static bool checkAddressIsValid(uint32_t address, uint32_t data_length)
{
    if (address < BSP_FLASH_START_ADDR || address > BSP_FLASH_END_ADDR) {
        return false;
    }

    uint32_t end = address + data_length;

    if (end > BSP_FLASH_END_ADDR) {
        return false;
    }

    return true;
}

bspFlashStatus_e bspFlashRead(uint32_t address, void *data_out, uint32_t data_length)
{
    if (data_out == NULL) {
        return BSP_FLASH_ERROR;
    }

    if (data_length == 0 || data_length > BSP_FLASH_SIZE) {
        return BSP_FLASH_INVALID_SIZE;
    }

    if (checkAddressIsValid(address, data_length) == false) {
        return BSP_FLASH_INVALID_ADDR;
    }

    
}

bspFlashStatus_e bspFlashErase(uint32_t address, uint32_t data_length);

bspFlashStatus_e bspFlashWrite(uint32_t address, const void *data_in, uint32_t data_length);
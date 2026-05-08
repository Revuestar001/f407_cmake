#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    UTIL_CRC16_VERSION_MODBUS = 0,
    UTIL_CRC16_VERSION_CCITT_FALSE,
} utilCRC16Version_e;

bool utilCRC16Calculate(const uint8_t *data_in, uint32_t data_length, utilCRC16Version_e crc_version, uint16_t *res_out);
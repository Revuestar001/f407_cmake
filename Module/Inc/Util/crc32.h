#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    UTIL_CRC32_VERSION_IEEE = 0,
} utilCRC32Version_e;

bool utilCRC32Calculate(const uint8_t *data_in, uint32_t data_length, utilCRC32Version_e crc_version, uint32_t *res_out);
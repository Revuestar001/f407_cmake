#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "crc32.h"

#define CRC32_IEEE_POLY 0x04C11DB7U
#define CRC32_IEEE_POLY_REFLECT 0xEDB88320U
#define CRC32_IEEE_INIT 0xFFFFFFFFU
#define CRC32_IEEE_XOROUT 0xFFFFFFFFU

static bool checkVersionIsValid(utilCRC32Version_e crc_version)
{
    switch (crc_version) {
        case UTIL_CRC32_VERSION_IEEE:
            break;
        default:
            return false;
    }

    return true;
}

// IEEE版本，位运算法
static uint32_t CRC32CalculateBitsIEEE(const uint8_t *data_in, uint32_t data_length)
{
    uint32_t crc = CRC32_IEEE_INIT;

    for (size_t i = 0; i < data_length; i++) {
        crc ^= data_in[i];

        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ CRC32_IEEE_POLY_REFLECT;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc ^ CRC32_IEEE_XOROUT;
}

bool utilCRC32Calculate(const uint8_t *data_in, uint32_t data_length, utilCRC32Version_e crc_version, uint32_t *res_out)
{
    if (data_in == NULL || data_length == 0U) {
        return false;
    }

    if (checkVersionIsValid(crc_version) == false) {
        return false;
    }

    switch (crc_version) {
        case UTIL_CRC32_VERSION_IEEE:
            *res_out = CRC32CalculateBitsIEEE(data_in, data_length);
            break;
        default:
            return false;
    }

    return true;
}
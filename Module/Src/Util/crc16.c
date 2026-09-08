#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "crc16.h"

#define CRC16_MODBUS_POLY 0xA001U
#define CRC16_MODBUS_INIT 0xFFFFU
#define CRC16_MODBUS_XOROUT 0x0000U

#define CRC16_CCITT_FALSE_POLY 0x1021U
#define CRC16_CCITT_FALSE_INIT 0xFFFFU
#define CRC16_CCITT_FALSE_XOROUT 0x0000U

static bool checkVersionIsValid(utilCRC16Version_e crc_version)
{
    switch (crc_version) {
        case UTIL_CRC16_VERSION_MODBUS:
        case UTIL_CRC16_VERSION_CCITT_FALSE:
            break;
        default:
            return false;
    }

    return true;
}

// MODBUS版本，位运算法
static uint16_t CRC16CalculateBitsModbus(const uint8_t *data_in, uint32_t data_length)
{
    uint16_t crc = CRC16_MODBUS_INIT;

    for (size_t i = 0; i < data_length; i++) {
        crc ^= data_in[i];

        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (crc >> 1U) ^ CRC16_MODBUS_POLY;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

// CCITT-FALSE版本，位运算法
static uint16_t CRC16CalculateBitsCCITTFalse(const uint8_t *data_in, uint32_t data_length)
{
    uint16_t crc = CRC16_CCITT_FALSE_INIT;

    for (size_t i = 0; i < data_length; i++) {
        crc ^= ((uint16_t)data_in[i] << 8U);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ CRC16_CCITT_FALSE_POLY);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

bool utilCRC16Calculate(const uint8_t *data_in, uint32_t data_length, utilCRC16Version_e crc_version, uint16_t *res_out)
{
    if (data_in == NULL || data_length == 0U) {
        return false;
    }

    if (checkVersionIsValid(crc_version) == false) {
        return false;
    }

    switch (crc_version) {
        case UTIL_CRC16_VERSION_MODBUS:
            *res_out = CRC16CalculateBitsModbus(data_in, data_length);
            break;
        case UTIL_CRC16_VERSION_CCITT_FALSE:
            *res_out = CRC16CalculateBitsCCITTFalse(data_in, data_length);
        default:
            return false;
    }

    return true;
}
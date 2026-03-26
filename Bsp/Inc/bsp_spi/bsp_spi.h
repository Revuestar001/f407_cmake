#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "bsp_def.h"

typedef enum
{
    BSP_SPI_OK = 0,
    BSP_SPI_ERROR,
    BSP_SPI_BUSY,
    BSP_SPI_TIMEOUT
} bspSPIStatus_e;

typedef struct spi_instance bspSPIInstance_t;

typedef void (*bspSPITxRxCpltCallback_f)(void *owner, uint8_t *tx_buffer_ptr, uint8_t *rx_buffer_ptr, uint16_t data_size);
typedef void (*bspSPIErrorCallback_f)(void *owner);

void bspSPITxRxCpltCallbackRegister(bspSPIInstance_t *instance, void *owner_ptr, bspSPITxRxCpltCallback_f callback);
void bspSPIErrorCallbackRegister(bspSPIInstance_t *instance, void *owner_ptr, bspSPIErrorCallback_f callback);

bspSPIStatus_e bspSPITransmitReceive(bspSPIInstance_t *instance, uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t data_size);
bool bspSPITxRxIsBusy(bspSPIInstance_t *instance);
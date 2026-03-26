#pragma once
#include "spi.h"

#include <stdint.h>

#include "bsp_def.h"
#include "bsp_spi.h"


typedef enum
{
    BSP_SPI_WORK_MODE_BLOCKING = 0,
    BSP_SPI_WORK_MODE_IT,
    BSP_SPI_WORK_MODE_DMA
} bspSPIWorkMode_e;

typedef struct spi_config
{
    SPI_HandleTypeDef *spi_handle_;
    bspSPIWorkMode_e spi_work_mode_;

    // 初始化时不需要像UART设置接收缓冲区指针和大小
    // spi的发送和接收都需要主动发起，因此缓冲区由上层(一定是owner吗？不一定，由传输任务发起方)维护

    const char *name_;
} bspSPIConfig_t;

bspSPIInstance_t *bspSPIInit(const bspSPIConfig_t *config);


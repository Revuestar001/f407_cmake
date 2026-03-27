#pragma once
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_spi.h"
#include "bsp_can.h"
#include "bsp_def.h"

// GPIO
typedef enum 
{
    BSP_GPIO_USER_LED_BLUE = 0,
    BSP_GPIO_IMU_CS1_ACCEL,
    BSP_GPIO_IMU_CS1_GYRO,
    BSP_GPIO_IMU_INT1_ACCEL, // EXTI
    BSP_GPIO_IMU_INT1_GYRO, // EXTI
    BSP_GPIO_MAX
} bspGPIOId_e;

bspGPIOInstance_t *bspBoardGetGPIOInstance(bspGPIOId_e gpio_id);

// UART
typedef enum 
{
    BSP_UART_PRINT = 0,
    BSP_UART_MAX
} bspUARTId_e;

bspUARTInstance_t *bspBoardGetUARTInstance(bspUARTId_e uart_id);

// SPI
typedef enum 
{
    BSP_SPI_IMU = 0,
    BSP_SPI_MAX
} bspSPIId_e;

bspSPIInstance_t *bspBoardGetSPIInstance(bspSPIId_e spi_id);

// CAN
typedef enum 
{
    BSP_CAN_1 = 0,
    BSP_CAN_MAX
} bspCANId_e;

bspCANInstance_t *bspBoardGetCANInstance(bspCANId_e can_id);

// BOARD
void bspBoardInit();

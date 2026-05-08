#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_fdcan.h"
#include "bsp_spi.h"
#include "main.h"

#include "bsp_board.h"
#include "bsp_def.h"
#include "bsp_gpio_private.h"
#include "bsp_uart_private.h"
#include "bsp_spi_private.h"
// #include "bsp_can_private.h"
#include "bsp_fdcan_private.h"
#include "bsp_i2c_private.h"
#include "bsp_dwt.h"

// 储存所有GPIO实例的指针
static bspGPIOInstance_t *bspBoardGPIOInstancePtrArray[BSP_GPIO_MAX] = {0};
// 储存所有UART实例的指针
static bspUARTInstance_t *bspBoardUARTInstancePtrArray[BSP_UART_MAX] = {0};
static uint8_t bspUARTRxBuffer[BSP_UART_MAX][BSP_UART_RX_BUFFER_SIZE] = {0};
// 储存所有SPI实例的指针
static bspSPIInstance_t *bspBoardSPIInstancePtrArray[BSP_SPI_MAX] = {0};
// // 储存所有CAN实例的指针
// static bspCANInstance_t *bspBoardCANInstancePtrArray[BSP_CAN_MAX] = {0};
// 储存所有FDCAN实例的指针
static bspFDCANInstance_t *bspBoardFDCANInstancePtrArray[BSP_FDCAN_MAX] = {0};
// 储存所有I2C实例的指针
static bspI2CInstance_t *bspBoardI2CInstancePtrArray[BSP_I2C_MAX] = {0};

//
// GPIO
//
static bspGPIOConfig_t bspBoardSetGPIOConfig(GPIO_TypeDef *gpio_port, 
                                            uint16_t gpio_pin, 
                                            bspGPIOActiveLevel_e gpio_activate_level, 
                                            const char *gpio_name)
{
    bspGPIOConfig_t gpio_config;
    gpio_config.port_ = gpio_port;
    gpio_config.pin_ = gpio_pin;
    gpio_config.active_level_ = gpio_activate_level;
    gpio_config.name_ = gpio_name;

    return gpio_config;
}

static void bspBoardGPIOInit()
{   
    // bspGPIOConfig_t gpio_config = {0};
    // // LED
    // gpio_config = bspBoardSetGPIOConfig(LED_BLUE_GPIO_Port, LED_BLUE_Pin, BSP_GPIO_ACTIVE_HIGH, "USER_LED_BLUE");
    // bspBoardGPIOInstancePtrArray[BSP_GPIO_USER_LED_BLUE] = bspGPIOInit(&gpio_config);

    // // SPI CS
    // // 认为CS选中(有效)时，为低电平，初始化后，应该立即拉高CS电平
    // gpio_config = bspBoardSetGPIOConfig(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, BSP_GPIO_ACTIVE_LOW, "CS1_ACCEL");
    // bspBoardGPIOInstancePtrArray[BSP_GPIO_IMU_CS1_ACCEL] = bspGPIOInit(&gpio_config);
    // bspGPIOWriteLogic(bspBoardGPIOInstancePtrArray[BSP_GPIO_IMU_CS1_ACCEL], false);
    // gpio_config = bspBoardSetGPIOConfig(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, BSP_GPIO_ACTIVE_LOW, "CS1_GYRO");
    // bspBoardGPIOInstancePtrArray[BSP_GPIO_IMU_CS1_GYRO] = bspGPIOInit(&gpio_config);
    // bspGPIOWriteLogic(bspBoardGPIOInstancePtrArray[BSP_GPIO_IMU_CS1_GYRO], false);

    // // MAG RESET
    // // 低电平有效
    // gpio_config = bspBoardSetGPIOConfig(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, BSP_GPIO_ACTIVE_LOW, "RSTN_IST8310");
    // bspBoardGPIOInstancePtrArray[BSP_GPIO_MAG_RSTN] = bspGPIOInit(&gpio_config);
    // bspGPIOWriteLogic(bspBoardGPIOInstancePtrArray[BSP_GPIO_MAG_RSTN], false);

    // // EXTI
    // // 逻辑有效电平怎么给？
    // gpio_config = bspBoardSetGPIOConfig(INT1_ACCEL_GPIO_Port, INT1_ACCEL_Pin, BSP_GPIO_ACTIVE_HIGH, "INT1_ACCEL");
    // bspBoardGPIOInstancePtrArray[BSP_GPIO_IMU_INT1_ACCEL] = bspGPIOInit(&gpio_config);
    // gpio_config = bspBoardSetGPIOConfig(INT1_GYRO_GPIO_Port, INT1_GYRO_Pin, BSP_GPIO_ACTIVE_HIGH, "INT1_GYRO");
    // bspBoardGPIOInstancePtrArray[BSP_GPIO_IMU_INT1_GYRO] = bspGPIOInit(&gpio_config);
    // gpio_config = bspBoardSetGPIOConfig(DRDY_IST8310_GPIO_Port, DRDY_IST8310_Pin, BSP_GPIO_ACTIVE_HIGH, "DRDY_IST8310");
    // bspBoardGPIOInstancePtrArray[BSP_GPIO_MAG_DRDY] = bspGPIOInit(&gpio_config);
}

bspGPIOInstance_t *bspBoardGetGPIOInstance(bspGPIOId_e gpio_id)
{
    if (gpio_id >= BSP_GPIO_MAX) {
        return NULL;
    }

    return bspBoardGPIOInstancePtrArray[gpio_id];
}

//
// UART
//
static bspUARTConfig_t bspBoardSetUARTConfig(UART_HandleTypeDef *uart_handle, 
                                            bspUARTRxMode_e rx_mode, 
                                            bspUARTTxMode_e tx_mode, 
                                            uint8_t *rx_buffer_ptr,
                                            uint16_t rx_buffer_size,
                                            const char *uart_name)
{
    bspUARTConfig_t uart_config;
    uart_config.uart_handle_ = uart_handle;
    uart_config.rx_mode_ = rx_mode;
    uart_config.tx_mode_ = tx_mode;
    uart_config.rx_buffer_ptr_ = rx_buffer_ptr;
    uart_config.rx_buffer_size_ = rx_buffer_size;
    uart_config.name_ = uart_name;

    return uart_config;
}

static void bspBoardUARTInit()
{   
    bspUARTConfig_t uart_config = {0};

    uart_config = bspBoardSetUARTConfig(&UART_PRINT, 
                                        BSP_UART_RX_MODE_DMA_IDLE, 
                                        BSP_UART_TX_MODE_DMA, 
                                        &bspUARTRxBuffer[BSP_UART_PRINT][0],
                                        BSP_UART_RX_BUFFER_SIZE, 
                                        "PRINT");
    bspBoardUARTInstancePtrArray[BSP_UART_PRINT] = bspUARTInit(&uart_config);

    // uart_config = bspBoardSetUARTConfig(&UART_SBUS, 
    //                                     BSP_UART_RX_MODE_DMA_IDLE, 
    //                                     BSP_UART_TX_MODE_DMA, 
    //                                     &bspUARTRxBuffer[BSP_UART_SBUS][0],
    //                                     BSP_UART_RX_BUFFER_SIZE, 
    //                                     "SBUS");
    // bspBoardUARTInstancePtrArray[BSP_UART_SBUS] = bspUARTInit(&uart_config);
}

bspUARTInstance_t *bspBoardGetUARTInstance(bspUARTId_e uart_id)
{
    if (uart_id >= BSP_UART_MAX) {
        return NULL;
    }

    return bspBoardUARTInstancePtrArray[uart_id];
}

//
// SPI
//
static bspSPIConfig_t bspBoardSetSPIConfig(SPI_HandleTypeDef *spi_handle, 
                                            bspSPIWorkMode_e spi_work_mode, 
                                            const char *spi_name)
{
    bspSPIConfig_t spi_config;
    spi_config.spi_handle_ = spi_handle;
    spi_config.spi_work_mode_ = spi_work_mode;
    spi_config.name_ = spi_name;

    return spi_config;
}

static void bspBoardSPIInit()
{   
    bspSPIConfig_t spi_config = {0};

    // spi_config = bspBoardSetSPIConfig(&SPI_IMU, BSP_SPI_WORK_MODE_DMA, "SPI_IMU");
    // bspBoardSPIInstancePtrArray[BSP_SPI_IMU] = bspSPIInit(&spi_config);
}

bspSPIInstance_t *bspBoardGetSPIInstance(bspSPIId_e spi_id)
{
    if (spi_id >= BSP_SPI_MAX) {
        return NULL;
    }

    return bspBoardSPIInstancePtrArray[spi_id];
}

//
// CAN
//
// static bspCANConfig_t bspBoardSetCANConfig(CAN_HandleTypeDef *can_handle,
//                                             const char *can_name)
// {
//     bspCANConfig_t can_config;
//     can_config.can_handle_ = can_handle;
//     can_config.name_ = can_name;

//     return can_config;
// }

static void bspBoardCANInit()
{
    // bspCANConfig_t can_config = {0};

    // can_config = bspBoardSetCANConfig(&hcan1, "CAN1");
    // bspBoardCANInstancePtrArray[BSP_CAN_1] = bspCANInit(&can_config);
}

// bspCANInstance_t *bspBoardGetCANInstance(bspCANId_e can_id)
// {
//     if (can_id >= BSP_CAN_MAX) {
//         return NULL;
//     }

//     return bspBoardCANInstancePtrArray[can_id];
// }

//
// FDCAN
//
static bspFDCANConfig_t bspBoardSetFDCANConfig(FDCAN_HandleTypeDef *fdcan_handle,
                                                const char *can_name)
{
    bspFDCANConfig_t fdcan_config;
    fdcan_config.fdcan_handle_ = fdcan_handle;
    fdcan_config.name_ = can_name;

    return fdcan_config;
}

static void bspBoardFDCANInit()
{
    bspFDCANConfig_t fdcan_config = {0};

    fdcan_config = bspBoardSetFDCANConfig(&hfdcan1, "FDCAN1");
    bspBoardFDCANInstancePtrArray[BSP_FDCAN_1] = bspFDCANInit(&fdcan_config);
}

bspFDCANInstance_t *bspBoardGetFDCANInstance(bspFDCANId_e fdcan_id)
{
    if (fdcan_id >= BSP_FDCAN_MAX) {
        return NULL;
    }

    return bspBoardFDCANInstancePtrArray[fdcan_id];
}

//
// I2C
//
static bspI2CConfig_t bspBoardSetI2CConfig(I2C_HandleTypeDef *i2c_handle,
                                            bspI2CWorkMode_e i2c_work_mode,
                                            const char *i2c_name)
{
    bspI2CConfig_t i2c_config;
    i2c_config.i2c_handle_ = i2c_handle;
    i2c_config.i2c_work_mode_ = i2c_work_mode;
    i2c_config.name_ = i2c_name;

    return i2c_config;
}

static void bspBoardI2CInit()
{
    bspI2CConfig_t i2c_config = {0};

    // i2c_config = bspBoardSetI2CConfig(&I2C_MAG, BSP_I2C_WORK_MODE_BLOCKING, "I2C_MAG");
    // bspBoardI2CInstancePtrArray[BSP_I2C_MAG] = bspI2CInit(&i2c_config);
}

bspI2CInstance_t *bspBoardGetI2CInstance(bspI2CId_e i2c_id)
{
    if (i2c_id >= BSP_I2C_MAX) {
        return NULL;
    }

    return bspBoardI2CInstancePtrArray[i2c_id];
}

//
// BOARD
//
void bspBoardInit()
{
    bspDWTInit();

    bspBoardGPIOInit();
    bspBoardUARTInit();
    bspBoardSPIInit();
    bspBoardCANInit();
    bspBoardI2CInit();
}

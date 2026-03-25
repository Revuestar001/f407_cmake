#include <stdint.h>
#include <string.h>

#include "bsp_board.h"
#include "bsp_def.h"
#include "bsp_gpio_private.h"
#include "bsp_uart_private.h"

// 储存所有GPIO实例的指针
static bspGPIOInstance_t *bspBoardGPIOInstancePtrArray[BSP_GPIO_MAX] = {NULL};
// 储存所有UART实例的指针
static bspUARTInstance_t *bspBoardUARTInstancePtrArray[BSP_UART_MAX] = {NULL};
static uint8_t bspUARTRxBuffer[BSP_UART_MAX][BSP_UART_RX_BUFFER_SIZE] = {0};

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
    bspGPIOConfig_t gpio_config = {0};

    gpio_config = bspBoardSetGPIOConfig(userLEDBule_GPIO_Port, userLEDBule_Pin, BSP_GPIO_ACTIVE_HIGH, "USER_LED_BLUE");
    bspBoardGPIOInstancePtrArray[BSP_GPIO_USER_LED_BLUE] = bspGPIOInit(&gpio_config);
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

    uart_config = bspBoardSetUARTConfig(&uart_print, 
                                        BSP_UART_RX_MODE_DMA_IDLE, 
                                        BSP_UART_TX_MODE_IT, 
                                        &bspUARTRxBuffer[BSP_UART_PRINT][0],
                                        BSP_UART_RX_BUFFER_SIZE, 
                                        "PRINT");
    bspBoardUARTInstancePtrArray[BSP_UART_PRINT] = bspUARTInit(&uart_config);
}

bspUARTInstance_t *bspBoardGetUARTInstance(bspUARTId_e uart_id)
{
    if (uart_id >= BSP_UART_MAX) {
        return NULL;
    }

    return bspBoardUARTInstancePtrArray[uart_id];
}

//
// BOARD
//
void bspBoardInit()
{
    bspBoardGPIOInit();
    bspBoardUARTInit();
}

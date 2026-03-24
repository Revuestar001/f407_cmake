#include <stdint.h>
#include <string.h>

#include "bsp_board.h"
#include "bsp_def.h"
#include "bsp_gpio_private.h"

// 储存所有GPIO实例的指针
static bspGPIOInstance_t *bspBoardGPIOInstancePtrArray[BSP_GPIO_MAX] = {NULL};

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

void bspBoardInit()
{
    bspBoardGPIOInit();
}

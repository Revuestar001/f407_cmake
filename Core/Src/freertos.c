/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_board.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMI088_SPI_READ_MASK            0x80U
#define BMI088_SPI_WRITE_MASK           0x7FU

#define BMI088_ACCEL_CHIP_ID_REG        0x00U
#define BMI088_ACCEL_CHIP_ID            0x1EU
#define BMI088_ACCEL_PWR_CONF_REG       0x7CU
#define BMI088_ACCEL_PWR_CTRL_REG       0x7DU
#define BMI088_ACCEL_PWR_CONF_ACTIVE    0x00U
#define BMI088_ACCEL_PWR_CTRL_ENABLE    0x04U

#define BMI088_GYRO_CHIP_ID_REG         0x00U
#define BMI088_GYRO_CHIP_ID             0x0FU
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static volatile uint8_t bmi088_test_ok_ = 0U;
static volatile uint8_t bmi088_test_error_step_ = 0U;
static volatile bspSPIStatus_e bmi088_last_spi_status_ = BSP_SPI_OK;
static volatile uint8_t bmi088_acc_chip_id_ = 0U;
static volatile uint8_t bmi088_gyro_chip_id_ = 0U;
static volatile uint8_t bmi088_acc_pwr_conf_readback_ = 0U;
static volatile uint8_t bmi088_acc_pwr_ctrl_readback_ = 0U;
static volatile uint32_t bmi088_test_count_ = 0U;
static volatile uint32_t debug = 0U;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static bspSPIStatus_e BMI088Transfer(bspSPIInstance_t *spi,
                                     bspGPIOInstance_t *cs,
                                     uint8_t *tx_buffer,
                                     uint8_t *rx_buffer,
                                     uint16_t data_size);
static bspSPIStatus_e BMI088AccelReadReg(bspSPIInstance_t *spi,
                                         bspGPIOInstance_t *acc_cs,
                                         uint8_t reg_addr,
                                         uint8_t *reg_data);
static bspSPIStatus_e BMI088AccelWriteReg(bspSPIInstance_t *spi,
                                          bspGPIOInstance_t *acc_cs,
                                          uint8_t reg_addr,
                                          uint8_t reg_data);
static bspSPIStatus_e BMI088GyroReadReg(bspSPIInstance_t *spi,
                                        bspGPIOInstance_t *gyro_cs,
                                        uint8_t reg_addr,
                                        uint8_t *reg_data);
static bspSPIStatus_e BMI088RunSmokeTest(bspSPIInstance_t *spi,
                                         bspGPIOInstance_t *acc_cs,
                                         bspGPIOInstance_t *gyro_cs);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  bspSPIInstance_t *spi = bspBoardGetSPIInstance(BSP_SPI_IMU);
  bspGPIOInstance_t *led = bspBoardGetGPIOInstance(BSP_GPIO_USER_LED_BLUE);
  bspGPIOInstance_t *acc_cs = bspBoardGetGPIOInstance(BSP_GPIO_IMU_CS1_ACCEL);
  bspGPIOInstance_t *gyro_cs = bspBoardGetGPIOInstance(BSP_GPIO_IMU_CS1_GYRO);
  uint32_t led_toggle_tick = 0U;

  (void)argument;

  if (spi == NULL || led == NULL || acc_cs == NULL || gyro_cs == NULL) {
    for (;;) {
      osDelay(1000);
    }
  }

  bspGPIOWriteLogic(led, false);

  bmi088_test_ok_ = 0U;
  bmi088_test_error_step_ = 0U;
  bmi088_last_spi_status_ = BMI088RunSmokeTest(spi, acc_cs, gyro_cs);

  if (bmi088_last_spi_status_ == BSP_SPI_OK) {
    bmi088_test_ok_ = 1U;
  }

  for (;;) {
    debug++;
    if (bmi088_test_ok_ != 0U) {
      if (osKernelGetTickCount() >= led_toggle_tick) {
        bspGPIOToggle(led);
        led_toggle_tick = osKernelGetTickCount() + 200U;
      }
    } else {
      bspGPIOWriteLogic(led, true);
    }

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static bspSPIStatus_e BMI088Transfer(bspSPIInstance_t *spi,
                                     bspGPIOInstance_t *cs,
                                     uint8_t *tx_buffer,
                                     uint8_t *rx_buffer,
                                     uint16_t data_size)
{
  bspSPIStatus_e status;

  if (spi == NULL || cs == NULL || tx_buffer == NULL || rx_buffer == NULL || data_size == 0U) {
    return BSP_SPI_ERROR;
  }

  bspGPIOWriteLogic(cs, true);
  status = bspSPITransmitReceive(spi, tx_buffer, rx_buffer, data_size);
  bspGPIOWriteLogic(cs, false);

  return status;
}

static bspSPIStatus_e BMI088AccelReadReg(bspSPIInstance_t *spi,
                                         bspGPIOInstance_t *acc_cs,
                                         uint8_t reg_addr,
                                         uint8_t *reg_data)
{
  uint8_t tx_buffer[3] = {(uint8_t)(reg_addr | BMI088_SPI_READ_MASK), 0x00U, 0x00U};
  uint8_t rx_buffer[3] = {0};
  bspSPIStatus_e status;

  if (reg_data == NULL) {
    return BSP_SPI_ERROR;
  }

  status = BMI088Transfer(spi, acc_cs, tx_buffer, rx_buffer, 3U);
  if (status == BSP_SPI_OK) {
    *reg_data = rx_buffer[2];
  }

  return status;
}

static bspSPIStatus_e BMI088AccelWriteReg(bspSPIInstance_t *spi,
                                          bspGPIOInstance_t *acc_cs,
                                          uint8_t reg_addr,
                                          uint8_t reg_data)
{
  uint8_t tx_buffer[2] = {(uint8_t)(reg_addr & BMI088_SPI_WRITE_MASK), reg_data};
  uint8_t rx_buffer[2] = {0};

  return BMI088Transfer(spi, acc_cs, tx_buffer, rx_buffer, 2U);
}

static bspSPIStatus_e BMI088GyroReadReg(bspSPIInstance_t *spi,
                                        bspGPIOInstance_t *gyro_cs,
                                        uint8_t reg_addr,
                                        uint8_t *reg_data)
{
  uint8_t tx_buffer[2] = {(uint8_t)(reg_addr | BMI088_SPI_READ_MASK), 0x00U};
  uint8_t rx_buffer[2] = {0};
  bspSPIStatus_e status;

  if (reg_data == NULL) {
    return BSP_SPI_ERROR;
  }

  status = BMI088Transfer(spi, gyro_cs, tx_buffer, rx_buffer, 2U);
  if (status == BSP_SPI_OK) {
    *reg_data = rx_buffer[1];
  }

  return status;
}

static bspSPIStatus_e BMI088RunSmokeTest(bspSPIInstance_t *spi,
                                         bspGPIOInstance_t *acc_cs,
                                         bspGPIOInstance_t *gyro_cs)
{
  bspSPIStatus_e status;
  uint8_t dummy_data = 0U;

  bmi088_test_count_++;

  /* First dummy read switches the accelerometer part into SPI mode.
     The returned value is expected to be invalid. */
  status = BMI088AccelReadReg(spi, acc_cs, BMI088_ACCEL_CHIP_ID_REG, &dummy_data);
  if (status != BSP_SPI_OK) {
    bmi088_test_error_step_ = 1U;
    return status;
  }

  osDelay(2);

  status = BMI088AccelWriteReg(spi, acc_cs, BMI088_ACCEL_PWR_CONF_REG, BMI088_ACCEL_PWR_CONF_ACTIVE);
  if (status != BSP_SPI_OK) {
    bmi088_test_error_step_ = 2U;
    return status;
  }

  osDelay(2);

  status = BMI088AccelWriteReg(spi, acc_cs, BMI088_ACCEL_PWR_CTRL_REG, BMI088_ACCEL_PWR_CTRL_ENABLE);
  if (status != BSP_SPI_OK) {
    bmi088_test_error_step_ = 3U;
    return status;
  }

  osDelay(5);

  status = BMI088AccelReadReg(spi, acc_cs, BMI088_ACCEL_CHIP_ID_REG, (uint8_t *)&bmi088_acc_chip_id_);
  if (status != BSP_SPI_OK) {
    bmi088_test_error_step_ = 4U;
    return status;
  }

  status = BMI088GyroReadReg(spi, gyro_cs, BMI088_GYRO_CHIP_ID_REG, (uint8_t *)&bmi088_gyro_chip_id_);
  if (status != BSP_SPI_OK) {
    bmi088_test_error_step_ = 5U;
    return status;
  }

  status = BMI088AccelReadReg(spi, acc_cs, BMI088_ACCEL_PWR_CONF_REG, (uint8_t *)&bmi088_acc_pwr_conf_readback_);
  if (status != BSP_SPI_OK) {
    bmi088_test_error_step_ = 6U;
    return status;
  }

  status = BMI088AccelReadReg(spi, acc_cs, BMI088_ACCEL_PWR_CTRL_REG, (uint8_t *)&bmi088_acc_pwr_ctrl_readback_);
  if (status != BSP_SPI_OK) {
    bmi088_test_error_step_ = 7U;
    return status;
  }

  if (bmi088_acc_chip_id_ != BMI088_ACCEL_CHIP_ID) {
    bmi088_test_error_step_ = 8U;
    return BSP_SPI_ERROR;
  }

  if (bmi088_gyro_chip_id_ != BMI088_GYRO_CHIP_ID) {
    bmi088_test_error_step_ = 9U;
    return BSP_SPI_ERROR;
  }

  if (bmi088_acc_pwr_conf_readback_ != BMI088_ACCEL_PWR_CONF_ACTIVE) {
    bmi088_test_error_step_ = 10U;
    return BSP_SPI_ERROR;
  }

  if (bmi088_acc_pwr_ctrl_readback_ != BMI088_ACCEL_PWR_CTRL_ENABLE) {
    bmi088_test_error_step_ = 11U;
    return BSP_SPI_ERROR;
  }

  return BSP_SPI_OK;
}
/* USER CODE END Application */


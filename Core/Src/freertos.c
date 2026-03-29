/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for FreeRTOS applications
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
#include "bsp_gpio.h"
#include "portmacro.h"
#include "projdefs.h"
#include "rc_mapper.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_remote_control.h"
#include "bmi088.h"
#include "bsp_board.h"
#include "bsp_dwt.h"
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile deviceBMI088Instance_t *bmi088_test_instance_ = NULL;
volatile deviceBMI088Status_e bmi088_test_init_status_ = DEVICE_BMI088_ERROR;
volatile deviceBMI088Status_e bmi088_test_update_status_ = DEVICE_BMI088_ERROR;
volatile deviceBMI088Status_e bmi088_test_get_status_ = DEVICE_BMI088_ERROR;
volatile uint32_t bmi088_test_update_count_ = 0U;
volatile deviceBMI088Data_t bmi088_test_data_ = {0};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for remoteControlTask */
osThreadId_t remoteControlTaskHandle;
const osThreadAttr_t remoteControlTask_attributes = {
  .name = "remoteControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

volatile UBaseType_t remote_control_task_stack_high_water_mark_ = 0;
volatile UBaseType_t remote_control_task_stack_high_water_mark_min_ = 0xFFFFFFFFU;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartRemoteControlTask(void *argument);

void bmi088GyroITCallback(void *owner)
{
  (void)owner;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
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
  /* creation of remoteControlTask */
  remoteControlTaskHandle = osThreadNew(StartRemoteControlTask, NULL, &remoteControlTask_attributes);

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
  (void)argument;

  deviceBMI088Config_t bmi088_config = {
    .spi_instance_ = bspBoardGetSPIInstance(BSP_SPI_IMU),
    .accel_cs_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_CS1_ACCEL),
    .gyro_cs_ = bspBoardGetGPIOInstance(BSP_GPIO_IMU_CS1_GYRO),
    .mode_ = DEVICE_BMI088_EXIT,
    .delay_us_callback_ = bspDWTDelayUs,
    .name_ = "BMI088_TEST",
  };

  bmi088_test_instance_ = deviceBMI088InstanceInit(&bmi088_config);
  if (bmi088_test_instance_ == NULL) {
    for (;;) {
      osDelay(1000);
    }
  }

  bspGPIOInstance_t *int3 = bspBoardGetGPIOInstance(BSP_GPIO_IMU_INT1_GYRO);
  bspGPIOIsrCallbackRegister(int3, (void *)NULL, bmi088GyroITCallback);

  bmi088_test_init_status_ = deviceBMI088Init((deviceBMI088Instance_t *)bmi088_test_instance_);
  bmi088_test_init_status_ &= deviceBMI088ConfigGyroDataReadyIT((deviceBMI088Instance_t *)bmi088_test_instance_);

  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (bmi088_test_init_status_ == DEVICE_BMI088_OK) {
      deviceBMI088Data_t data = {0};

      bmi088_test_update_status_ = deviceBMI088UpdateData((deviceBMI088Instance_t *)bmi088_test_instance_);
      if (bmi088_test_update_status_ == DEVICE_BMI088_OK) {
        bmi088_test_get_status_ = deviceBMI088GetData((const deviceBMI088Instance_t *)bmi088_test_instance_, &data);
        if (bmi088_test_get_status_ == DEVICE_BMI088_OK) {
          bmi088_test_data_ = data;
          bmi088_test_update_count_ ++;
        }
      }
    }

    // osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRemoteControlTask */
/**
  * @brief  Function implementing the remoteControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRemoteControlTask */
void StartRemoteControlTask(void *argument)
{
  /* USER CODE BEGIN StartRemoteControlTask */
  (void)argument;

  appRemoteControlInit();
  remote_control_task_stack_high_water_mark_ = uxTaskGetStackHighWaterMark(NULL);

  for (;;) {
    // 无限阻塞不合理，超时10ms认为lost
    (void)appRemoteControlUpdate(pdMS_TO_TICKS(10));

    {
      UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);
      if (stack_high_water_mark < remote_control_task_stack_high_water_mark_min_) {
        remote_control_task_stack_high_water_mark_min_ = stack_high_water_mark;
      }
    }
  }
  /* USER CODE END StartRemoteControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

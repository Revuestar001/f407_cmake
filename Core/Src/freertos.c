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
#include "rc_mapper.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_remote_control.h"
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

int16_t stick_left_x = 0;
int16_t stick_left_y = 0;
int16_t stick_right_x = 0;
int16_t stick_right_y = 0;

moduleRCSwitch_t switch_left = {0};
moduleRCSwitch_t switch_right = {0};
volatile UBaseType_t remote_control_task_stack_high_water_mark_ = 0;
volatile UBaseType_t remote_control_task_stack_high_water_mark_min_ = 0xFFFFFFFFU;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartRemoteControlTask(void *argument);
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

  for (;;) {
    osDelay(1000);
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

  taskRemoteControlInit();
  remote_control_task_stack_high_water_mark_ = uxTaskGetStackHighWaterMark(NULL);

  for (;;) {
    bool update_success = taskRemoteControlUpdate(portMAX_DELAY);
    if (update_success == true) {
      const moduleRCMapper_t *rc = taskRemoteControlGetRCMapped();

      stick_left_x = rc->stick_left_x_;
      stick_left_y = rc->stick_left_y_;
      stick_right_x = rc->stick_right_x_;
      stick_right_y = rc->stick_right_y_;

      switch_left = rc->switch_left_;
      switch_right = rc->switch_right_;
    }
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


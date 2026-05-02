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
#include "cmsis_os2.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_remote_control.h"
#include "app_ins.h"
#include "app_chassis.h"
#include "app_topic_bus_test.h"
#include "user_def.h"
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
/* Definitions for remoteControlTask */
osThreadId_t remoteControlTaskHandle;
const osThreadAttr_t remoteControlTask_attributes = {
  .name = "remoteControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
// volatile UBaseType_t remote_control_task_stack_high_water_mark_ = 0;
// volatile UBaseType_t remote_control_task_stack_high_water_mark_min_ = 0xFFFFFFFFU;

osThreadId_t appINSTaskHandle;
const osThreadAttr_t insTask_attributes = {
  .name = "insTask_",
  .stack_size = 256 * 16,
  .priority = (osPriority_t) osPriorityHigh,
};
volatile UBaseType_t ins_task_stack_high_water_mark_ = 0;
volatile UBaseType_t ins_task_stack_high_water_mark_min_ = 0xFFFFFFFFU;

osThreadId_t appChassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask_",
  .stack_size = 256 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

#if USER_TOPIC_BUS_TEST_ENABLE
osThreadId_t topicBusTestPublisherTaskHandle;
const osThreadAttr_t topicBusTestPublisherTask_attributes = {
  .name = "topicBusPub_",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t topicBusTestCmdFastTaskHandle;
const osThreadAttr_t topicBusTestCmdFastTask_attributes = {
  .name = "topicBusCmdF",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t topicBusTestCmdSlowTaskHandle;
const osThreadAttr_t topicBusTestCmdSlowTask_attributes = {
  .name = "topicBusCmdS",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t topicBusTestStateTaskHandle;
const osThreadAttr_t topicBusTestStateTask_attributes = {
  .name = "topicBusSt_",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t topicBusTestStateAuxTaskHandle;
const osThreadAttr_t topicBusTestStateAuxTask_attributes = {
  .name = "topicBusAux",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
#endif

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
void StartRemoteControlTask(void *argument);
void StartINSTask(void *argument);
void StartChassisTask(void *argument);
#if USER_TOPIC_BUS_TEST_ENABLE
void StartTopicBusTestPublisherTask(void *argument);
void StartTopicBusTestCommandFastSubscriberTask(void *argument);
void StartTopicBusTestCommandSlowSubscriberTask(void *argument);
void StartTopicBusTestStateSubscriberTask(void *argument);
void StartTopicBusTestStateAuxSubscriberTask(void *argument);
#endif
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
  remoteControlTaskHandle = osThreadNew(StartRemoteControlTask, NULL, &remoteControlTask_attributes);
  appINSTaskHandle = osThreadNew(StartINSTask, NULL, &insTask_attributes);
  appChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &chassisTask_attributes);
#if USER_TOPIC_BUS_TEST_ENABLE
  topicBusTestPublisherTaskHandle = osThreadNew(StartTopicBusTestPublisherTask, NULL, &topicBusTestPublisherTask_attributes);
  topicBusTestCmdFastTaskHandle = osThreadNew(StartTopicBusTestCommandFastSubscriberTask, NULL, &topicBusTestCmdFastTask_attributes);
  topicBusTestCmdSlowTaskHandle = osThreadNew(StartTopicBusTestCommandSlowSubscriberTask, NULL, &topicBusTestCmdSlowTask_attributes);
  topicBusTestStateTaskHandle = osThreadNew(StartTopicBusTestStateSubscriberTask, NULL, &topicBusTestStateTask_attributes);
  topicBusTestStateAuxTaskHandle = osThreadNew(StartTopicBusTestStateAuxSubscriberTask, NULL, &topicBusTestStateAuxTask_attributes);
#endif
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartRemoteControlTask(void *argument)
{
  appRemoteControlTaskEntry(argument);
}

void StartINSTask(void *argument)
{
  appINSTaskEntry(argument);
}

void StartChassisTask(void *argument)
{
  appChassisTaskEntry(argument);
}

#if USER_TOPIC_BUS_TEST_ENABLE
void StartTopicBusTestPublisherTask(void *argument)
{
  appTopicBusTestPublisherTaskEntry(argument);
}

void StartTopicBusTestCommandFastSubscriberTask(void *argument)
{
  appTopicBusTestCommandFastSubscriberTaskEntry(argument);
}

void StartTopicBusTestCommandSlowSubscriberTask(void *argument)
{
  appTopicBusTestCommandSlowSubscriberTaskEntry(argument);
}

void StartTopicBusTestStateSubscriberTask(void *argument)
{
  appTopicBusTestStateSubscriberTaskEntry(argument);
}

void StartTopicBusTestStateAuxSubscriberTask(void *argument)
{
  appTopicBusTestStateAuxSubscriberTaskEntry(argument);
}
#endif
/* USER CODE END Application */


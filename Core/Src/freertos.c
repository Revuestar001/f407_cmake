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
#include "bsp_uart.h"
#include <string.h>
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
osThreadId_t userLEDTaskHandle;
static volatile uint8_t uart3_rx_ready_ = 0;
static volatile uint16_t uart3_rx_len_ = 0;
static volatile bspUARTRxEventType_e uart3_rx_event_ = BSP_UART_RX_EVENT_INVALID;
static volatile uint32_t uart3_rx_count_ = 0;
static volatile uint8_t uart3_tx_ready_ = 1;
static uint8_t uart3_rx_shadow_[BSP_UART_RX_BUFFER_SIZE] = {0};
static uint8_t uart3_tx_buffer_[BSP_UART_RX_BUFFER_SIZE + 16] = {0};
static uint8_t uart3_boot_msg_[] = "UART3 TX_IT RX_DMA_IDLE test ready.\r\n";
const osThreadAttr_t userLEDTask_attributes = {
  .name = "userLEDTask",
  .stack_size = 128 * 4, // 字节数，不可以小于#define configMINIMAL_STACK_SIZE ((uint16_t)128) * 4
  .priority = (osPriority_t) osPriorityNormal,
};
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
void StartUserLEDTask(void *argument);
static void UART3RxTestCallback(void *owner_ptr, uint8_t *rx_buffer_ptr, uint16_t rx_buffer_cur_index, bspUARTRxEventType_e rx_event);
static void UART3TxTestCallback(void *owner_ptr);
static void UART3ErrorTestCallback(void *owner_ptr);
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
  /* UART3 RX test uses the blue LED as receive indication, so the heartbeat
     LED task is intentionally not created here. */
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
  bspUARTInstance_t *uart = bspBoardGetUARTInstance(BSP_UART_PRINT);
  bspGPIOInstance_t *led = bspBoardGetGPIOInstance(BSP_GPIO_USER_LED_BLUE);
  uint32_t led_off_tick = 0;

  if (uart != NULL) {
    bspUARTRxEventCallbackRegister(uart, NULL, UART3RxTestCallback);
    bspUARTTxCpltCallbackRegister(uart, NULL, UART3TxTestCallback);
    bspUARTErrorCallbackRegister(uart, NULL, UART3ErrorTestCallback);
    bspUARTRxStart(uart);
    uart3_tx_ready_ = 0;
    if (bspUARTTransimt(uart, uart3_boot_msg_, sizeof(uart3_boot_msg_) - 1) != BSP_UART_OK) {
      uart3_tx_ready_ = 1;
    }
  }

  if (led != NULL) {
    bspGPIOWriteLogic(led, false);
  }

  for (;;)
  {
    if (uart3_rx_ready_ != 0 && uart != NULL && uart3_tx_ready_ != 0) {
      uint16_t rx_len = 0;
      uint16_t tx_len = 0;

      taskENTER_CRITICAL();
      rx_len = uart3_rx_len_;
      if (rx_len > BSP_UART_RX_BUFFER_SIZE) {
        rx_len = BSP_UART_RX_BUFFER_SIZE;
      }
      memcpy(&uart3_tx_buffer_[0], "RX: ", 4);
      memcpy(&uart3_tx_buffer_[4], uart3_rx_shadow_, rx_len);
      memcpy(&uart3_tx_buffer_[4 + rx_len], "\r\n", 2);
      tx_len = 4 + rx_len + 2;
      uart3_rx_ready_ = 0;
      taskEXIT_CRITICAL();

      if (led != NULL) {
        bspGPIOWriteLogic(led, true);
        led_off_tick = osKernelGetTickCount() + 100U;
      }

      uart3_tx_ready_ = 0;
      if (bspUARTTransimt(uart, uart3_tx_buffer_, tx_len) != BSP_UART_OK) {
        uart3_tx_ready_ = 1;
      }
    }

    if (led != NULL && led_off_tick != 0U && osKernelGetTickCount() >= led_off_tick) {
      bspGPIOWriteLogic(led, false);
      led_off_tick = 0U;
    }

    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartUserLEDTask(void *argument)
{
  for (;;) {
    osDelay(1000);
  } 
}

static void UART3RxTestCallback(void *owner_ptr, uint8_t *rx_buffer_ptr, uint16_t rx_buffer_cur_index, bspUARTRxEventType_e rx_event)
{
  (void)owner_ptr;

  if (rx_buffer_ptr == NULL) {
    return;
  }

  if (rx_buffer_cur_index > BSP_UART_RX_BUFFER_SIZE) {
    rx_buffer_cur_index = BSP_UART_RX_BUFFER_SIZE;
  }

  memcpy(uart3_rx_shadow_, rx_buffer_ptr, rx_buffer_cur_index);
  uart3_rx_len_ = rx_buffer_cur_index;
  uart3_rx_event_ = rx_event;
  uart3_rx_count_++;
  uart3_rx_ready_ = 1;
}

static void UART3TxTestCallback(void *owner_ptr)
{
  (void)owner_ptr;
  uart3_tx_ready_ = 1;
}

static void UART3ErrorTestCallback(void *owner_ptr)
{
  (void)owner_ptr;
  uart3_tx_ready_ = 1;
}
/* USER CODE END Application */


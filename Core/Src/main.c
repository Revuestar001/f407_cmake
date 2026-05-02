/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_board.h"
#include "bsp_can.h"
#include "rmd_v2_x6.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if USER_RMD_V2_X6_CAN_ID_CONFIG_ENABLE
#define MAIN_RMD_V2_X6_CAN_ID_CONFIG_TX_ID 0x300U
#define MAIN_RMD_V2_X6_CAN_ID_CONFIG_DLC 8U
#define MAIN_RMD_V2_X6_CAN_ID_CONFIG_WRITE_FLAG 0U

static volatile bool main_rmd_v2_x6_can_id_config_reply_ok_ = false;
static uint8_t main_rmd_v2_x6_can_id_config_expected_reply_[MAIN_RMD_V2_X6_CAN_ID_CONFIG_DLC] = {0U};

static void mainRMDV2X6CANIDConfigReplyCallback(void *owner, const bspCANMessage_t *rx_message)
{
  if (owner == NULL || rx_message == NULL) {
    return;
  }

  if (rx_message->message_header_.message_id_ != MAIN_RMD_V2_X6_CAN_ID_CONFIG_TX_ID ||
      rx_message->message_header_.message_ide_ != 0U ||
      rx_message->message_header_.message_dlc_ != MAIN_RMD_V2_X6_CAN_ID_CONFIG_DLC) {
    return;
  }

  for (uint32_t i = 0; i < MAIN_RMD_V2_X6_CAN_ID_CONFIG_DLC; i++) {
    if (rx_message->message_data_[i] != main_rmd_v2_x6_can_id_config_expected_reply_[i]) {
      return;
    }
  }

  *((volatile bool *)owner) = true;
}

static void mainRunRMDV2X6CANIDConfigMode(void)
{
  bspCANInstance_t *can_instance = bspBoardGetCANInstance(BSP_CAN_1);
  if (can_instance == NULL) {
    Error_Handler();
  }

  for (uint32_t i = 0; i < MAIN_RMD_V2_X6_CAN_ID_CONFIG_DLC; i++) {
    main_rmd_v2_x6_can_id_config_expected_reply_[i] = 0U;
  }
  main_rmd_v2_x6_can_id_config_expected_reply_[0] = 0x79U;
  main_rmd_v2_x6_can_id_config_expected_reply_[2] = MAIN_RMD_V2_X6_CAN_ID_CONFIG_WRITE_FLAG;
  main_rmd_v2_x6_can_id_config_expected_reply_[7] = (uint8_t)USER_RMD_V2_X6_CAN_ID_CONFIG_TARGET_CAN_ID;

  bspCANRxRoute_t rx_route = {0};
  rx_route.route_id_ = MAIN_RMD_V2_X6_CAN_ID_CONFIG_TX_ID;
  rx_route.route_ide_ = 0U;
  rx_route.route_id_mask_ = 0U;
  rx_route.route_owner_ = (void *)&main_rmd_v2_x6_can_id_config_reply_ok_;
  rx_route.route_rx_callback_ = mainRMDV2X6CANIDConfigReplyCallback;
  bspCANRxCallbackRegister(can_instance, rx_route);

  if (bspCANSetFilter(can_instance) != BSP_CAN_OK) {
    Error_Handler();
  }

  if (bspCANStart(can_instance) != BSP_CAN_OK) {
    Error_Handler();
  }

  while (1) {
    main_rmd_v2_x6_can_id_config_reply_ok_ = false;

    if (motorRMDV2X6SetSingleMotorCANID(can_instance, USER_RMD_V2_X6_CAN_ID_CONFIG_TARGET_CAN_ID) == MOTOR_OK) {
      uint32_t wait_start_ms = HAL_GetTick();
      while ((HAL_GetTick() - wait_start_ms) < USER_RMD_V2_X6_CAN_ID_CONFIG_REPLY_TIMEOUT_MS) {
        if (main_rmd_v2_x6_can_id_config_reply_ok_ == true) {
          while (1) {
            HAL_Delay(1000);
          }
        }
      }
    }

    HAL_Delay(USER_RMD_V2_X6_CAN_ID_CONFIG_SEND_PERIOD_MS);
  }
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  bspBoardInit();
#if USER_RMD_V2_X6_CAN_ID_CONFIG_ENABLE
  mainRunRMDV2X6CANIDConfigMode();
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 7;
  RCC_OscInitStruct.PLL.PLLN = 196;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Tracked.h"
#include <stdio.h>
#include <string.h>
#include "oled.h"
#include "IMU406.h"
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
char oled_buf[32];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void OLED_Show_Speed(void);
void OLED_Show_Attitude(void);
void OLED_Show_IMU(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化 OLED */
  OLED_Init();
  OLED_Clear();

  /* 初始化履带底盘（内含 ICM20948_Init） */
  Tracked_Init();


  /* ★ 静止 2 秒做 IMU 零偏校准（400 次 × 5ms = 2000ms） */
  /* IMU 静止校准 */
  OLED_ShowString(0, 0, (unsigned char *)"Calibrating IMU");
  OLED_ShowString(0, 2, (unsigned char *)"Keep STILL...");
  HAL_Delay(2000);
  OLED_Clear();
  OLED_ShowString(0, 0, (unsigned char *)"Calib OK!");
  HAL_Delay(500);
  OLED_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OLED_Clear();
    Tracked_SetTargetSpeed(0.0f, 1.0f);
    OLED_Show_Attitude();
    OLED_Show_Speed();
    HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void OLED_Show_Speed(void)
{
  sprintf(oled_buf, "L:%d R:%d", (int)g_tracked.delta_left_cnt,
                                (int)g_tracked.delta_right_cnt);
  OLED_ShowString(0, 0, (unsigned char *)oled_buf);
}

void OLED_Show_Attitude(void)
{
  int32_t yaw   = IMU406_Get_Yaw();
  int32_t roll  = IMU406_Get_Roll();
  int32_t pitch = IMU406_Get_Pitch();

  /* 整数部分 */
  int yaw_i   = (int)(yaw   / 100);   /* 假设单位是 0.01°，可按需调整 */
  int roll_i  = (int)(roll  / 100);
  int pitch_i = (int)(pitch / 100);

  /* 小数部分（1 位） */
  int yaw_f   = (int)(yaw   % 100);
  int roll_f  = (int)(roll  % 100);
  int pitch_f = (int)(pitch % 100);

  /* 处理负数的小数部分 */
  if (yaw_f   < 0) yaw_f   = -yaw_f;
  if (roll_f  < 0) roll_f  = -roll_f;
  if (pitch_f < 0) pitch_f = -pitch_f;

  /* 显示 yaw */
  sprintf(oled_buf, "yaw:%d.%d  ", yaw_i, yaw_f / 10);
  OLED_ShowString(0, 0, (unsigned char *)oled_buf);

  /* 显示 roll */
  sprintf(oled_buf, "rol:%d.%d  ", roll_i, roll_f / 10);
  OLED_ShowString(0, 2, (unsigned char *)oled_buf);

  /* 显示 pitch */
  sprintf(oled_buf, "pit:%d.%d  ", pitch_i, pitch_f / 10);
  OLED_ShowString(0, 4, (unsigned char *)oled_buf);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

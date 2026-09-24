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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "Tracked.h"
#include <stdio.h>
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
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  OLED_Init();
  OLED_NewFrame();
  OLED_PrintString(0, 0, "Tracked Init...", &font16x16, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
  HAL_Delay(300);

  /* ---- 底盘初始化 ---- */
  Tracked_Init();

  /* ---- 设置目标速度：前进 0.2 m/s ---- */
  Tracked_SetTargetSpeed(0.05f, 0.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* ============================================================ */
    /*  1. 设置目标速度（持续刷新，防止超时清零）                     */
    /* ============================================================ */
    Tracked_SetTargetSpeed(0.05f, 0.0f);   /* 前进 0.05 m/s，不转向 */

    /* ============================================================ */
    /*  2. OLED 显示实时状态                                         */
    /* ============================================================ */
    {
      char buf[24];

      OLED_NewFrame();

      /* 第 0 行：左右轮编码器原始计数 */
      snprintf(buf, sizeof(buf), "L:%d", (int)g_tracked.delta_left_cnt);
      OLED_PrintString(0, 0, buf, &font16x16, OLED_COLOR_NORMAL);

      snprintf(buf, sizeof(buf), "R:%d", (int)g_tracked.delta_right_cnt);
      OLED_PrintString(64, 0, buf, &font16x16, OLED_COLOR_NORMAL);

      /* 第 1 行：左右轮实时线速度（由编码器算得） */
      snprintf(buf, sizeof(buf), "vl:%.3f", g_tracked.current_vl);
      OLED_PrintString(0, 16, buf, &font16x16, OLED_COLOR_NORMAL);

      snprintf(buf, sizeof(buf), "vr:%.3f", g_tracked.current_vr);
      OLED_PrintString(64, 16, buf, &font16x16, OLED_COLOR_NORMAL);

      /* 第 2 行：目标速度 */
      snprintf(buf, sizeof(buf), "tvl:%.3f", g_tracked.target_vl);
      OLED_PrintString(0, 32, buf, &font16x16, OLED_COLOR_NORMAL);

      snprintf(buf, sizeof(buf), "tvr:%.3f", g_tracked.target_vr);
      OLED_PrintString(64, 32, buf, &font16x16, OLED_COLOR_NORMAL);

      /* 第 3 行：PID 输出 */
      snprintf(buf, sizeof(buf), "ol:%.0f", g_tracked.pid_left.output);
      OLED_PrintString(0, 48, buf, &font16x16, OLED_COLOR_NORMAL);

      snprintf(buf, sizeof(buf), "or:%.0f", g_tracked.pid_right.output);
      OLED_PrintString(64, 48, buf, &font16x16, OLED_COLOR_NORMAL);

      OLED_ShowFrame();
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

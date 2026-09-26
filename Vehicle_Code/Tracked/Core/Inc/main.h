/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LEFT_A_Pin GPIO_PIN_0
#define LEFT_A_GPIO_Port GPIOA
#define LEFT_B_Pin GPIO_PIN_1
#define LEFT_B_GPIO_Port GPIOA
#define IMU406_TX_Pin GPIO_PIN_2
#define IMU406_TX_GPIO_Port GPIOA
#define IMU406_RX_Pin GPIO_PIN_3
#define IMU406_RX_GPIO_Port GPIOA
#define RIGHT_A_Pin GPIO_PIN_6
#define RIGHT_A_GPIO_Port GPIOA
#define RIGHT_B_Pin GPIO_PIN_7
#define RIGHT_B_GPIO_Port GPIOA
#define LEFT_1_Pin GPIO_PIN_12
#define LEFT_1_GPIO_Port GPIOB
#define LEFT_2_Pin GPIO_PIN_13
#define LEFT_2_GPIO_Port GPIOB
#define RIGHT_1_Pin GPIO_PIN_14
#define RIGHT_1_GPIO_Port GPIOB
#define RIGHT_2_Pin GPIO_PIN_15
#define RIGHT_2_GPIO_Port GPIOB
#define ROS_TX_Pin GPIO_PIN_9
#define ROS_TX_GPIO_Port GPIOA
#define ROS_RX_Pin GPIO_PIN_10
#define ROS_RX_GPIO_Port GPIOA
#define PWM_LEFT_Pin GPIO_PIN_6
#define PWM_LEFT_GPIO_Port GPIOB
#define PWM_RIGHT_Pin GPIO_PIN_7
#define PWM_RIGHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

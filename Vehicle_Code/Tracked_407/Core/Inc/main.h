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
#include "stm32f4xx_hal.h"

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
#define Motor_B_IN1_Pin GPIO_PIN_5
#define Motor_B_IN1_GPIO_Port GPIOE
#define Motor_B_IN2_Pin GPIO_PIN_6
#define Motor_B_IN2_GPIO_Port GPIOE
#define Motor_C_Encoder_A_Pin GPIO_PIN_0
#define Motor_C_Encoder_A_GPIO_Port GPIOA
#define Motor_C_Encoder_B_Pin GPIO_PIN_1
#define Motor_C_Encoder_B_GPIO_Port GPIOA
#define IMU406_TX_Pin GPIO_PIN_2
#define IMU406_TX_GPIO_Port GPIOA
#define IMU406_RX_Pin GPIO_PIN_3
#define IMU406_RX_GPIO_Port GPIOA
#define HEAD_LED_Pin GPIO_PIN_5
#define HEAD_LED_GPIO_Port GPIOC
#define BLUE_LED_Pin GPIO_PIN_8
#define BLUE_LED_GPIO_Port GPIOE
#define Motor_C_IN2_Pin GPIO_PIN_9
#define Motor_C_IN2_GPIO_Port GPIOE
#define Motor_C_IN1_Pin GPIO_PIN_11
#define Motor_C_IN1_GPIO_Port GPIOE
#define Motor_D_IN2_Pin GPIO_PIN_13
#define Motor_D_IN2_GPIO_Port GPIOE
#define Motor_D_IN1_Pin GPIO_PIN_14
#define Motor_D_IN1_GPIO_Port GPIOE
#define USART3_ROS_TX_Pin GPIO_PIN_8
#define USART3_ROS_TX_GPIO_Port GPIOD
#define USART3_ROS_RX_Pin GPIO_PIN_9
#define USART3_ROS_RX_GPIO_Port GPIOD
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOD
#define OLED_RES_Pin GPIO_PIN_12
#define OLED_RES_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOD
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOA
#define Motor_A_Encoder_A_Pin GPIO_PIN_15
#define Motor_A_Encoder_A_GPIO_Port GPIOA
#define Motor_A_Encoder_B_Pin GPIO_PIN_3
#define Motor_A_Encoder_B_GPIO_Port GPIOB
#define Motor_B_Encoder_A_Pin GPIO_PIN_4
#define Motor_B_Encoder_A_GPIO_Port GPIOB
#define Motor_B_Encoder_B_Pin GPIO_PIN_5
#define Motor_B_Encoder_B_GPIO_Port GPIOB
#define Motor_D_Encoder_A_Pin GPIO_PIN_6
#define Motor_D_Encoder_A_GPIO_Port GPIOB
#define Motor_D_Encoder_B_Pin GPIO_PIN_7
#define Motor_D_Encoder_B_GPIO_Port GPIOB
#define Motor_A_IN2_Pin GPIO_PIN_8
#define Motor_A_IN2_GPIO_Port GPIOB
#define Motor_A_IN1_Pin GPIO_PIN_9
#define Motor_A_IN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal_tim.h"
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
#define MOTOR1_OT1_Pin GPIO_PIN_6
#define MOTOR1_OT1_GPIO_Port GPIOA
#define MOTOR1_OT2_Pin GPIO_PIN_7
#define MOTOR1_OT2_GPIO_Port GPIOA
#define PWM_SLAVE_Pin GPIO_PIN_0
#define PWM_SLAVE_GPIO_Port GPIOB
#define SR04_OT3_Pin GPIO_PIN_2
#define SR04_OT3_GPIO_Port GPIOB
#define SR04_IN3_Pin GPIO_PIN_10
#define SR04_IN3_GPIO_Port GPIOB
#define SR04_IN1_Pin GPIO_PIN_12
#define SR04_IN1_GPIO_Port GPIOB
#define SR04_OT1_Pin GPIO_PIN_13
#define SR04_OT1_GPIO_Port GPIOB
#define MOTOR2_OT1_Pin GPIO_PIN_14
#define MOTOR2_OT1_GPIO_Port GPIOB
#define MOTOR2_OT2_Pin GPIO_PIN_15
#define MOTOR2_OT2_GPIO_Port GPIOB
#define PWM_MOTOR1_Pin GPIO_PIN_8
#define PWM_MOTOR1_GPIO_Port GPIOA
#define PWM_MOTOR2_Pin GPIO_PIN_9
#define PWM_MOTOR2_GPIO_Port GPIOA
#define SR04_OT2_Pin GPIO_PIN_11
#define SR04_OT2_GPIO_Port GPIOA
#define SR04_IN2_Pin GPIO_PIN_12
#define SR04_IN2_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_6
#define BT_TX_GPIO_Port GPIOB
#define BT_RX_Pin GPIO_PIN_7
#define BT_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PWM_MOTOR_HANDLER &htim3
#define TIM_HANDLE      &htim1
#define TIM_CH_LEFT     TIM_CHANNEL_1
#define TIM_CH_RIGHT    TIM_CHANNEL_2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

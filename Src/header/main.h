/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LIGHT_SENSOR1_Pin GPIO_PIN_0
#define LIGHT_SENSOR1_GPIO_Port GPIOC
#define LIGHT_SENSOR2_Pin GPIO_PIN_1
#define LIGHT_SENSOR2_GPIO_Port GPIOC
#define RIGHT_LIGHT_Pin GPIO_PIN_3
#define RIGHT_LIGHT_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_4
#define IN3_GPIO_Port GPIOA
#define LEFT_LIGHT_Pin GPIO_PIN_4
#define LEFT_LIGHT_GPIO_Port GPIOC
#define IN4_Pin GPIO_PIN_0
#define IN4_GPIO_Port GPIOB
#define TRIG2_Pin GPIO_PIN_10
#define TRIG2_GPIO_Port GPIOB
#define TRIG_Pin GPIO_PIN_8
#define TRIG_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define EXTRA_OUT1_Pin GPIO_PIN_10
#define EXTRA_OUT1_GPIO_Port GPIOC
#define EXTRA_OUT2_Pin GPIO_PIN_11
#define EXTRA_OUT2_GPIO_Port GPIOC
#define BRAKE_LIGHT_Pin GPIO_PIN_12
#define BRAKE_LIGHT_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TRIG3_Pin GPIO_PIN_4
#define TRIG3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

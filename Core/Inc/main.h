/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define DIR_2_Pin GPIO_PIN_15
#define DIR_2_GPIO_Port GPIOC
#define M1_A_Pin GPIO_PIN_4
#define M1_A_GPIO_Port GPIOA
#define M1_A_EXTI_IRQn EXTI4_IRQn
#define M1_B_Pin GPIO_PIN_5
#define M1_B_GPIO_Port GPIOA
#define M1_B_EXTI_IRQn EXTI9_5_IRQn
#define M2_A_Pin GPIO_PIN_6
#define M2_A_GPIO_Port GPIOA
#define M2_A_EXTI_IRQn EXTI9_5_IRQn
#define M2_B_Pin GPIO_PIN_7
#define M2_B_GPIO_Port GPIOA
#define M2_B_EXTI_IRQn EXTI9_5_IRQn
#define L_2_Pin GPIO_PIN_12
#define L_2_GPIO_Port GPIOB
#define L_2_EXTI_IRQn EXTI15_10_IRQn
#define L_3_Pin GPIO_PIN_13
#define L_3_GPIO_Port GPIOB
#define L_3_EXTI_IRQn EXTI15_10_IRQn
#define L_4_Pin GPIO_PIN_14
#define L_4_GPIO_Port GPIOB
#define L_4_EXTI_IRQn EXTI15_10_IRQn
#define L_5_Pin GPIO_PIN_15
#define L_5_GPIO_Port GPIOB
#define L_5_EXTI_IRQn EXTI15_10_IRQn
#define L_1_Pin GPIO_PIN_8
#define L_1_GPIO_Port GPIOA
#define L_1_EXTI_IRQn EXTI9_5_IRQn
#define RST_Pin GPIO_PIN_6
#define RST_GPIO_Port GPIOB
#define NSS_Pin GPIO_PIN_7
#define NSS_GPIO_Port GPIOB
#define DIO0_Pin GPIO_PIN_9
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

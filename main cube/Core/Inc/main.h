/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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

volatile uint32_t servo_pos;
volatile int servo_d;

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MUX_S0_Pin GPIO_PIN_2
#define MUX_S0_GPIO_Port GPIOA
#define VBATT0_Pin GPIO_PIN_3
#define VBATT0_GPIO_Port GPIOA
#define VBATT1_Pin GPIO_PIN_4
#define VBATT1_GPIO_Port GPIOA
#define MUX_EN_Pin GPIO_PIN_5
#define MUX_EN_GPIO_Port GPIOA
#define MUX_S1_Pin GPIO_PIN_6
#define MUX_S1_GPIO_Port GPIOA
#define MUX_S2_Pin GPIO_PIN_7
#define MUX_S2_GPIO_Port GPIOA
#define MTR3_PWM_Pin GPIO_PIN_0
#define MTR3_PWM_GPIO_Port GPIOB
#define MTR4_PWM_Pin GPIO_PIN_1
#define MTR4_PWM_GPIO_Port GPIOB
#define MTR5_PWM_Pin GPIO_PIN_8
#define MTR5_PWM_GPIO_Port GPIOA
#define MTR1_PWM_Pin GPIO_PIN_9
#define MTR1_PWM_GPIO_Port GPIOA
#define MTR2_PWM_Pin GPIO_PIN_10
#define MTR2_PWM_GPIO_Port GPIOA
#define MTR_FB_Pin GPIO_PIN_11
#define MTR_FB_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

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
  extern SPI_HandleTypeDef hspi3;
  extern TIM_HandleTypeDef htim1;
  extern TIM_HandleTypeDef htim2;
  extern TIM_HandleTypeDef htim3;
  extern TIM_HandleTypeDef htim4;
  extern TIM_HandleTypeDef htim5;
  extern TIM_HandleTypeDef htim8;
  extern TIM_HandleTypeDef htim11;
	  extern TIM_HandleTypeDef htim13;
  extern UART_HandleTypeDef huart3;
	  extern UART_HandleTypeDef huart4;
		  extern UART_HandleTypeDef huart5;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
  void uart_printf(const char *fmt, ...);
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
#define soft_pwm_button_Pin GPIO_PIN_2
#define soft_pwm_button_GPIO_Port GPIOB
#define Motor1_PH_Pin GPIO_PIN_12
#define Motor1_PH_GPIO_Port GPIOB
#define Motor2_PH_Pin GPIO_PIN_13
#define Motor2_PH_GPIO_Port GPIOB
#define Motor3_PH_Pin GPIO_PIN_14
#define Motor3_PH_GPIO_Port GPIOB
#define Motor4_PH_Pin GPIO_PIN_15
#define Motor4_PH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

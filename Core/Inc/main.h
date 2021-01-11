/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define Led_1_Pin GPIO_PIN_0
#define Led_1_GPIO_Port GPIOA
#define Led_2_Pin GPIO_PIN_1
#define Led_2_GPIO_Port GPIOA
#define Led_3_Pin GPIO_PIN_2
#define Led_3_GPIO_Port GPIOA
#define Led_4_Pin GPIO_PIN_3
#define Led_4_GPIO_Port GPIOA
#define Led_5_Pin GPIO_PIN_4
#define Led_5_GPIO_Port GPIOA
#define Led_6_Pin GPIO_PIN_5
#define Led_6_GPIO_Port GPIOA
#define Led_7_Pin GPIO_PIN_6
#define Led_7_GPIO_Port GPIOA
#define Led_8_Pin GPIO_PIN_7
#define Led_8_GPIO_Port GPIOA
#define EnA_Pin GPIO_PIN_9
#define EnA_GPIO_Port GPIOE
#define InA_High_Pin GPIO_PIN_10
#define InA_High_GPIO_Port GPIOE
#define InA_Low_Pin GPIO_PIN_11
#define InA_Low_GPIO_Port GPIOE
#define EnB_Pin GPIO_PIN_6
#define EnB_GPIO_Port GPIOC
#define InB_Low_Pin GPIO_PIN_7
#define InB_Low_GPIO_Port GPIOC
#define InB_High_Pin GPIO_PIN_8
#define InB_High_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

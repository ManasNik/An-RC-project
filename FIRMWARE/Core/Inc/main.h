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
#define Reverse_button_Pin GPIO_PIN_13
#define Reverse_button_GPIO_Port GPIOC
#define Throttle_Pin GPIO_PIN_0
#define Throttle_GPIO_Port GPIOA
#define Braking_Pin GPIO_PIN_1
#define Braking_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Reverse_Indicator_Pin GPIO_PIN_5
#define Reverse_Indicator_GPIO_Port GPIOA
#define PWMA_1_Pin GPIO_PIN_6
#define PWMA_1_GPIO_Port GPIOA
#define AIN1_1_Pin GPIO_PIN_7
#define AIN1_1_GPIO_Port GPIOA
#define AIN2_1_Pin GPIO_PIN_4
#define AIN2_1_GPIO_Port GPIOC
#define STBY_1_Pin GPIO_PIN_5
#define STBY_1_GPIO_Port GPIOC
#define PWMA_2_Pin GPIO_PIN_0
#define PWMA_2_GPIO_Port GPIOB
#define PWMB_2_Pin GPIO_PIN_1
#define PWMB_2_GPIO_Port GPIOB
#define AIN1_2_Pin GPIO_PIN_12
#define AIN1_2_GPIO_Port GPIOB
#define AIN2_2_Pin GPIO_PIN_13
#define AIN2_2_GPIO_Port GPIOB
#define STBY_2_Pin GPIO_PIN_14
#define STBY_2_GPIO_Port GPIOB
#define BIN1_2_Pin GPIO_PIN_15
#define BIN1_2_GPIO_Port GPIOB
#define BIN2_2_Pin GPIO_PIN_6
#define BIN2_2_GPIO_Port GPIOC
#define PWMB_1_Pin GPIO_PIN_7
#define PWMB_1_GPIO_Port GPIOC
#define BIN1_1_Pin GPIO_PIN_8
#define BIN1_1_GPIO_Port GPIOC
#define BIN2_1_Pin GPIO_PIN_9
#define BIN2_1_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

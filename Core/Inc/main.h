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
#define LED_BUILTIN_Pin GPIO_PIN_13
#define LED_BUILTIN_GPIO_Port GPIOC
#define M1_ISENS_U_Pin GPIO_PIN_0
#define M1_ISENS_U_GPIO_Port GPIOC
#define M1_ISENS_V_Pin GPIO_PIN_1
#define M1_ISENS_V_GPIO_Port GPIOC
#define M1_ISENS_W_Pin GPIO_PIN_2
#define M1_ISENS_W_GPIO_Port GPIOC
#define M2_ISENS_U_Pin GPIO_PIN_3
#define M2_ISENS_U_GPIO_Port GPIOC
#define M2_HALL_SENS_A_Pin GPIO_PIN_0
#define M2_HALL_SENS_A_GPIO_Port GPIOA
#define M2_HALL_SENS_B_Pin GPIO_PIN_1
#define M2_HALL_SENS_B_GPIO_Port GPIOA
#define M2_HALL_SENS_C_Pin GPIO_PIN_2
#define M2_HALL_SENS_C_GPIO_Port GPIOA
#define M2_ISENS_V_Pin GPIO_PIN_3
#define M2_ISENS_V_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define M2_ISENS_W_Pin GPIO_PIN_6
#define M2_ISENS_W_GPIO_Port GPIOA
#define M2_EN_U_Pin GPIO_PIN_7
#define M2_EN_U_GPIO_Port GPIOA
#define VBAT_SENS_Pin GPIO_PIN_4
#define VBAT_SENS_GPIO_Port GPIOC
#define M2_EN_V_Pin GPIO_PIN_0
#define M2_EN_V_GPIO_Port GPIOB
#define M2_EN_W_Pin GPIO_PIN_1
#define M2_EN_W_GPIO_Port GPIOB
#define Z_STEER_DET_Pin GPIO_PIN_12
#define Z_STEER_DET_GPIO_Port GPIOB
#define Z_STEER_DET_EXTI_IRQn EXTI15_10_IRQn
#define M1_EN_U_Pin GPIO_PIN_13
#define M1_EN_U_GPIO_Port GPIOB
#define M1_EN_V_Pin GPIO_PIN_14
#define M1_EN_V_GPIO_Port GPIOB
#define M1_EN_W_Pin GPIO_PIN_15
#define M1_EN_W_GPIO_Port GPIOB
#define M2_PWM_U_Pin GPIO_PIN_6
#define M2_PWM_U_GPIO_Port GPIOC
#define M2_PWM_V_Pin GPIO_PIN_7
#define M2_PWM_V_GPIO_Port GPIOC
#define M2_PWM_W_Pin GPIO_PIN_8
#define M2_PWM_W_GPIO_Port GPIOC
#define M1_PWM_U_Pin GPIO_PIN_8
#define M1_PWM_U_GPIO_Port GPIOA
#define M1_PWM_V_Pin GPIO_PIN_9
#define M1_PWM_V_GPIO_Port GPIOA
#define M1_PWM_W_Pin GPIO_PIN_10
#define M1_PWM_W_GPIO_Port GPIOA
#define Z_LIMIT_Pin GPIO_PIN_12
#define Z_LIMIT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

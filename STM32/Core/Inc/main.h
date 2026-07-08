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

#include "OLED.h"
#include "OLED_Data.h"
#include "MOTOR.h"
#include "jy61p.h"
#include "pid.h"
#include "Emm_V5.h"

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
#define Moter_3_IN_1_Pin GPIO_PIN_2
#define Moter_3_IN_1_GPIO_Port GPIOA
#define Moter_3_IN_2_Pin GPIO_PIN_3
#define Moter_3_IN_2_GPIO_Port GPIOA
#define Moter_4_IN_1_Pin GPIO_PIN_0
#define Moter_4_IN_1_GPIO_Port GPIOB
#define Moter_4_IN_2_Pin GPIO_PIN_1
#define Moter_4_IN_2_GPIO_Port GPIOB
#define Moter_1_IN_1_Pin GPIO_PIN_13
#define Moter_1_IN_1_GPIO_Port GPIOE
#define Moter_1_IN_2_Pin GPIO_PIN_14
#define Moter_1_IN_2_GPIO_Port GPIOE
#define Moter_2_IN_1_Pin GPIO_PIN_14
#define Moter_2_IN_1_GPIO_Port GPIOD
#define Moter_2_IN_2_Pin GPIO_PIN_15
#define Moter_2_IN_2_GPIO_Port GPIOD
#define Key_1_Pin GPIO_PIN_8
#define Key_1_GPIO_Port GPIOC
#define JY61_TX_Pin GPIO_PIN_10
#define JY61_TX_GPIO_Port GPIOC
#define JY61_RX_Pin GPIO_PIN_11
#define JY61_RX_GPIO_Port GPIOC
#define Key_2_Pin GPIO_PIN_3
#define Key_2_GPIO_Port GPIOD
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

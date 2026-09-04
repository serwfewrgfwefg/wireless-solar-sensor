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
#include "stm32u0xx_hal.h"

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
void Error_Handler(void);  /* 错误兜底处理：关闭中断并停在死循环，便于调试定位。 */

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOA
#define LED_STATUS1_Pin GPIO_PIN_10
#define LED_STATUS1_GPIO_Port GPIOB
#define GPOUT_Pin GPIO_PIN_11
#define GPOUT_GPIO_Port GPIOB
#define PWR_EN_Pin GPIO_PIN_12
#define PWR_EN_GPIO_Port GPIOB
#define DTR_Pin GPIO_PIN_13
#define DTR_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_14
#define RST_GPIO_Port GPIOB
#define RDY_Pin GPIO_PIN_15
#define RDY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

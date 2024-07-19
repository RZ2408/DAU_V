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
#include "stm32f0xx_hal.h"

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCUIN_RESET_Pin GPIO_PIN_3
#define MCUIN_RESET_GPIO_Port GPIOE
#define SYS_ISENSE_Pin GPIO_PIN_0
#define SYS_ISENSE_GPIO_Port GPIOB
#define PWRIN_VSENSE_Pin GPIO_PIN_1
#define PWRIN_VSENSE_GPIO_Port GPIOB
#define PMBUS_SCL_Pin GPIO_PIN_10
#define PMBUS_SCL_GPIO_Port GPIOB
#define PMBUS_SDA_Pin GPIO_PIN_11
#define PMBUS_SDA_GPIO_Port GPIOB
#define MAIN_PWR_ENABLE_Pin GPIO_PIN_14
#define MAIN_PWR_ENABLE_GPIO_Port GPIOB
#define PWR_LED_ON_Pin GPIO_PIN_13
#define PWR_LED_ON_GPIO_Port GPIOD
#define FAULT_LED_ON_Pin GPIO_PIN_14
#define FAULT_LED_ON_GPIO_Port GPIOD
#define CORE_RST_OUT_Pin GPIO_PIN_8
#define CORE_RST_OUT_GPIO_Port GPIOC
#define CORE_RST_IN_Pin GPIO_PIN_9
#define CORE_RST_IN_GPIO_Port GPIOC
#define CORE_3V3_5V_PG_Pin GPIO_PIN_3
#define CORE_3V3_5V_PG_GPIO_Port GPIOB
#define CORE_PWR_ISO_PG_Pin GPIO_PIN_4
#define CORE_PWR_ISO_PG_GPIO_Port GPIOB
#define CORE_PWR_CONN_PG_Pin GPIO_PIN_5
#define CORE_PWR_CONN_PG_GPIO_Port GPIOB
#define V3_PG_Pin GPIO_PIN_6
#define V3_PG_GPIO_Port GPIOB
#define V3STB_PG_Pin GPIO_PIN_7
#define V3STB_PG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

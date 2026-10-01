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
#include "stm32wlxx_hal.h"

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
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define RTC_N_PREDIV_S 10
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define IN_GPIO1_Pin GPIO_PIN_5
#define IN_GPIO1_GPIO_Port GPIOB
#define IN_GPIO2_Pin GPIO_PIN_6
#define IN_GPIO2_GPIO_Port GPIOB
#define IN_GPIO3_Pin GPIO_PIN_7
#define IN_GPIO3_GPIO_Port GPIOB
#define IN_GPIO4_Pin GPIO_PIN_8
#define IN_GPIO4_GPIO_Port GPIOB
#define TX2_Pin GPIO_PIN_2
#define TX2_GPIO_Port GPIOA
#define RX2_Pin GPIO_PIN_3
#define RX2_GPIO_Port GPIOA
#define RS485_DR_Pin GPIO_PIN_4
#define RS485_DR_GPIO_Port GPIOA
#define FREQ_HIGH_Pin GPIO_PIN_12
#define FREQ_HIGH_GPIO_Port GPIOB
#define Led_Signal_Pin GPIO_PIN_11
#define Led_Signal_GPIO_Port GPIOA
#define RF_SW_CTRL_Pin GPIO_PIN_13
#define RF_SW_CTRL_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

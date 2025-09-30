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
#include "stm32l4xx_hal.h"

#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_gpio.h"

#include "stm32l4xx_ll_exti.h"

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
#define PC3_BATTMON_EN_Pin GPIO_PIN_3
#define PC3_BATTMON_EN_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define PB0_USART3_EN_Pin GPIO_PIN_0
#define PB0_USART3_EN_GPIO_Port GPIOB
#define PB1_USART2_EN_Pin GPIO_PIN_1
#define PB1_USART2_EN_GPIO_Port GPIOB
#define SD_PWR_Pin GPIO_PIN_2
#define SD_PWR_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CLK_OE_Pin GPIO_PIN_6
#define CLK_OE_GPIO_Port GPIOC
#define TIM3_CH2_CLOCKOUT_Pin GPIO_PIN_7
#define TIM3_CH2_CLOCKOUT_GPIO_Port GPIOC
#define PA8_USART1_RX_INT_Pin GPIO_PIN_8
#define PA8_USART1_RX_INT_GPIO_Port GPIOA
#define PB4_AUX_SEL_A0_Pin GPIO_PIN_4
#define PB4_AUX_SEL_A0_GPIO_Port GPIOB
#define PB5_AUX_SEL_A1_Pin GPIO_PIN_5
#define PB5_AUX_SEL_A1_GPIO_Port GPIOB
#define PB8_TRUCK_INT_IN_Pin GPIO_PIN_8
#define PB8_TRUCK_INT_IN_GPIO_Port GPIOB
#define PB9_TRUCK_INT_OUT_Pin GPIO_PIN_9
#define PB9_TRUCK_INT_OUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* SD Card SPI Definitions */
#define SD_SPI_HANDLE hspi1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

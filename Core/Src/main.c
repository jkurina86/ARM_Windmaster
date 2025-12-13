/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shell.h"
#include "filesystem.h"
#include "tasker.h"
#include "windmaster.h"
#include "vectornav.h"
#include "recorder.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "ab-rtcmc-rtc.h"
#include "systime.h"
#include "stm32l4xx_ll_dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* RTC timer notification flag set by EXTI callback */
static volatile uint8_t rtc_timer_flag = 0;

/* 20 Hz timer notification flag */
//volatile uint8_t tick20_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void uart_rx_process_char(uint8_t ch);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_FATFS_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  shell_printf("\r\nSystem initializing...\r\n");

  /* Start the free-running 1 MHz timer */
  LL_TIM_EnableCounter(TIM2);

  /* UART Migration: Disable old UART4/5 and enable new USART2/3 for sensors */
  /* Disable UART4 (old WindMaster) and UART5 (old VectorNav) */
  LL_USART_DisableIT_RXNE(UART4);
  LL_USART_DisableIT_RXNE(UART5);

  /* Disable the old UART interrupts completely */
  HAL_NVIC_DisableIRQ(UART4_IRQn);
  HAL_NVIC_DisableIRQ(UART5_IRQn);

  /* Start the 20 Hz timer and interrupt */
  /*
  LL_TIM_EnableIT_UPDATE(TIM4);
  LL_TIM_EnableCounter(TIM4);
  */

  shell_printf("\r\n===============================================\r\n");
  shell_printf("       SYSTEM INIT\r\n");
  shell_printf("===============================================\r\n");

  /* Initialize RTC */
  RTC_Init();

  /* Get the current date/time from the RTC */
  RTC_DateTime_t initial_dt;
  RTC_GetDateTime(&initial_dt);

  /* Initialize system time with the RTC date/time */
  systime_init(&initial_dt);

  /* print current system time from systime */
  uint32_t full_epoch_sec = time_s_now();  /* time_s_now() already returns full epoch seconds */
  RTC_DateTime_t current_dt;
  current_dt = epoch_to_datetime(full_epoch_sec);
  shell_printf("System time initialized to: %02d-%02d-20%02d %02d:%02d:%02d\r\n",
              current_dt.months, current_dt.days, current_dt.years,
              current_dt.hours, current_dt.minutes, current_dt.seconds);

  shell_printf("Current timestamp: %s\r\n", timestamp(time_s_now()));

  /* Initialize the filesystem */
  filesystem_init();

  /* Initialize the task scheduler */
  tasker_init();

  /* Initialize the shell */
  shell_init();

  /* Initialize RS232 transceivers */
  transceiver_init();

  /* Initialize UART interrupts */
  init_uart_interrupts();

  /* Initialize the WM - windmaster.c */
  wm_init();

  /* Initialize the VN300 - vn.c */
  vn_init();

  /* Allow time for SD card power and UART lines to stabilize and clear any flags */
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /* Kick the Dog (2 second timeout until system reset) */
    HAL_IWDG_Refresh(&hiwdg);

    /* Process recorder queues and write to SD, returns fast if not recording */
    recorder_service();

    /* Shell parsing, processing, and task scheduling */
    shell_task();

    /* Execute any pending tasks */
    tasker_run();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCCEx_EnableLSECSS();
}

/* USER CODE BEGIN 4 */

/** @brief Enter low-power snooze mode for a specified number of seconds
  * @param seconds: Number of seconds to snooze
  * @retval None
  */
void snooze (uint16_t seconds) {
  /* Check if the recorder is running */
  if (recorder_is_recording()) {
    shell_printf("[SNOOZE] ERROR: Cannot enter snooze while recording is active!\r\n");
    return;
  }

  /* Disable the TIM2 1Mhz free-running counter */
  LL_TIM_DisableCounter(TIM2);
  /* Disable the TIM3 counter */
  LL_TIM_DisableCounter(TIM3);
  /* Disable TIM4 */
  LL_TIM_DisableCounter(TIM4);

  /* Turn off the SPI Bus */
  LL_SPI_Disable(SPI1);
  LL_SPI_Disable(SPI2);

  /* Turn off SD Card Power */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /* Turn off UARTs */
  LL_USART_Disable(USART1);
  LL_USART_Disable(USART2);
  LL_USART_Disable(USART3);
  LL_USART_Disable(UART4);
  LL_USART_Disable(UART5);

  /* Turn off the RS232 transceivers */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // USART3
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // USART2
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // UART5
  /* UART4 RS232 Transceiver is Always-On */

  /* Configure RTC Wakeup Timer using HAL */
  /* Use RTC wakeup timer with RTCCLK/16 = LSE/16 = 32768/16 = 2048 Hz */
  /* For N seconds wakeup, counter = (N * 2048) - 1 */
  /* LSE provides precise 32.768 kHz crystal accuracy */

  uint32_t wakeup_counter = (seconds * 2048) - 1;

  /* Set wakeup timer with interrupt mode */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeup_counter, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    /* Timer configuration failed - should not happen */
    Error_Handler();
  }

  /* Enter Stop Mode 2 for lowest power consumption */
  /* Stop Mode 2: All clocks stopped, SRAM and registers retained */
  /* RTC continues running and can wake up the system */
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

  /* CPU will resume here after wakeup interrupt */
  /* System clock needs to be reconfigured (done in wakeup() function) */
}

/** @brief Wake up from low-power snooze mode
  * @param None
  * @retval None
  */
void wakeup (void) {
  /* After waking from Stop Mode 2, system clock is MSI (4 MHz default) */
  /* Need to reconfigure system clock to PLL */
  SystemClock_Config();

  /* Disable RTC wakeup timer using HAL */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* Re-enable the TIM2 1Mhz free-running counter */
  LL_TIM_EnableCounter(TIM2);

  /* Re-enable the TIM3 counter for PPS synchronization */
  LL_TIM_EnableCounter(TIM3);

  /* Turn on SD Card Power and wait for stabilization */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_Delay(10);

  /* Turn on the SPI Bus */
  LL_SPI_Enable(SPI1);
  LL_SPI_Enable(SPI2);

  /* Turn on the RS232 transceivers */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); /* USART3 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); /* USART2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); /* UART5 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); /* UART5 second control */
  /* UART4 RS232 Transceiver is Always-On */

  /* Turn on UARTs */
  LL_USART_Enable(USART1);
  LL_USART_Enable(USART2);
  LL_USART_Enable(USART3);
  LL_USART_Enable(UART4);
  LL_USART_Enable(UART5);

  /* Re-initialize UART interrupts for shell */
  init_uart_interrupts();

  /* Small delay to allow peripherals to stabilize */
  HAL_Delay(50);
}

/**
  * @brief UART RX-Complete callback (HAL library)
  * @param huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Handle shell input on USART1 */
    if (huart->Instance == USART1) {
        shell_uart_receive_callback();
    }
}

/**
  * @brief  EXTI line detection callback
  * @param  GPIO_Pin: Specifies the pin connected to the EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

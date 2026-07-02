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
#include "calculations.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "ab-rtcmc-rtc.h"
#include "systime.h"
#include "stm32l4xx_ll_dma.h"
#include "telos.h"
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
//static volatile uint8_t rtc_timer_flag = 0;

/* 20 Hz timer notification flag */
//volatile uint8_t tick20_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void gps_time_sync(void);
void systime_startup(void);
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
  /* USER CODE BEGIN 2 */

  /* Start the free-running 1 MHz timer */
  LL_TIM_EnableCounter(TIM2);


  /* Start the 20 Hz timer and interrupt */
  /*
  LL_TIM_EnableIT_UPDATE(TIM4);
  LL_TIM_EnableCounter(TIM4);
  */
  shell_printf("SYSTEM INIT\r\n");

  systime_startup();
  filesystem_init();
  tasker_init();
  calc_init();
  transceiver_init();

  /* Allow the VectorNav to boot up after its power/transceiver enable is asserted */
  shell_printf("Waiting 5s for Sensors to boot...\r\n");
  HAL_Delay(5000);

  init_uart_interrupts();
  wm_init();
  vn_init();
  telos_init();

  /* Allow time for SD card power and UART lines to stabilize and clear any flags */
  HAL_Delay(100);

  /* Verify the VectorNav is alive before attempting a blocking GPS fix loop */
  shell_printf("Checking VectorNav connectivity...");
  if (!vn_check_alive()) {
    shell_printf("FAILED!\r\n");
    shell_printf("ERROR: VectorNav not responding. Check USART3 wiring and PB0 enable.\r\n");
    Error_Handler();
  }
  shell_printf("OK\r\n");

  /* Verify the WindMaster is alive before entering the main loop */
  shell_printf("Checking WindMaster connectivity...");
  if (!wm_check_alive()) {
    shell_printf("FAILED!\r\n");
    shell_printf("WARNING: WindMaster not responding. Check USART2 wiring and PB1 enable.\r\n");
  } else {
    shell_printf("OK\r\n");
  }

  /* Get a GPS fix */
  shell_printf("Getting GPS fix...");
  while (!vn_gps_fix()) {
      HAL_Delay(5000);
      shell_printf(".");
  }
  shell_printf("\nGPS fix acquired!\n");
  gps_time_sync();

  shell_printf("System initialization complete.\n");
  shell_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Process recorder queues and write to SD, returns fast if not recording */
    recorder_service();

    /* Process calculation buffers, returns fast if no buffer ready */
    calc_service();

    /* Process TELOS commands */
    telos_service();

    /* Process any bad WindMaster packets */
    bad_packet_recorder_service();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
void snooze (uint16_t seconds)
{
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // USART2 (WindMaster)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // UART5 spare
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); /* UART5 spare second control */
  /* UART4 TELOS RS232 transceiver is always on */

  /* Configure on-chip RTC Wakeup Timer using HAL */
  /* Use RTC wakeup timer with RTCCLK/16 = LSE/16 = 32768/16 = 2048 Hz */
  /* For N seconds wakeup, counter = (N * 2048) - 1 */

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
void wakeup (void)
{
  /* After waking from Stop Mode 2, system clock is MSI (4 MHz default) */
  /* Need to reconfigure system clock to PLL */
  SystemClock_Config();

  /* Disable RTC wakeup timer using HAL */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* Re-enable the TIM2 1Mhz free-running counter */
  LL_TIM_EnableCounter(TIM2);

  /* Re-enable the TIM3 counter for PPS synchronization */
  LL_TIM_EnableCounter(TIM3);

  /* Reset the system time from the RTC */
  RTC_DateTime_t wakeup_dt;
  RTC_GetDateTime(&wakeup_dt);
  systime_init(&wakeup_dt);

  /* Turn on SD Card Power and wait for stabilization */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_Delay(10);

  /* Turn on the SPI Bus */
  LL_SPI_Enable(SPI1);
  LL_SPI_Enable(SPI2);

  /* Turn on the RS232 transceivers */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   /* USART3 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   /* USART2 (WindMaster) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   /* UART5 spare */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); /* UART5 spare second control */
  /* UART4 TELOS RS232 transceiver is always on */

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

/** @brief Initialize system time from RTC at startup
  *  @param None
  *  @retval None
  *  @note   Reads the current date/time from the RTC and initializes systime module.
  *         Prints the initialized system time to the shell.
  */
void systime_startup(void)
{
  /* Initialize RTC */
  RTC_Status_t rtc_status = RTC_Init();
  if (rtc_status != RTC_OK) {
    shell_printf("RTC_Init failed with status: %d\r\n", rtc_status);
    shell_printf("  (0=OK, 1=ERROR, 2=TIMEOUT, 3=INVALID_PARAM, 4=EEPROM_BUSY)\r\n");
  } else {
    shell_printf("RTC_Init OK, CLKOUT enabled @ 1Hz\r\n");
  }

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

  /* Wait and check PPS */
  shell_printf("Checking PPS (waiting 3 seconds)...\r\n");
  HAL_Delay(3000);
  uint64_t pps_count = systime_get_pps_count();
  shell_printf("PPS count: %u\r\n", (uint32_t)pps_count);
  if (pps_count == 0) {
    shell_printf("WARNING: No PPS detected! Check RTC CLKOUT and TIM3.\r\n");
  }
}

/** @brief Synchronize RTC and system time with GPS time from VectorNav
  *  @param None
  *  @retval None
  *  @note  Reads GPS week and time-of-week from VectorNav and updates system time
  */
void gps_time_sync(void)
{
  /* Get current GPS date/time from VectorNav (derived from GNSS week + TOW) */
  RTC_DateTime_t gps_dt = {0};
  const uint8_t MAX_TIME_ATTEMPTS = 10;
  uint8_t time_attempts = 0;
  while (time_attempts < MAX_TIME_ATTEMPTS) {
    gps_dt = vn_get_gps_datetime();

    bool dt_ok = (gps_dt.years <= 99) &&
            (gps_dt.months >= 1 && gps_dt.months <= 12) &&
            (gps_dt.days >= 1 && gps_dt.days <= 31) &&
            (gps_dt.hours <= 23) &&
            (gps_dt.minutes <= 59) &&
            (gps_dt.seconds <= 59);

    if (dt_ok) {
      break;
    }

    time_attempts++;
    HAL_Delay(50);
  }

  if (time_attempts < MAX_TIME_ATTEMPTS) {
    RTC_SetDateTime(&gps_dt);
    systime_init(&gps_dt);
    shell_printf("RTC and System Time updated to GPS time: %02d-%02d-20%02d %02d:%02d:%02d\r\n",
                gps_dt.months, gps_dt.days, gps_dt.years,
                gps_dt.hours, gps_dt.minutes, gps_dt.seconds);
  } else {
    shell_printf("WARNING: Failed to read valid time from VectorNav; leaving RTC unchanged.\r\n");
  }
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

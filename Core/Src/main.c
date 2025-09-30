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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shell.h"
#include "filesystem.h"
#include "tasker.h"
#include "dummy_WM.h"
#include "dummy_IMU.h"
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

typedef struct {
    char identifier[6];
    uint32_t packet_number;
} LogEntry;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_RX_BUFFER_SIZE 128
#define DMA_BUFFER_SIZE 16384
#define PACKET_SIZE 23
#define WM_TEST_PACKET_SIZE 10
#define IMU_TEST_PACKET_SIZE 11

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Print buffers and flags for UART2-5 */
char uart2_print_buffer[UART_RX_BUFFER_SIZE];
char uart3_print_buffer[UART_RX_BUFFER_SIZE];
char uart4_print_buffer[UART_RX_BUFFER_SIZE];
char uart5_print_buffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uart2_print_flag = 0;
volatile uint8_t uart3_print_flag = 0;
volatile uint8_t uart4_print_flag = 0;
volatile uint8_t uart5_print_flag = 0;

/* RTC timer notification flag set by EXTI callback */
static volatile uint8_t rtc_timer_flag = 0;

/* 20 Hz timer notification flag */
volatile uint8_t tick20_flag = 0;

/* DMA buffer extern */
extern uint8_t dma_buffer_wm[DMA_BUFFER_SIZE];
extern uint8_t dma_buffer_imu[DMA_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void rtc_init(void);
void uart_rx_process_char(uint8_t ch);
void init_uart_interrupts(void);

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
  /* USER CODE BEGIN 2 */

  /* Start the free-running 1 MHz timer */
  LL_TIM_EnableCounter(TIM2);

  /* Start the 20 Hz timer and interrupt */
  LL_TIM_EnableIT_UPDATE(TIM4);
  LL_TIM_EnableCounter(TIM4);

  shell_printf("\r\n===============================================\r\n");
  shell_printf("       SYSTEM INIT\r\n");
  shell_printf("===============================================\r\n");

  /* Initialize RTC */
  rtc_init();

  /* Get the current date/time from the RTC */
  RTC_DateTime_t initial_dt;
  RTC_GetDateTime(&initial_dt);

  /* Initialize system time with the RTC date/time */
  systime_init(&initial_dt);

  /* print current system time from systime */
  RTC_DateTime_t current_dt;
  current_dt = epoch_to_datetime(time_s_now());
  shell_printf("System time initialized to: %02d-%02d-20%02d %02d:%02d:%02d\r\n",
              current_dt.months, current_dt.days, current_dt.years,
              current_dt.hours, current_dt.minutes, current_dt.seconds);

  shell_printf("Current timestamp: %s\r\n", timestamp(time_us_now()));

  /* Debug: Check PPS count */
  uint64_t test_pps_count = systime_get_pps_count();
  shell_printf("PPS events since init: %u\r\n", (uint32_t)test_pps_count);

  /* Debug: Wait a few seconds to gather some PPS events */
  shell_printf("Waiting 3 seconds to gather PPS events...\r\n");
  HAL_Delay(3000);

  /* Debug: Check PPS count again */
  test_pps_count = systime_get_pps_count();
  shell_printf("PPS events since last check: %u\r\n", (uint32_t)test_pps_count);
  
  /* Debug: Check if systime has lock and show estimated frequency */
  if (systime_have_lock()) {
    shell_printf("Systime has lock. PPM estimate: %d\r\n", systime_ppm_estimate());
  } else {
    shell_printf("Systime no lock yet\r\n");
  }

  /* Debug: Check 20 Hz timer + timestamp */
  uint32_t tick20_counter = 0;
  shell_printf("20 Hz tick test (5 seconds):\r\n");
  for (int i = 0; i < 100; i++) { /* 100 ticks at 20 Hz = 5 seconds */
      HAL_Delay(50); /* 50 ms delay */
      if (tick20_flag) {
          tick20_counter++;
          tick20_flag = 0; /* Clear the flag */
          shell_printf("  %s\r\n", timestamp(time_us_now()));
      }
  }
  shell_printf("20 Hz ticks counted: %u (expected 100)\r\n", tick20_counter);

  /* Initialize the filesystem */
  filesystem_init();

  /* Initialize the task scheduler */
  tasker_init();

  /* Initialize the shell */
  shell_init();

  /* Initialize UART interrupts for input detection */
  init_uart_interrupts();

  /* Set PB0 GPIO high - ENABLE pin for the USART3 transceiver */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /* Set PB4 GPIO high and PB5 GPIO low to enable the UART5 transceiver */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /* Set PB1 GPIO high - ENABLE pin for USART2 transceiver */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /* Set PB2_GPIO high to supply power to the SD card */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  /* Allow time for SD card power and UART lines to stabilize and clear any flags */
  HAL_Delay(100);

  /* Debug: Create a FATFS filesystem Object and File Object */
  FATFS fs;
  FIL file;
  FRESULT fres;

  /* Debug: Mount the SD Card */
  if (f_mount(&fs, "", 1) == FR_OK) {
      shell_printf("SD Card mounted successfully\r\n");
  } else {
      shell_printf("Failed to mount SD Card\r\n");
  }

  /* Debug: Dummy WindMaster */
  dummy_WM_init();

  /* Debug: Dummy IMU */
  dummy_IMU_init();

  /* Debug: Start the WindMaster */
  uint64_t wm_start_time;
  dummy_WM_start(&wm_start_time);

  /* Debug: Start the IMU */
  uint64_t imu_start_time;
  dummy_IMU_start(&imu_start_time);
  
  /* Debug: Get current us timestamp */
  //uint64_t start_us = time_us_now();

  if (dummy_WM_is_running()) {
      shell_printf("Dummy WindMaster started\r\n");
  } else {
      shell_printf("Failed to start Dummy WindMaster\r\n");
  }

  /* Debug: Create and write to a test file */
  fres = f_open(&file, "init.txt", FA_OPEN_APPEND | FA_WRITE | FA_CREATE_ALWAYS);
  if (fres == FR_OK) {
      char log_buffer[128];
      const char *ts = timestamp(wm_start_time);
      snprintf(log_buffer, sizeof(log_buffer), "WindMaster Log Start @ %s \r\n", ts);
      UINT bytes_written;
      fres = f_write(&file, log_buffer, strlen(log_buffer), &bytes_written);
      if (fres == FR_OK && bytes_written == strlen(log_buffer)) {
          shell_printf("Created init.txt\r\n", bytes_written);
      } else {
          shell_printf("Failed to write to init.txt\r\n");
      }
  } else {
      shell_printf("Failed to create init.txt\r\n");
  }
  
  /* Debug: Wait for 5 seconds */
  HAL_Delay(5000);

  /* Debug: Stop the IMU */
  uint64_t imu_stop_us;
  dummy_IMU_stop(&imu_stop_us);

  /* Debug: Stop the WindMaster */
  uint64_t stop_us;
  dummy_WM_stop(&stop_us);

  /* Debug: Check if WindMaster is stopped */
  if (!dummy_WM_is_running()) {
      shell_printf("Dummy WindMaster stopped\r\n");
  } else {
      shell_printf("Failed to stop Dummy WindMaster\r\n");
  }

  /* Debug: Check if IMU is stopped */
  if (!dummy_IMU_is_running()) {
      shell_printf("Dummy IMU stopped\r\n");
  } else {
      shell_printf("Failed to stop Dummy IMU\r\n");
  }

  /* Debug: Collect test packets from both WM and IMU DMA buffers */
  LogEntry log_entries[2000];
  int wm_count = 0;
  int imu_count = 0;

  // Process WM buffer
  uint16_t dma_pos_wm = DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_5);
  for (uint16_t i = 0; i <= dma_pos_wm - WM_TEST_PACKET_SIZE && wm_count < 2000; i++) {
      if (dma_buffer_wm[i] == 0xB4 && dma_buffer_wm[i + 1] == 0xB4) {
          uint8_t* packet = &dma_buffer_wm[i];
          log_entries[wm_count].identifier[0] = packet[2];
          log_entries[wm_count].identifier[1] = packet[3];
          log_entries[wm_count].identifier[2] = packet[4];
          log_entries[wm_count].identifier[3] = packet[5];
          log_entries[wm_count].identifier[4] = '\0';
          uint32_t packet_number = (uint32_t)(packet[6] | (packet[7] << 8) | (packet[8] << 16) | (packet[9] << 24));
          log_entries[wm_count].packet_number = packet_number;
          wm_count++;
      }
  }

  // Process IMU buffer
  uint16_t dma_pos_imu = DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_2);
  for (uint16_t i = 0; i <= dma_pos_imu - IMU_TEST_PACKET_SIZE && imu_count < 2000; i++) {
      if (dma_buffer_imu[i] == 0xB4 && dma_buffer_imu[i + 1] == 0xB4) {
          uint8_t* packet = &dma_buffer_imu[i];
          log_entries[wm_count + imu_count].identifier[0] = packet[2];
          log_entries[wm_count + imu_count].identifier[1] = packet[3];
          log_entries[wm_count + imu_count].identifier[2] = packet[4];
          log_entries[wm_count + imu_count].identifier[3] = packet[5];
          log_entries[wm_count + imu_count].identifier[4] = packet[6];
          log_entries[wm_count + imu_count].identifier[5] = '\0';
          uint32_t packet_number = (uint32_t)(packet[7] | (packet[8] << 8) | (packet[9] << 16) | (packet[10] << 24));
          log_entries[wm_count + imu_count].packet_number = packet_number;
          imu_count++;
      }
  }

  // Write to file, interleaving WM and IMU packets
  int max_count = (wm_count > imu_count) ? wm_count : imu_count;
  for (int i = 0; i < max_count; i++) {
      if (i < wm_count) {
          char log_entry[256];
          snprintf(log_entry, sizeof(log_entry), "%s %lu\n", log_entries[i].identifier, log_entries[i].packet_number);
          UINT bytes_written;
          f_write(&file, log_entry, strlen(log_entry), &bytes_written);
      }
      if (i < imu_count) {
          char log_entry[256];
          snprintf(log_entry, sizeof(log_entry), "%s %lu\n", log_entries[wm_count + i].identifier, log_entries[wm_count + i].packet_number);
          UINT bytes_written;
          f_write(&file, log_entry, strlen(log_entry), &bytes_written);
      }
  }

  shell_printf("Total bytes received WM: %u\n", dma_pos_wm);
  shell_printf("Total bytes received IMU: %u\n", dma_pos_imu);
  shell_printf("Total test packets found: %u\n", wm_count + imu_count);

  /* Debug: Write a log footer with the end timestamp */
  char footer[128];
  const char* ts_end = timestamp(stop_us);
  snprintf(footer, sizeof(footer), "\r\nWindMaster Log End @ %s\r\n", ts_end);
  UINT bytes_written;
  f_write(&file, footer, strlen(footer), &bytes_written);

  /* Debug: Close the log file */
  f_close(&file);

  /* Debug: UnMount the SD Card */
  if (f_mount(NULL, "0:", 0) == FR_OK) {
      shell_printf("SD Card unmounted successfully\r\n");
  } else {
      shell_printf("Failed to unmount SD Card\r\n");
  }

  /* Debug: Wait for 3 seconds before continuing */
  HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Shell processing and parsing */
    shell_task();

    /* Run any pending tasks */
    tasker_run();

    /* UART2-5 print notifications */
    if (uart2_print_flag) {
        uart2_print_flag = 0;
        shell_printf("\r\nUART2: %s\r\n", uart2_print_buffer);
    }
    if (uart3_print_flag) {
        uart3_print_flag = 0;
        shell_printf("\r\nUART3: %s\r\n", uart3_print_buffer);
    }
    if (uart4_print_flag) {
        uart4_print_flag = 0;
        shell_printf("\r\nUART4: %s\r\n", uart4_print_buffer);
    }
    if (uart5_print_flag) {
        uart5_print_flag = 0;
        shell_printf("\r\nUART5: %s\r\n", uart5_print_buffer);
    }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
}

/* USER CODE BEGIN 4 */

void rtc_init(void) {
  /* Initialize the RTC and enable CLOCKOUT @ 1 Hz */
  if (RTC_Init() == RTC_OK) {
    /* Set CLKOUT frequency to 1Hz using EEPROM Control register */
    uint8_t eeprom_ctrl;
    if (RTC_ReadEEPROM(RTC_REG_EEPROM_CONTROL, &eeprom_ctrl) == RTC_OK) {
        /* Clear existing FD0 and FD1 bits */
        eeprom_ctrl &= ~(RTC_EEPROM_CTRL_FD0 | RTC_EEPROM_CTRL_FD1);
        /* Set FD1=1, FD0=1 for 1Hz */
        eeprom_ctrl |= RTC_EEPROM_CTRL_FD0;  // FD0=1
        eeprom_ctrl |= RTC_EEPROM_CTRL_FD1;  // FD1=1
        /* Enable temperature compensation for accuracy */
        eeprom_ctrl &= ~RTC_EEPROM_CTRL_THP; // ThP=0 for 1s scanning interval
        eeprom_ctrl |= RTC_EEPROM_CTRL_THE;  // ThE=1 to enable thermometer
        
        if (RTC_WriteEEPROM(RTC_REG_EEPROM_CONTROL, eeprom_ctrl) == RTC_OK) {            
            /* Enable clock output */
            RTC_EnableClockOutput(true);
            HAL_Delay(100); // Wait longer for CLKOUT to stabilize with new frequency
            shell_printf("RTC CLKOUT enabled @ 1Hz\r\n");
        } else {
            shell_printf("EEPROM write failed\r\n");
        }
    } else {
        shell_printf("EEPROM read failed\r\n");
    }
  } else {
      shell_printf("RTC init failed\r\n");
  }

  /* Enable TIM3 counter to start capturing RTC CLKOUT */
  LL_TIM_EnableCounter(TIM3);
}

/**
  * @brief Initialize UART interrupts for input detection
  * @param None
  * @retval None
  */
void init_uart_interrupts(void)
{
  volatile uint32_t dummy;
  /* Flush UART RX Registers with a dummy read */
  if (LL_USART_IsActiveFlag_RXNE(USART2)) dummy = LL_USART_ReceiveData8(USART2);
  if (LL_USART_IsActiveFlag_RXNE(USART3)) dummy = LL_USART_ReceiveData8(USART3);
  if (LL_USART_IsActiveFlag_RXNE(UART4)) dummy = LL_USART_ReceiveData8(UART4);
  if (LL_USART_IsActiveFlag_RXNE(UART5)) dummy = LL_USART_ReceiveData8(UART5); 
  (void)dummy;

  /* Clear USART2 RX Flags */
  LL_USART_ClearFlag_ORE(USART2);  /* Overrun Error Flag */
  LL_USART_ClearFlag_NE(USART2);   /* Noise Error Flag */
  LL_USART_ClearFlag_PE(USART2);   /* Parity Error Flag */
  LL_USART_ClearFlag_FE(USART2);   /* Framing Error Flag */
  /* Clear USART3 RX Flags */
  LL_USART_ClearFlag_ORE(USART3);  /* Overrun Error Flag */
  LL_USART_ClearFlag_NE(USART3);   /* Noise Error Flag */
  LL_USART_ClearFlag_PE(USART3);   /* Parity Error Flag */
  LL_USART_ClearFlag_FE(USART3);   /* Framing Error Flag */
  /* Clear UART4 RX Flags */
  LL_USART_ClearFlag_ORE(UART4);  /* Overrun Error Flag */
  LL_USART_ClearFlag_NE(UART4);   /* Noise Error Flag */
  LL_USART_ClearFlag_PE(UART4);   /* Parity Error Flag */
  LL_USART_ClearFlag_FE(UART4);   /* Framing Error Flag */
  /* Clear UART5 RX Flags */
  LL_USART_ClearFlag_ORE(UART5);  /* Overrun Error Flag */
  LL_USART_ClearFlag_NE(UART5);   /* Noise Error Flag */
  LL_USART_ClearFlag_PE(UART5);   /* Parity Error Flag */
  LL_USART_ClearFlag_FE(UART5);   /* Framing Error Flag */

  /* Enable The UART RXNE interrupts */
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_RXNE(USART3);
  LL_USART_EnableIT_RXNE(UART4);
  LL_USART_EnableIT_RXNE(UART5);
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

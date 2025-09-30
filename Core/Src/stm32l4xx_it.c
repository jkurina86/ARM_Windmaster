/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "stm32l4xx_ll_dma.h"
#include "systime.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_RX_BUFFER_SIZE 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* RX index for each UART */
static uint16_t uart2_rx_index = 0;
static uint16_t uart3_rx_index = 0;
//static uint16_t uart4_rx_index = 0;
//static uint16_t uart5_rx_index = 0;

/* RX character for each UART */
static uint8_t uart2_rx_char;
static uint8_t uart3_rx_char;
//static uint8_t uart4_rx_char;
//static uint8_t uart5_rx_char;

/* RX buffer for each UART */
char uart2_rx_buffer[UART_RX_BUFFER_SIZE];
char uart3_rx_buffer[UART_RX_BUFFER_SIZE];
//char uart4_rx_buffer[UART_RX_BUFFER_SIZE];
//char uart5_rx_buffer[UART_RX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void uart_user_input(USART_TypeDef *Instance, uint8_t *rx_char, char *rx_buffer, uint16_t *rx_index, char *print_buffer, volatile uint8_t *print_flag);
void rtc_countdown(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* External (main.c) print buffer for UART */
extern char uart2_print_buffer[UART_RX_BUFFER_SIZE];
extern char uart3_print_buffer[UART_RX_BUFFER_SIZE];
//extern char uart4_print_buffer[UART_RX_BUFFER_SIZE];
//extern char uart5_print_buffer[UART_RX_BUFFER_SIZE];

/* External (main.c) print flags for UART */
extern volatile uint8_t uart2_print_flag;
extern volatile uint8_t uart3_print_flag;
//extern volatile uint8_t uart4_print_flag;
//extern volatile uint8_t uart5_print_flag;

/* External (main.c) 20 Hz timer notification flag */
extern volatile uint8_t tick20_flag;

/* External DMA buffer variables from dummy_WM.c */
extern uint8_t dma_buffer[32768];
extern uint16_t dma_old_pos;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  
  /* Note this is for the 1 Hz CLOCKOUT signal from the RTC */
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* Trigger PPS event for system time */
  if (LL_TIM_IsActiveFlag_CC2(TIM3)) {
    LL_TIM_ClearFlag_CC2(TIM3);
    systime_pps_event();
  }

  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* 20 Hz Timer */
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */
  
  if (LL_TIM_IsActiveFlag_UPDATE(TIM4)) {
      LL_TIM_ClearFlag_UPDATE(TIM4);
      tick20_flag = 1;  // Set the 20 Hz tick flag
  }

  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* Handle USART2 RXNE interrupt */
  uart_user_input(USART2, &uart2_rx_char, uart2_rx_buffer, &uart2_rx_index, uart2_print_buffer, &uart2_print_flag);

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* Handle USART3 RXNE interrupt */
  uart_user_input(USART3, &uart3_rx_char, uart3_rx_buffer, &uart3_rx_index, uart3_print_buffer, &uart3_print_flag);

  /* USER CODE END USART3_IRQn 0 */
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel2 global interrupt.
  */
void DMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */

  /* USER CODE END DMA2_Channel2_IRQn 0 */
  /* USER CODE BEGIN DMA2_Channel2_IRQn 1 */

  /* USER CODE END DMA2_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel5 global interrupt.
  */
void DMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

  // Handle DMA interrupts for UART4 RX
  if (LL_DMA_IsActiveFlag_TC5(DMA2)) {
    LL_DMA_ClearFlag_TC5(DMA2);
    // Transfer complete - buffer is full
  }
  if (LL_DMA_IsActiveFlag_HT5(DMA2)) {
    LL_DMA_ClearFlag_HT5(DMA2);
    // Half transfer - buffer is half full
  }

  /* USER CODE END DMA2_Channel5_IRQn 0 */
  /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */

  /* USER CODE END DMA2_Channel5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/** @brief RTC Countdown, used for testing/bringup.
  * @retval None
  * @note This function is called from TIM3 IRQ handler on each rising edge of RTC CLKOUT.
  * @note Printing in the IRQ is bad, but this is just for initial verification.
  */
void rtc_countdown(void) {
  /* Static variables to hold state between calls */
  static uint8_t rtc_sec_counter = 0;
  static uint8_t countdown_completed = 0;

  /* Early return fencepost if the countdown is complete */
  if (countdown_completed) {
      return;
  }
    
  rtc_sec_counter++;
  if (rtc_sec_counter <= 10) {
    char buffer[64];
    sprintf(buffer, "RTC tick %d\r\n", rtc_sec_counter);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
  } else {
    HAL_UART_Transmit(&huart1, (uint8_t*)"RTC countdown complete\r\n", 24, HAL_MAX_DELAY);
    countdown_completed = 1;  /* Mark countdown as completed - no more processing */
  }
}

/** @brief Common function to handle UART user input, used for testing/bringup.
  * @param Instance: USART instance (e.g., USART2, USART3, etc.)
  * @param rx_char: Pointer to store the received character
  * @param rx_buffer: Buffer to accumulate received characters
  * @param rx_index: Pointer to the current index in the rx_buffer
  * @param print_buffer: Buffer to copy the complete message for printing
  * @param print_flag: Pointer to a flag indicating a complete message is ready
  * @retval None
  * @note Reads characters into rx_buffer until CR or LF is received or buffer is full.
  */
void uart_user_input(USART_TypeDef *Instance, uint8_t *rx_char, char *rx_buffer, uint16_t *rx_index, char *print_buffer, volatile uint8_t *print_flag) {
  /* Check if the RXNE flag is set and the interrupt is enabled */
  if (LL_USART_IsActiveFlag_RXNE(Instance) && LL_USART_IsEnabledIT_RXNE(Instance)) {
    /* Read the received character */
    *rx_char = LL_USART_ReceiveData8(Instance);
    
    /* Process the character */
    if (*rx_index < UART_RX_BUFFER_SIZE - 1) { /* Buffer Overflow Check */
      rx_buffer[*rx_index] = *rx_char; /* Store received char */
      (*rx_index)++; /* Increment index */
      
      /* Check for end of line (CR or LF) */
      if (*rx_char == '\r' || *rx_char == '\n') {
        memcpy(print_buffer, rx_buffer, *rx_index);
        print_buffer[*rx_index - 1] = '\0';
        *print_flag = 1;
        *rx_index = 0; /* Reset index for next message */
      }
    } else { /* Stop accumulating when buffer is full, copy to print buffer, set the flag, and reset. */
      memcpy(print_buffer, rx_buffer, UART_RX_BUFFER_SIZE - 1);
      print_buffer[UART_RX_BUFFER_SIZE - 1] = '\0';
      *print_flag = 1;
      *rx_index = 0;
    }
  }
}

/* USER CODE END 1 */

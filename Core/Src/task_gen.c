/**
  ******************************************************************************
  * @file    task_gen.c
  * @brief   General task handlers implementation
  * @note    Contains handlers for general system tasks
  ******************************************************************************
  */

#include "task_gen.h"
#include "tasker.h"
#include "shell.h"
#include "usart.h"
#include "main.h"
#include "stm32l4xx_ll_usart.h"
#include <string.h>

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Handle reset task
 * @param task_data: Pointer to task data containing reset parameters
 * @retval None
 */
void handle_reset(const task_data_t *task_data)
{
    shell_print("Resetting system in 3 seconds...\r\n");
    
    /* This just uses the HAL tick which is fine for resetting the system via shell */
    if (HAL_GetTick() >= task_data->reset.reset_due_ms) {
        shell_print("\r\nResetting now...\r\n");
        HAL_Delay(10); /* allow TX to flush */
        NVIC_SystemReset();
    }
    /* Note: Don't clear the task flag here as we want to keep checking until reset */
}

/**
 * @brief Handle hello task
 * @param task_data: Pointer to task data containing UART number
 * @retval None
 */
void handle_hello(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_HELLO);
    
    uint8_t uart_num = task_data->hello.uart_num;
    
    /* Validate UART number */
    if (uart_num < 1 || uart_num > 5) {
        shell_print("Usage: hello <uart_number>\r\n");
        shell_print("Send hello message to UART 1-5\r\n");
        shell_print("Example: hello 2\r\n");
        shell_print("Error: UART number must be between 1 and 5\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    const char *hello_msg = "\r\n ***Hello World!*** \r\n";
    UART_HandleTypeDef *huart_ptr = NULL;
    
    /* Select the appropriate UART handle */
    switch (uart_num) {
        case 1:
            huart_ptr = &huart1;
            break;
        case 2:
            {
                /* Send message using LL functions */
                for (int i = 0; i < strlen(hello_msg); i++) {
                    /* Wait until transmit data register is empty */
                    while (!LL_USART_IsActiveFlag_TXE(USART2));
                    /* Send character */
                    LL_USART_TransmitData8(USART2, hello_msg[i]);
                }
                /* Wait for transmission complete */
                while (!LL_USART_IsActiveFlag_TC(USART2));
                
                shell_printf("Hello message sent to UART%d\r\n", uart_num);
                shell_print(SHELL_PROMPT);
                return;
            }
        case 3:
            {
                /* Send message using LL functions */
                for (int i = 0; i < strlen(hello_msg); i++) {
                    /* Wait until transmit data register is empty */
                    while (!LL_USART_IsActiveFlag_TXE(USART3));
                    /* Send character */
                    LL_USART_TransmitData8(USART3, hello_msg[i]);
                }
                /* Wait for transmission complete */
                while (!LL_USART_IsActiveFlag_TC(USART3));
                
                shell_printf("Hello message sent to UART%d\r\n", uart_num);
                shell_print(SHELL_PROMPT);
                return;
            }
        case 4:
            {
                /* Send message using LL functions */
                for (int i = 0; i < strlen(hello_msg); i++) {
                    /* Wait until transmit data register is empty */
                    while (!LL_USART_IsActiveFlag_TXE(UART4));
                    /* Send character */
                    LL_USART_TransmitData8(UART4, hello_msg[i]);
                }
                /* Wait for transmission complete */
                while (!LL_USART_IsActiveFlag_TC(UART4));
                
                shell_printf("Hello message sent to UART%d\r\n", uart_num);
                shell_print(SHELL_PROMPT);
                return;
            }
        case 5:
            {
                /* Send message using LL functions */
                for (int i = 0; i < strlen(hello_msg); i++) {
                    /* Wait until transmit data register is empty */
                    while (!LL_USART_IsActiveFlag_TXE(UART5));
                    /* Send character */
                    LL_USART_TransmitData8(UART5, hello_msg[i]);
                }
                /* Wait for transmission complete */
                while (!LL_USART_IsActiveFlag_TC(UART5));
                
                shell_printf("Hello message sent to UART%d\r\n", uart_num);
                shell_print(SHELL_PROMPT);
                return;
            }
        default:
            /* This should never happen due to validation above, but just in case */
            shell_print("Error: Invalid UART number\r\n");
            shell_print(SHELL_PROMPT);
            return;
    }
    
    /* Send the hello message to the specified UART (for UART1) */
    HAL_StatusTypeDef result = HAL_UART_Transmit(huart_ptr, (uint8_t*)hello_msg, strlen(hello_msg), HAL_MAX_DELAY);
    
    if (result == HAL_OK) {
        shell_printf("Hello message sent to UART%d\r\n", uart_num);
    } else {
        shell_printf("Error sending message to UART%d (error code: %d)\r\n", uart_num, result);
    }
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle version task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_version(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_VERSION);
    
    shell_print("Firmware Information:\r\n");
    shell_print("====================\r\n");

    shell_printf("Shell Version: 1.0\r\n");
    shell_printf("Build Date: %s %s\r\n", __DATE__, __TIME__);
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle help task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_help(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_HELP);
    
    shell_print("Available commands:\r\n");
    shell_print("==================\r\n");

    for (int i = 0; shell_commands[i].name != NULL; i++) {
        shell_printf("  %-10s - %s\r\n",
                    shell_commands[i].name,
                    shell_commands[i].description);
    }
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle clear task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_clear(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_CLEAR);
    
    shell_print("\033[2J\033[H"); /* ANSI clear screen and home cursor */
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle status task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_status(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_STATUS);
    
    shell_print("System Status:\r\n");
    shell_print("=============\r\n");

    shell_printf("MCU: STM32L476RGT6\r\n");
    shell_printf("Core Clock: %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
    shell_printf("System Tick: %lu ms\r\n", HAL_GetTick());
    shell_print(SHELL_PROMPT);
}

/* Scheduling Functions ------------------------------------------------------*/

/**
 * @brief Schedule a reset task with delay
 * @param delay_ms: Delay in milliseconds before reset
 * @retval None
 */
void schedule_reset(uint32_t delay_ms)
{
    task_data_t task_data;
    task_data.reset.reset_due_ms = HAL_GetTick() + delay_ms;
    tasker_schedule_task(TASK_RESET, &task_data);
}

/**
 * @brief Schedule a hello task for specific UART
 * @param uart_num: UART number (1-5)
 * @retval None
 */
void schedule_hello(uint8_t uart_num)
{
    task_data_t task_data;
    task_data.hello.uart_num = uart_num;
    tasker_schedule_task(TASK_HELLO, &task_data);
}

/**
 * @brief Schedule a version task
 * @param None
 * @retval None
 */
void schedule_version(void)
{
    tasker_schedule_task(TASK_VERSION, NULL);
}

/**
 * @brief Schedule a help task
 * @param None
 * @retval None
 */
void schedule_help(void)
{
    tasker_schedule_task(TASK_HELP, NULL);
}

/**
 * @brief Schedule a clear terminal task
 * @param None
 * @retval None
 */
void schedule_clear(void)
{
    tasker_schedule_task(TASK_CLEAR, NULL);
}

/**
 * @brief Schedule a status task
 * @param None
 * @retval None
 */
void schedule_status(void)
{
    tasker_schedule_task(TASK_STATUS, NULL);
}

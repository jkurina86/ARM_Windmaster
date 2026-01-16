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
#include "rtc.h"
#include "systime.h"
#include "stm32l4xx_ll_usart.h"
#include <string.h>

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern RTC_HandleTypeDef hrtc;

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Handle reset task
 * @param arg Pointer to reset_args_t containing reset parameters
 */
void handle_reset(const void *arg)
{
    const reset_args_t *args = (const reset_args_t *)arg;
    
    /* Check if it's time to reset */
    if (HAL_GetTick() >= args->reset_due_ms) {
        shell_print("\r\nResetting now...\r\n");
        HAL_Delay(10); /* allow TX to flush */
        NVIC_SystemReset();
    } else {
        /* Not time yet - re-enqueue ourselves */
        shell_print("Resetting system in 3 seconds...\r\n");
        tasker_enqueue(handle_reset, arg, sizeof(reset_args_t));
    }
}

/**
 * @brief Handle hello task
 * @param arg Pointer to hello_args_t containing UART number
 */
void handle_hello(const void *arg)
{
    const hello_args_t *args = (const hello_args_t *)arg;
    uint8_t uart_num = args->uart_num;
    
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
                for (size_t i = 0; i < strlen(hello_msg); i++) {
                    while (!LL_USART_IsActiveFlag_TXE(USART2));
                    LL_USART_TransmitData8(USART2, hello_msg[i]);
                }
                while (!LL_USART_IsActiveFlag_TC(USART2));
                
                shell_printf("Hello message sent to UART%d\r\n", uart_num);
                shell_print(SHELL_PROMPT);
                return;
            }
        case 3:
            {
                for (size_t i = 0; i < strlen(hello_msg); i++) {
                    while (!LL_USART_IsActiveFlag_TXE(USART3));
                    LL_USART_TransmitData8(USART3, hello_msg[i]);
                }
                while (!LL_USART_IsActiveFlag_TC(USART3));
                
                shell_printf("Hello message sent to UART%d\r\n", uart_num);
                shell_print(SHELL_PROMPT);
                return;
            }
        case 4:
            {
                for (size_t i = 0; i < strlen(hello_msg); i++) {
                    while (!LL_USART_IsActiveFlag_TXE(UART4));
                    LL_USART_TransmitData8(UART4, hello_msg[i]);
                }
                while (!LL_USART_IsActiveFlag_TC(UART4));
                
                shell_printf("Hello message sent to UART%d\r\n", uart_num);
                shell_print(SHELL_PROMPT);
                return;
            }
        case 5:
            {
                for (size_t i = 0; i < strlen(hello_msg); i++) {
                    while (!LL_USART_IsActiveFlag_TXE(UART5));
                    LL_USART_TransmitData8(UART5, hello_msg[i]);
                }
                while (!LL_USART_IsActiveFlag_TC(UART5));
                
                shell_printf("Hello message sent to UART%d\r\n", uart_num);
                shell_print(SHELL_PROMPT);
                return;
            }
        default:
            shell_print("Error: Invalid UART number\r\n");
            shell_print(SHELL_PROMPT);
            return;
    }
    
    /* Send the hello message to UART1 */
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
 * @param arg Unused (pass NULL)
 */
void handle_version(const void *arg)
{
    (void)arg;
    
    shell_print("Firmware Information:\r\n");
    shell_print("====================\r\n");
    shell_printf("Shell Version: 1.0\r\n");
    shell_printf("Build Date: %s %s\r\n", __DATE__, __TIME__);
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle help task
 * @param arg Unused (pass NULL)
 */
void handle_help(const void *arg)
{
    (void)arg;
    
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
 * @param arg Unused (pass NULL)
 */
void handle_clear(const void *arg)
{
    (void)arg;
    
    shell_print("\033[2J\033[H"); /* ANSI clear screen and home cursor */
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle status task
 * @param arg Unused (pass NULL)
 */
void handle_status(const void *arg)
{
    (void)arg;
    
    shell_print("System Status:\r\n");
    shell_print("=============\r\n");
    shell_printf("MCU: STM32L476RGT6\r\n");
    shell_printf("Core Clock: %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
    shell_printf("System Tick: %lu ms\r\n", HAL_GetTick());
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle snooze task
 * @param arg Pointer to snooze_args_t containing snooze duration
 */
void handle_snooze(const void *arg)
{
    const snooze_args_t *args = (const snooze_args_t *)arg;
    uint16_t seconds = args->seconds;
    
    shell_printf("Entering low-power sleep mode for %u seconds...\r\n", seconds);
    
    snooze(seconds);
    
    /* Upon wakeup, re-initialize the system */
    wakeup();
    
    shell_print("Woke up from snooze mode.\r\n");
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle systime task - display current system time
 * @param arg Unused (pass NULL)
 */
void handle_systime(const void *arg)
{
    (void)arg;

    uint32_t epoch_sec = time_s_now();

    shell_print("System Time:\r\n");
    shell_print("============\r\n");
    shell_printf("Epoch: %lu seconds since 2000-01-01\r\n", epoch_sec);
    shell_printf("ISO 8601: %s\r\n", timestamp(epoch_sec));
    shell_printf("PPS count: %lu\r\n", systime_get_pps_count());
    shell_printf("PPS lock: %s\r\n", systime_have_lock() ? "YES" : "NO");
    shell_print(SHELL_PROMPT);
}

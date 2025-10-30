/**
  ******************************************************************************
  * @file    vn_cmd.c
  * @brief   VectorNav pass-through command implementation
  * @note    Direct UART5 communication for sensor configuration and testing
  ******************************************************************************
  */

#include "vn_cmd.h"
#include "shell.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"
#include <stdio.h>

/* External variables --------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Send a passthrough command to VectorNav and receive response
 * @param cmd: Command string to send (e.g., "$VNRRG,0*XX\r\n")
 * @retval true if response received, false on timeout/error
 */
bool vn_cmd_passthrough(const char *cmd)
{
    if (!cmd || *cmd == '\0') {
        shell_print("Error: Empty command\r\n");
        return false;
    }

    /* Check command length */
    if (strlen(cmd) > VN_CMD_MAX_LEN) {
        shell_print("Error: Command too long\r\n");
        return false;
    }

    /* Disable DMA temporarily to avoid interference */
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_2);
    LL_USART_DisableDMAReq_RX(UART5);

    /* Flush any pending RX data */
    while (LL_USART_IsActiveFlag_RXNE(UART5)) {
        (void)LL_USART_ReceiveData8(UART5);
    }

    /* Send command using LL driver */
    shell_printf(">> %s", cmd);
    const uint8_t *p = (const uint8_t *)cmd;
    while (*p) {
        while (!LL_USART_IsActiveFlag_TXE(UART5));
        LL_USART_TransmitData8(UART5, *p++);
    }

    /* Wait for TX complete */
    while (!LL_USART_IsActiveFlag_TC(UART5));

    /* Small delay to allow sensor to respond */
    HAL_Delay(10);

    /* Receive response */
    uint8_t response_buffer[VN_RESP_MAX_LEN];
    uint16_t response_len = vn_cmd_recv_response(response_buffer, VN_RESP_MAX_LEN, VN_RESP_TIMEOUT_MS);

    if (response_len == 0) {
        shell_print("<< No response (timeout after 3000ms)\r\n");
        shell_print("DEBUG: Check if sensor is powered and UART5 connection is correct\r\n");
        LL_USART_EnableDMAReq_RX(UART5);
        LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);
        return false;
    }

    /* Display response in both ASCII and hex */
    shell_print("<< ");
    for (uint16_t i = 0; i < response_len; i++) {
        uint8_t ch = response_buffer[i];
        if (ch >= 32 && ch <= 126) {
            /* Printable ASCII */
            shell_printf("%c", ch);
        } else if (ch == '\r') {
            shell_print("[CR]");
        } else if (ch == '\n') {
            shell_print("[LF]");
        } else {
            shell_printf("[%02X]", ch);
        }
    }
    shell_print("\r\n");

    /* Re-enable DMA */
    LL_USART_EnableDMAReq_RX(UART5);
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);

    return true;
}

/**
 * @brief Receive response from VectorNav with timeout
 * @param buffer: Buffer to store response
 * @param max_len: Maximum bytes to receive
 * @param timeout_ms: Timeout in milliseconds
 * @retval Number of bytes received, 0 on timeout
 */
uint16_t vn_cmd_recv_response(uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    uint16_t bytes_received = 0;
    bool received_cr = false;
    uint32_t last_byte_time = start_time;

    while (bytes_received < max_len) {
        uint32_t now = HAL_GetTick();

        /* Check timeout */
        if (now - start_time > timeout_ms) {
            break;
        }

        /* Check for received data */
        if (LL_USART_IsActiveFlag_RXNE(UART5)) {
            uint8_t ch = LL_USART_ReceiveData8(UART5);
            buffer[bytes_received++] = ch;
            last_byte_time = now;

            /* Look for CR+LF terminator */
            if (ch == '\r') {
                received_cr = true;
            } else if (ch == '\n' && received_cr) {
                break;  /* Got complete response */
            } else if (ch == '\n' && !received_cr) {
                /* LF without CR - also acceptable terminator */
                break;
            } else {
                received_cr = false;  /* Reset CR flag if not followed by another CR */
            }
        } else {
            /* If we've received some data but nothing for 100ms, likely end of message */
            if (bytes_received > 0 && (now - last_byte_time) > 100) {
                break;
            }
        }

        /* Small delay to prevent busy-waiting */
        HAL_Delay(1);
    }

    return bytes_received;
}

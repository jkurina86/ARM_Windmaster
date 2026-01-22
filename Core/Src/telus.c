/**
  ******************************************************************************
  * @file    telus.c
  * @brief   Telus communication module implementation
  * @note    Handles UART4 communication with Telus system.
  *          Receives "idata\r\n" commands and responds with CSV-formatted
  *          CalcReport data via DMA.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "telus.h"
#include "calculations.h"
#include "systime.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_usart.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
#define RX_BUFFER_SIZE      64      /* DMA RX buffer size */
#define TX_BUFFER_SIZE      6144    /* TX buffer for CSV response (~6KB for 30 reports) */
#define COMMAND_LEN         7       /* Length of "idata\r\n" */

/* Private types -------------------------------------------------------------*/
typedef enum {
    TELUS_IDLE,         /* Waiting for command */
    TELUS_SENDING       /* DMA transfer in progress */
} TelusState_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static char tx_buffer[TX_BUFFER_SIZE] __attribute__((section(".telus_tx")));
static volatile TelusState_t state = TELUS_IDLE;
static uint16_t rx_read_pos = 0;
static volatile uint16_t tx_length = 0;

/* Command to match */
static const char COMMAND[] = "idata\r\n";

/* CSV header row */
static const char CSV_HEADER[] =
    "timestamp,latitude,longitude,u_mean,u_std,v_mean,v_std,w_mean,w_std,wind_speed_mean,wind_speed_std,wind_from_mean,wind_from_std,gust_mean,gust_std\r\n";

/* Private function prototypes -----------------------------------------------*/
static void configure_dma_rx(void);
static void configure_dma_tx(void);
static void restart_dma_rx(void);
static bool check_for_command(void);
static uint16_t build_csv_response(void);
static void start_dma_tx(uint16_t length);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initialize Telus module
 */
void telus_init(void) {
    /* Clear buffers */
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    memset(tx_buffer, 0, TX_BUFFER_SIZE);

    /* Reset state */
    state = TELUS_IDLE;
    rx_read_pos = 0;
    tx_length = 0;

    /* Configure DMA channels */
    configure_dma_rx();
    configure_dma_tx();

    /* Enable UART4 DMA requests */
    LL_USART_EnableDMAReq_RX(UART4);
    LL_USART_EnableDMAReq_TX(UART4);

    /* Enable UART4 */
    LL_USART_Enable(UART4);

    /* Start RX DMA */
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);
}

/**
 * @brief  Service routine - call from main loop
 */
void telus_service(void) {
    /* Don't process if currently sending */
    if (state == TELUS_SENDING) {
        return;
    }

    /* Check for incoming command */
    if (check_for_command()) {
        /* Build CSV response */
        uint16_t len = build_csv_response();

        if (len > 0) {
            /* Start DMA transmission */
            start_dma_tx(len);
        }
    }
}

/**
 * @brief  DMA TX complete callback - call from DMA2_Channel3_IRQHandler
 * @retval None
 * @note   Called when UART4 TX DMA transfer is complete.
 */
void telus_tx_complete(void) {
    /* Disable TX DMA channel */
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);

    /* Clear the report buffer after successful transmission */
    calc_clear_reports();

    /* Return to idle state */
    state = TELUS_IDLE;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Configure DMA2 Channel 5 for UART4 RX
 * @note   Sets up DMA for reception into a linear buffer.
 *         Must be restarted after buffer fills or after processing.
 */
static void configure_dma_rx(void) {
    /* Disable channel before configuration */
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_5);

    /* Configure addresses */
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_5,
                           LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_RECEIVE),
                           (uint32_t)rx_buffer,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    /* Set data length */
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_5, RX_BUFFER_SIZE);
}

/**
 * @brief  Restart DMA RX after processing
 * @note   Resets buffer position and restarts DMA transfer.
 */
static void restart_dma_rx(void) {
    /* Disable channel */
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_5);

    /* Clear buffer */
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    rx_read_pos = 0;

    /* Reset data length */
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_5, RX_BUFFER_SIZE);

    /* Clear flags */
    LL_DMA_ClearFlag_TC5(DMA2);
    LL_DMA_ClearFlag_HT5(DMA2);
    LL_DMA_ClearFlag_TE5(DMA2);

    /* Re-enable channel */
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);
}

/**
 * @brief  Configure DMA2 Channel 3 for UART4 TX (normal mode)
 * @retval None
 * @note   Sets up DMA for transmitting data from tx_buffer.
 */
static void configure_dma_tx(void) {
    /* Disable channel before configuration */
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);

    /* Configure addresses */
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_3,
                           (uint32_t)tx_buffer,
                           LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_TRANSMIT),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    /* Enable transfer complete interrupt */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_3);
}

/**
 * @brief  Check RX buffer for "idata\r\n" command
 * @retval true if command found, false otherwise
 * @note   Scans the linear RX buffer for the command. Restarts DMA after
 *         finding command or when buffer is nearly full.
 */
static bool check_for_command(void) {
    /* Get current write position from DMA counter */
    uint16_t dma_remaining = LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_5);
    uint16_t wr = RX_BUFFER_SIZE - dma_remaining;
    uint16_t rd = rx_read_pos;

    /* Calculate available bytes */
    uint16_t avail = (wr > rd) ? (wr - rd) : 0;

    /* Need at least COMMAND_LEN bytes */
    if (avail < COMMAND_LEN) {
        /* If DMA is complete (buffer full), restart it */
        if (dma_remaining == 0) {
            restart_dma_rx();
        }
        return false;
    }

    /* Scan for command in buffer */
    while (avail >= COMMAND_LEN) {
        /* Check if current position matches command */
        bool match = true;
        for (uint16_t i = 0; i < COMMAND_LEN; i++) {
            if (rx_buffer[rd + i] != COMMAND[i]) {
                match = false;
                break;
            }
        }

        if (match) {
            /* Found command - restart DMA for next command */
            restart_dma_rx();
            return true;
        }

        /* Move to next byte */
        rd++;
        avail--;
    }

    /* Update read position (consume non-matching bytes) */
    rx_read_pos = rd;

    /* If buffer is getting full, restart DMA */
    if (dma_remaining < COMMAND_LEN) {
        restart_dma_rx();
    }

    return false;
}

/**
 * @brief  Build CSV response in TX buffer
 * @retval Length of CSV data, or 0 if no data
 * @note   Formats CalcReport data into CSV rows in tx_buffer.
 */
static uint16_t build_csv_response(void) {
    char *ptr = tx_buffer;
    size_t remaining = TX_BUFFER_SIZE;
    int written;

    /* Write header row */
    size_t header_len = strlen(CSV_HEADER);
    if (header_len >= remaining) {
        return 0;
    }
    memcpy(ptr, CSV_HEADER, header_len);
    ptr += header_len;
    remaining -= header_len;

    /* Get report buffer info */
    CalcReport_t *buffer = calc_get_report_buffer();
    uint16_t count = calc_get_report_count();
    int16_t head = calc_get_report_head();

    /* If no reports, just return header */
    if (count == 0 || head < 0) {
        return (uint16_t)(ptr - tx_buffer);
    }

    /* Calculate starting index (oldest report) */
    /* Reports are stored: oldest at (head - count + 1), newest at head */
    for (uint16_t i = 0; i < count; i++) {
        /* Calculate index with wraparound */
        int16_t idx = (head - count + 1 + i + CALC_REPORT_BUFFER_SIZE) % CALC_REPORT_BUFFER_SIZE;
        CalcReport_t *report = &buffer[idx];

        /* Format row */
        written = snprintf(ptr, remaining,
            "%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
            timestamp(report->timestamp_s),
            report->latitude,
            report->longitude,
            report->u_mean,
            report->u_std,
            report->v_mean,
            report->v_std,
            report->w_mean,
            report->w_std,
            report->wind_speed_mean,
            report->wind_speed_std,
            report->wind_from_mean,
            report->wind_from_std,
            report->gust_mean,
            report->gust_std);

        if (written < 0 || (size_t)written >= remaining) {
            /* Buffer overflow - stop here */
            break;
        }

        ptr += written;
        remaining -= written;
    }

    return (uint16_t)(ptr - tx_buffer);
}

/**
 * @brief  Start DMA TX transfer
 * @param  length: Number of bytes to transmit
 */
static void start_dma_tx(uint16_t length) {
    /* Store length for reference */
    tx_length = length;

    /* Set state before enabling DMA */
    state = TELUS_SENDING;

    /* Disable channel before reconfiguration */
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_3);

    /* Set data length */
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_3, length);

    /* Clear any pending flags */
    LL_DMA_ClearFlag_TC3(DMA2);
    LL_DMA_ClearFlag_HT3(DMA2);
    LL_DMA_ClearFlag_TE3(DMA2);

    /* Enable channel to start transfer */
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_3);
}

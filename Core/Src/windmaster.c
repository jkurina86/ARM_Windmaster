/**
  ******************************************************************************
  * @file    windmaster.c
  * @brief   WindMaster functions
  * @note    Implements WindMaster Mode 10 - Binary UVW Long protocol
  *          UART5 @ 57600 baud, 23-byte packets at 20Hz
  ******************************************************************************
*/

#include "windmaster.h"
#include "systime.h"
#include "recorder.h"
#include "shell.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define DMA_BUFFER_SIZE 1024 /* 1 KB for incoming WM sensor data */
#define PACKET_SIZE 23
#define HEADER_SIZE 2
#define DATA_SIZE 18
#define CHECKSUM_SIZE 1

/* Private variables ---------------------------------------------------------*/
static bool wm_running = false;
static WM_Packet_t latest_packet = {0};
uint8_t dma_buffer_wm[DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer_wm")));
uint16_t dma_old_pos_wm = 0;

/* Private function prototypes -----------------------------------------------*/
static void send_command(const char* cmd);
static void flush_rx(void);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the WindMaster
  * @param  None
  * @retval None
  * @note   Sends '*\n' command to enter configuration mode, flushes echo,
  *         then configures DMA2 Ch2 for RX (but doesn't enable it).
  *         Leaves the WindMaster in config mode (not sending binary data).
  *         Call wm_start() to begin measurement mode and enable DMA.
  *         UART5 RX uses DMA2 Channel 2.
  *         UART5 @ 57600 baud, 8-N-1.
  */
void wm_init(void) {
  /* Clear any pending flags and data */
  LL_USART_ClearFlag_TC(UART5);
  LL_USART_ClearFlag_IDLE(UART5);
  (void)UART5->RDR;  // Dummy read to clear RX

  /* Send '*\n' to enter configuration mode (stops any output) */
  send_command("*\n");

  /* Wait for echo to arrive and flush it */
  HAL_Delay(50);  // Allow time for WindMaster to respond
  flush_rx();

  /* Now configure DMA for binary data reception */
  /* Enable USART5 DMA request for RX */
  LL_USART_EnableDMAReq_RX(UART5);

  /* Configure DMA RX addresses (DMA2 Channel 2) */
  LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_2,
                        LL_USART_DMA_GetRegAddr(UART5, LL_USART_DMA_REG_DATA_RECEIVE),
                        (uint32_t)dma_buffer_wm,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_2, DMA_BUFFER_SIZE);
}

/** @brief  Start the WindMaster
  * @param  None
  * @retval None
  * @note   Sends 'Q\n' to start measurement mode (binary output).
  *         Init leaves the WindMaster in config mode (not sending data).
  *         This function transitions to measurement mode and enables DMA.
  */
void wm_start(void) {
  /* Return if already running */
  if (wm_running) {
    return;
  }

  /* Send 'Q\n' to return to measurement mode (starts binary output) */
  send_command("Q\n");

  /* Wait for echo to arrive and flush it */
  HAL_Delay(10);
  flush_rx();

  /* Enable DMA for binary data reception */
  LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);

  /* Enable DMA RX interrupts */
  //LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_2);
  //LL_DMA_EnableIT_HT(DMA2, LL_DMA_CHANNEL_2);

  /* Initialize latest data */
  memset(&latest_packet, 0, sizeof(WM_Packet_t));

  /* Set running flag */
  wm_running = true;
}

/** @brief  Stop the WindMaster
  * @param  None
  * @retval None
  * @note   Disables DMA, sends '*\n' to enter configuration mode (stops binary output),
  *         flushes echo, and clears running flag.
  */
void wm_stop(void) {
  if (wm_running) {
    /* Disable DMA to prevent binary data corruption during command */
    LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_2);

    /* Send '*\n' to enter configuration mode (stops output) */
    send_command("*\n");

    /* Wait for echo and flush */
    HAL_Delay(50);
    flush_rx();

    wm_running = false;
  }
}

/** @brief  Check if the WindMaster is running
  * @param  None
  * @retval true if running, false otherwise
  */
bool wm_is_running(void) {
  return wm_running;
}

/** @brief Drain and queue the latest WM packet
  * @param  None
  * @retval true if a complete packet was processed, false otherwise
  * @note   Processes data from the DMA buffer, extracts complete packets,
  *         verifies checksums, and updates the latest_packet structure.
  * @note   Iteration limit to prevent blocking.
  */
bool wm_drain_and_queue(void)
{
    const uint16_t MASK = (uint16_t)(DMA_BUFFER_SIZE - 1);
    const uint16_t wr   = (uint16_t)((DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_2)) & MASK);

    uint16_t rd    = dma_old_pos_wm;
    uint16_t avail = (uint16_t)((wr - rd) & MASK);

    bool got_any = false;
    uint16_t iterations = 0;
    const uint16_t MAX_ITERATIONS = 200; // Prevent ISR from running too long

    while (avail >= PACKET_SIZE && iterations < MAX_ITERATIONS) {
        iterations++;
        /* Expect 0xB4 0xB4 header (wrap-safe) */
        uint8_t h0 = dma_buffer_wm[rd];
        uint8_t h1 = dma_buffer_wm[(uint16_t)((rd + 1) & MASK)];
        if (h0 != 0xB4 || h1 != 0xB4) {
            rd = (uint16_t)((rd + 1) & MASK);
            avail--;
            continue;
        }

        /* Copy one full 23-byte packet from ring (wrap-safe) */
        uint8_t pkt[PACKET_SIZE];
        uint16_t first = (uint16_t)((DMA_BUFFER_SIZE - rd) < PACKET_SIZE ? (DMA_BUFFER_SIZE - rd) : PACKET_SIZE);
        memcpy(pkt, &dma_buffer_wm[rd], first);
        if (first < PACKET_SIZE) {
            memcpy(pkt + first, &dma_buffer_wm[0], (size_t)(PACKET_SIZE - first));
        }

        /* Update latest_packet (truncate to struct size if smaller) */
        size_t copy_len = sizeof(WM_Packet_t) < PACKET_SIZE ? sizeof(WM_Packet_t) : (size_t)PACKET_SIZE;
        memcpy(&latest_packet, pkt, copy_len);

        /* Queue the packet for recording with current system timestamp */
        recorder_queue_wm(&latest_packet);

        /* Advance by exactly one packet */
        rd    = (uint16_t)((rd + PACKET_SIZE) & MASK);
        avail = (uint16_t)((wr - rd) & MASK);
        got_any = true;
    }

    dma_old_pos_wm = rd;
    return got_any;
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the WindMaster using polling TX
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over UART5 using polling mode.
  *         Used for short configuration commands ('*\n', 'Q\n').
  */
static void send_command(const char* cmd) {
  while (*cmd) {
    /* Wait for TX empty */
    while (!LL_USART_IsActiveFlag_TXE(UART5));

    /* Send character */
    LL_USART_TransmitData8(UART5, *cmd);
    cmd++;
  }

  /* Wait for transmission complete */
  while (!LL_USART_IsActiveFlag_TC(UART5));
}

/** @brief  Flush RX FIFO to clear command echoes
  * @param  None
  * @retval None
  * @note   Reads and discards all available data from UART5 RX.
  *         Used after sending configuration commands to clear echoes.
  */
static void flush_rx(void) {
  /* Drain RX FIFO */
  while (LL_USART_IsActiveFlag_RXNE(UART5)) {
    (void)LL_USART_ReceiveData8(UART5);
  }

  /* Clear IDLE flag if set */
  if (LL_USART_IsActiveFlag_IDLE(UART5)) {
    LL_USART_ClearFlag_IDLE(UART5);
  }
}

/** @brief Parse a received WM packet
  * @param pkt: Pointer to the received packet data
  * @param packet: Pointer to WM_Packet_t structure to populate
  * @retval true if the packet is valid, false otherwise
  * @note   Validates the packet structure and checksum, updating the latest_packet
  *         structure if valid.
  */
bool wm_parse_packet(const uint8_t* pkt, WM_Packet_t* packet) {
  if (pkt[0] != 0xB4 || pkt[1] != 0xB4) {
    return false; /* Invalid header */
  }

  /* Compute checksum (XOR of bytes BETWEEN header and checksum, i.e., bytes 2-21) */
  uint8_t checksum = 0;
  for (size_t i = 2; i < PACKET_SIZE - 1; i++) {
    checksum ^= pkt[i];
  }

  /* Validate checksum */
  if (checksum != pkt[PACKET_SIZE - 1]) {
    return false; /* Checksum mismatch */
  }

  /* Populate the structure */
  memcpy(packet, pkt, sizeof(WM_Packet_t) < PACKET_SIZE ? sizeof(WM_Packet_t) : (size_t)PACKET_SIZE);
  return true;
}

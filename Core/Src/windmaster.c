/**
  ******************************************************************************
  * @file    windmaster.c
  * @brief   WindMaster functions
  * @note    Implements WindMaster Mode 10 - Binary UVW Long protocol
  *          USART2 @ 57600 baud, 23-byte packets at 20Hz
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
#define DMA_BUFFER_SIZE     1024        /* 1 KB for incoming WM sensor data */
#define PACKET_SIZE         23          /* 23 Bytes for M10, 13 Bytes for M8 */
#define HEADER_SIZE         2
#define HEADER_VALUE        0xB4        /* 0xB4 for M10, 0xB2 for M8 */
#define DATA_SIZE           18          /* 18 Bytes for M10, 8 Bytes for M8 */
#define CHECKSUM_SIZE       1

/* Private variables ---------------------------------------------------------*/
static bool wm_running = false;
static WM_Packet_t latest_packet = {0};
uint8_t dma_buffer_wm[DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer_wm")));
uint16_t dma_old_pos_wm = 0;

/* Private function prototypes -----------------------------------------------*/
static void send_command(const char* cmd);
static void flush_rx(void);
bool wm_validate_packet(const uint8_t* pkt, WM_Packet_t* packet);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the WindMaster
  * @param  None
  * @retval None
  * @note   Sends '*\r' command to enter configuration mode, flushes echo,
  *         then configures DMA1 Ch6 for RX (but doesn't enable it).
  *         Leaves the WindMaster in config mode (not sending binary data).
  *         Call wm_start() to begin measurement mode and enable DMA.
  *         USART2 RX uses DMA1 Channel 6.
  *         USART2 @ 57600 baud, 8-N-1.
  */
void wm_init(void) {
  /* Clear any pending flags and data */
  LL_USART_ClearFlag_TC(USART2);
  LL_USART_ClearFlag_IDLE(USART2);
  (void)USART2->RDR;  // Dummy read to clear RX

  /* Send '*\r' to enter configuration mode (stops any output) */
  send_command("*\r");

  /* Wait for echo to arrive and flush it */
  HAL_Delay(10);
  flush_rx();

  /* Now configure DMA for binary data reception */
  /* Ensure DMA channel is disabled before touching counters/addresses */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);

  /* Enable USART2 DMA request for RX */
  LL_USART_EnableDMAReq_RX(USART2);

  /* Configure DMA RX addresses (DMA1 Channel 6) */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_6,
                        LL_USART_DMA_GetRegAddr(USART2, LL_USART_DMA_REG_DATA_RECEIVE),
                        (uint32_t)dma_buffer_wm,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, DMA_BUFFER_SIZE);
}

/** @brief  Start the WindMaster
  * @param  None
  * @retval None
  * @note   Sends 'Q\r' to start measurement mode (binary output).
  *         Flushes echo, advances DMA buffer position to skip initial <CR><LF>,
  *         enables DMA channel for data reception, and sets running flag.
  */
void wm_start(void) {
  /* Return if already running */
  if (wm_running) {
    return;
  }

  /* Send 'Q\r' to start measurement mode (binary output) and flush the echo */
  send_command("Q\r");
  HAL_Delay(1);
  flush_rx();

  /* Advance the DMA buffer position to skip the <CR><LF> it sends before the first measurement */
  dma_old_pos_wm = 2;

  /* Reset DMA transfer counter (must be done while channel is disabled) */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, DMA_BUFFER_SIZE);

  /* Enable DMA for binary data reception */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);

  /* Initialize latest data */
  memset(&latest_packet, 0, sizeof(WM_Packet_t));

  /* Set running flag */
  wm_running = true;
}

/** @brief  Stop the WindMaster
  * @param  None
  * @retval None
  * @note   Disables DMA, sends '*\r' to enter configuration mode (stops binary output),
  *         flushes echo, and clears running flag.
  */
void wm_stop(void) {
  if (wm_running) {
    /* Disable DMA to prevent binary data corruption during command */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);

    /* Send '*\r' to enter configuration mode (stops output) */
    send_command("*\r");

    /* Wait for echo and flush the RX FIFO */
    HAL_Delay(10);
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
  const uint16_t MASK = DMA_BUFFER_SIZE - 1;
  const uint16_t wr = (DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6)) & MASK;
  uint16_t rd = dma_old_pos_wm;
  uint16_t avail = (wr - rd) & MASK;
  bool packets_drained = false;
  uint16_t iterations = 0;
  const uint16_t MAX_ITERATIONS = 16;

  /* Loop through the DMA buffer if at least one full packet is available */
  while (avail >= PACKET_SIZE && iterations < MAX_ITERATIONS) {
    iterations++;
    
    /* Expect 2 byte header header */
    uint8_t h0 = dma_buffer_wm[rd];
    uint8_t h1 = dma_buffer_wm[(rd + 1) & MASK];
    
    /* If header bytes do not match expected values, advance by one and continue */
    if (h0 != HEADER_VALUE || h1 != HEADER_VALUE) {
      rd = (rd + 1) & MASK;
      avail--;
      continue;
    }

    /* Copy one full 23-byte packet from ring */
    uint8_t pkt[PACKET_SIZE];

    /* Copy packet from ring buffer and split if the packet wraps around the end */
    uint16_t head_len = DMA_BUFFER_SIZE - rd;
    if (head_len >= PACKET_SIZE) {
      memcpy(pkt, &dma_buffer_wm[rd], PACKET_SIZE);
    } else {
      memcpy(pkt, &dma_buffer_wm[rd], head_len);
      memcpy(pkt + head_len, &dma_buffer_wm[0], (size_t)(PACKET_SIZE - head_len));
    }

    /* Validate packet before queueing (XOR checksum check) */
    if (!wm_validate_packet(pkt, &latest_packet)) {
      /* Corrupted packet - skip and continue searching */
      rd = (rd + 1) & MASK;
      avail--;
      continue;
    }

    /* Queue the validated packet for recording with current system timestamp */
    recorder_queue_wm(&latest_packet);

    /* Advance by exactly one packet */
    rd = (rd + PACKET_SIZE) & MASK;
    avail = (wr - rd) & MASK;
    packets_drained = true;
  }

  dma_old_pos_wm = rd;
  return packets_drained;
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the WindMaster using polling TX
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over USART2 using polling mode.
  *         Used for short configuration commands ('*\r', 'Q\r').
  *         Includes 2ms inter-character delay to allow WindMaster to echo and process each byte.
  */
static void send_command(const char* cmd) {
  while (*cmd) {
    /* Wait for TX empty */
    while (!LL_USART_IsActiveFlag_TXE(USART2));

    /* Send character */
    LL_USART_TransmitData8(USART2, *cmd);

    /* Wait for transmission complete */
    while (!LL_USART_IsActiveFlag_TC(USART2));

    cmd++;
  }
}

/** @brief  Flush RX FIFO to clear command echoes
  * @param  None
  * @retval None
  * @note   Reads and discards all available data from USART2 RX.
  *         Used after sending configuration commands to clear echoes.
  */
static void flush_rx(void) {
  /* Drain RX FIFO */
  while (LL_USART_IsActiveFlag_RXNE(USART2)) {
    (void)LL_USART_ReceiveData8(USART2);
  }

  /* Clear IDLE flag if set */
  if (LL_USART_IsActiveFlag_IDLE(USART2)) {
    LL_USART_ClearFlag_IDLE(USART2);
  }
}

/** @brief Validate a received WM packet
  * @param pkt: Pointer to the received packet data
  * @param packet: Pointer to WM_Packet_t structure to populate
  * @retval true if the packet is valid, false otherwise
  * @note   Validates the packet structure and checksum, updating the latest_packet
  *         structure if valid.
  */
bool wm_validate_packet(const uint8_t* pkt, WM_Packet_t* packet) {
  /* Validate header */
  if (pkt[0] != HEADER_VALUE || pkt[1] != HEADER_VALUE) {
    return false;
  }

  /* Compute checksum (XOR of bytes BETWEEN header and checksum, i.e., bytes 2-21) */
  uint8_t checksum = 0;
  for (uint8_t i = 2; i < PACKET_SIZE - 1; i++) {
    checksum ^= pkt[i];
  }

  /* Validate checksum */
  if (checksum != pkt[PACKET_SIZE - 1]) {
    return false;
  }

  /* Populate the structure */
  memcpy(packet, pkt, PACKET_SIZE);
  return true;
}

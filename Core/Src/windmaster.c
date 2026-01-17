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
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define DMA_BUFFER_SIZE 1024 /* 1 KB for incoming WM sensor data */
#define PACKET_SIZE 23
#define HEADER_VALUE 0xB4

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
  */
void wm_init(void)
{
  /* Clear any pending flags and data */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
  __HAL_UART_CLEAR_IDLEFLAG(&huart2);
  (void)huart2.Instance->RDR;  /* Dummy read to clear RX */

  /* Send '*\r' to enter configuration mode (stops any output) */
  send_command("*\r");

  /* Wait for echo to arrive and flush it */
  HAL_Delay(10);
  flush_rx();

  /* Ensure DMA channel is disabled before configuration */
  __HAL_DMA_DISABLE(huart2.hdmarx);

  /* Enable USART2 DMA request for RX */
  SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);

  /* Configure DMA RX addresses (DMA1 Channel 6) */
  huart2.hdmarx->Instance->CPAR = (uint32_t)&huart2.Instance->RDR;
  huart2.hdmarx->Instance->CMAR = (uint32_t)dma_buffer_wm;
  huart2.hdmarx->Instance->CNDTR = DMA_BUFFER_SIZE;
}

/** @brief  Start the WindMaster
  * @param  None
  * @retval None
  */
void wm_start(void)
{
  if (wm_running) {
    return;
  }

  /* Send 'Q\r' to start measurement mode (binary output) and flush the echo */
  send_command("Q\r");
  HAL_Delay(1);
  flush_rx();

  /* Advance the DMA buffer position to skip the <CR><LF> it sends before the first measurement */
  dma_old_pos_wm = 2;

  /* Reset DMA (must be done while channel is disabled) */
  __HAL_DMA_DISABLE(huart2.hdmarx);
  huart2.hdmarx->Instance->CPAR = (uint32_t)&huart2.Instance->RDR;
  huart2.hdmarx->Instance->CMAR = (uint32_t)dma_buffer_wm;
  huart2.hdmarx->Instance->CNDTR = DMA_BUFFER_SIZE;

  /* Ensure USART2 DMA request is enabled */
  SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);

  /* Enable DMA for binary data reception */
  __HAL_DMA_ENABLE(huart2.hdmarx);

  /* Initialize latest data */
  memset(&latest_packet, 0, sizeof(WM_Packet_t));

  /* Set running flag */
  wm_running = true;
}

/** @brief  Stop the WindMaster
  * @param  None
  * @retval None
  */
void wm_stop(void)
{
  if (wm_running) {
    /* Disable DMA to prevent binary data corruption during command */
    __HAL_DMA_DISABLE(huart2.hdmarx);

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
bool wm_is_running(void)
{
  return wm_running;
}

/** @brief Drain and queue the latest WM packet
  * @param  None
  * @retval true if a complete packet was processed, false otherwise
  */
bool wm_drain_and_queue(void)
{
  const uint16_t MASK = DMA_BUFFER_SIZE - 1;
  const uint16_t wr = (DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx)) & MASK;
  uint16_t rd = dma_old_pos_wm;
  uint16_t avail = (wr - rd) & MASK;
  bool packets_drained = false;
  uint16_t iterations = 0;
  const uint16_t MAX_ITERATIONS = 16;

  /* Loop through the DMA buffer if at least one full packet is available */
  while (avail >= PACKET_SIZE && iterations < MAX_ITERATIONS) {
    iterations++;

    /* Expect 2 byte header */
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

    /* Update latest_packet (struct matches fixed packet size) */
    memcpy(&latest_packet, pkt, PACKET_SIZE);

    /* Queue the packet for recording with current system timestamp */
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
  * @note   Sends character by character with TXE/TC waits.
  */
static void send_command(const char* cmd)
{
  uint16_t len = strlen(cmd);
  HAL_UART_Transmit(&huart2, (uint8_t*)cmd, len, HAL_MAX_DELAY);
}

/** @brief  Flush RX FIFO to clear command echoes
  * @param  None
  * @retval None
  */
static void flush_rx(void)
{
  /* Drain RX FIFO */
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
    (void)huart2.Instance->RDR;
  }

  /* Clear IDLE flag if set */
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
  }
}

/** @brief  Validate a received WM packet
  * @param  pkt: Pointer to the start of the packet buffer
  * @param  packet: Pointer to WM_Packet_t structure to populate
  * @retval true if the packet is valid, false otherwise
  */
bool wm_validate_packet(const uint8_t* pkt, WM_Packet_t* packet)
{
  /* Validate header */
  if (pkt[0] != HEADER_VALUE || pkt[1] != HEADER_VALUE) {
    return false;
  }

  /* Compute checksum (XOR of bytes 2-21) */
  uint8_t checksum = 0;
  for (uint8_t i = 2; i < PACKET_SIZE - 1; i++) {
    checksum ^= pkt[i];
  }

  /* Validate checksum */
  if (checksum != pkt[PACKET_SIZE - 1]) {
    return false;
  }

  memcpy(packet, pkt, PACKET_SIZE);
  return true;
}

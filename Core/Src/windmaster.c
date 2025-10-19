/**
  ******************************************************************************
  * @file    windmaster.c
  * @brief   WindMaster functions
  * @note    Placeholder for WindMaster functionality
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
#define DMA_BUFFER_SIZE 4096 /* 4 KB for WM sensor data */
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

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the WindMaster
  * @param  None
  * @retval None
  * @note   Sets up the UART and DMA for receiving data, configuring
  *         hardware peripherals and preparing the DMA buffer for data reception.
  */
void wm_init(void) {
  /* Clear any pending flags and data */
  LL_USART_ClearFlag_TC(UART4);
  LL_USART_ClearFlag_IDLE(UART4);
  (void)UART4->RDR;  // Dummy read to clear RX
  
  /* Configure UART4 for DMA RX (same as UART5 - don't disable) */
  LL_USART_EnableDMAReq_RX(UART4);

  /* Configure DMA addresses */
  LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_5,
                        LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_RECEIVE),
                        (uint32_t)dma_buffer_wm,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_5, DMA_BUFFER_SIZE);

  /* Enable DMA interrupts */
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_HT(DMA2, LL_DMA_CHANNEL_5);

  /* Start DMA reception */
  LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_5);

  /* Initialize latest data */
  memset(&latest_packet, 0, sizeof(WM_Packet_t));
}

/** @brief  Start the WindMaster
  * @param  None
  * @retval None
  * @note   Sends the START command to the WindMaster
  */
void wm_start(void) {
  if (!wm_running) {
    send_command("\n"); /* Wake up the Python script for dummy sensor. */
    send_command("START\n");
    wm_running = true;
  }
}

/** @brief  Stop the WindMaster
  * @param  None
  * @retval None
  * @note   Sends the STOP command to the WindMaster
  */
void wm_stop(void) {
  if (wm_running) {
    send_command("\n");
    send_command("STOP\n");
    wm_running = false;
  }
}

/** @brief  Check if the dummy WindMaster is running
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
  */
bool wm_drain_and_queue(void)
{
    const uint16_t MASK = (uint16_t)(DMA_BUFFER_SIZE - 1);
    const uint16_t wr   = (uint16_t)((DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_5)) & MASK);

    uint16_t rd    = dma_old_pos_wm;
    uint16_t avail = (uint16_t)((wr - rd) & MASK);

    bool got_any = false;

    while (avail >= PACKET_SIZE) {
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

        /* Get timestamp and queue the packet for recording */
        uint64_t timestamp_us = time_us_now();
        recorder_queue_wm(&latest_packet, timestamp_us);

        /* Advance by exactly one packet */
        rd    = (uint16_t)((rd + PACKET_SIZE) & MASK);
        avail = (uint16_t)((wr - rd) & MASK);
        got_any = true;
    }

    dma_old_pos_wm = rd;
    return got_any;
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the dummy WindMaster
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over UART4 to control the dummy WindMaster.
  */
static void send_command(const char* cmd) {
    /* Simple transmission - same as working test code */
    while (*cmd) {
        while (!LL_USART_IsActiveFlag_TXE(UART4));
        LL_USART_TransmitData8(UART4, *cmd++);
    }
    while (!LL_USART_IsActiveFlag_TC(UART4));
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
/**
  ******************************************************************************
  * @file    vectornav.c
  * @brief   VectorNav GPS/IMU functions
  * @note    Placeholder for IMU functionality
  ******************************************************************************
  */

#include "vectornav.h"
#include "systime.h"
#include "recorder.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
#define DMA_BUFFER_SIZE 16384 /* 16 KB for IMU sensor data */
#define PACKET_SIZE 86

/* Private variables ---------------------------------------------------------*/
static bool imu_running = false;
static VN_Packet_t latest_packet = {0};
uint8_t dma_buffer_imu[DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer_imu")));
uint16_t dma_old_pos_imu = 0;

/* Private function prototypes -----------------------------------------------*/
static void send_command(const char* cmd);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the VectorNav
  * @param  None
  * @retval None
  * @note   Sets up the UART and DMA for receiving data, configuring
  *         hardware peripherals and preparing the DMA buffer for data reception.
  *         Ensures async mode is disabled before starting DMA to avoid buffer corruption.
  */
void vn_init(void) {
    /* Flush UART RX buffer before sending command (clear any pending data) */
    while (LL_USART_IsActiveFlag_RXNE(UART5)) {
        (void)LL_USART_ReceiveData8(UART5);
    }

    /* Disable async mode in case it was left on from previous power cycle */
    send_command("$VNWRG,75,0,20,00*F819\r\n");

    /* Flush the ASCII echo response from VectorNav (e.g., "$VNWRG,75,0,20,00*F819\r\n") */
    /* Wait briefly for response to arrive (VN-300 responds quickly) */
    for (volatile uint32_t i = 0; i < 100000; i++); /* ~10ms delay at 80MHz */

    while (LL_USART_IsActiveFlag_RXNE(UART5)) {
        (void)LL_USART_ReceiveData8(UART5);
    }

    /* Configure UART5 for DMA RX (don't disable UART to preserve TX functionality) */
    LL_USART_EnableDMAReq_RX(UART5);

    /* Configure DMA addresses */
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_2,
                          LL_USART_DMA_GetRegAddr(UART5, LL_USART_DMA_REG_DATA_RECEIVE),
                          (uint32_t)dma_buffer_imu,
                          LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_2, DMA_BUFFER_SIZE);

    /* Enable DMA interrupts */
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_HT(DMA2, LL_DMA_CHANNEL_2);

    /* Start DMA reception */
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);

    /* Initialize latest packet */
    memset(&latest_packet, 0, sizeof(VN_Packet_t));
}

/** @brief  Start the vectornav
  * @param  None
  * @retval None
  * @note   Sends the START command to the vectornav.
  */
void vn_start(void) {
    if (imu_running == false) {
        send_command("\n"); /* Wake up the Python script for dummy sensor. */
        send_command("START\n");
        imu_running = true;
    }
}

/** @brief  Stop the dummy IMU
  * @param  None
  * @retval None
  * @note   Sends the STOP command to the dummy IMU.
  */
void vn_stop(void) {
  if (imu_running == true) {
    send_command("\n");
    send_command("STOP\n");
    imu_running = false;
  }
}

/** @brief  Get the state of the dummy IMU
  * @param  None
  * @retval bool: true if running, false otherwise
  */
bool vn_is_running(void) {
  return imu_running;
}

/** @brief  Drain and queue the latest VN packet
  * @param  None
  * @retval true if a valid packet was processed, false otherwise
  * @note   Processes the DMA buffer to extract complete VN packets,
  *         updating the latest_packet structure with the most recent valid data.
  * @note   ISR-safe with iteration limit to prevent blocking
  */
bool vn_drain_and_queue(void) {
  const uint16_t WR = (uint16_t)((DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_2)) & (DMA_BUFFER_SIZE - 1));
  const uint16_t MASK = (uint16_t)(DMA_BUFFER_SIZE - 1);

  /* Compute the number of bytes between RD and WR */
  uint16_t rd = dma_old_pos_imu;
  uint16_t avail = (uint16_t)((WR - rd) & MASK);

  bool rx_any = false;
  uint16_t iterations = 0;
  const uint16_t MAX_ITERATIONS = 200;

  while (avail >= PACKET_SIZE && iterations < MAX_ITERATIONS) {
    iterations++;
    /* Start of Packet Check */
    if (dma_buffer_imu[rd] != 0xFA) {
      rd = (uint16_t)((rd + 1) & MASK);
      avail--;
      continue;
    }

    /* Copy one full packet into a linear temporary buffer */
    uint8_t tmp[PACKET_SIZE];
    uint16_t first = (uint16_t)(DMA_BUFFER_SIZE - rd) < PACKET_SIZE ? (uint16_t)(DMA_BUFFER_SIZE - rd) : PACKET_SIZE;
    memcpy(tmp, &dma_buffer_imu[rd], first);
    if (first < PACKET_SIZE) {
      memcpy(tmp + first, &dma_buffer_imu[0], (size_t)(PACKET_SIZE - first));
    }

    /* Update the latest packet */
    size_t copy_len = sizeof(VN_Packet_t) < PACKET_SIZE ? sizeof(VN_Packet_t) : (size_t)PACKET_SIZE;
    memcpy(&latest_packet, tmp, copy_len);

    /* Get timestamp and queue the packet for recording */
    uint64_t timestamp_us = time_us_now();
    recorder_queue_vn(&latest_packet, timestamp_us);

    /* Advance the read pointer */
    rd = (uint16_t)((rd + PACKET_SIZE) & MASK);
    avail = (uint16_t)((WR - rd) & MASK);

    rx_any = true;
  }

  dma_old_pos_imu = rd;
  return rx_any;
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the vectornav
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over UART5 to control the vectornav.
  * @note   IDLE interrupts are never enabled in polling architecture
  */
static void send_command(const char* cmd) {
  while (*cmd) {
    while (!LL_USART_IsActiveFlag_TXE(UART5));
    LL_USART_TransmitData8(UART5, *cmd++);
  }

  /* Wait for transmission to complete */
  while (!LL_USART_IsActiveFlag_TC(UART5));

  /* Clear any spurious IDLE flag that might have been set during TX */
  if (LL_USART_IsActiveFlag_IDLE(UART5)) {
    LL_USART_ClearFlag_IDLE(UART5);
    (void)UART5->RDR;  // Dummy read to clear flag
  }
}

/** @brief  Parse a received VN packet
  * @param  pkt: Pointer to the start of the packet buffer
  * @param  struct: Pointer to VN_Packet_t structure to populate
  * @retval true if the packet is valid, false otherwise
  */
bool vn_parse_packet(uint8_t* buffer_start, uint16_t length, VN_Packet_t* packet) {
  if (length < PACKET_SIZE || buffer_start[0] != 0xFA) {
    return false; // Invalid packet
  }

  /* Compute checksum */
  uint16_t checksum = 0;
  for (size_t i = 0; i < PACKET_SIZE - 2; i++) {
    checksum += buffer_start[i];
  }

  /* Validate checksum (Big Endian) */
  uint16_t received_checksum = (uint16_t)(buffer_start[PACKET_SIZE - 2] << 8 | buffer_start[PACKET_SIZE - 1]);
  if (checksum != received_checksum) {
    return false; // Checksum mismatch
  }
  
  memcpy(packet, buffer_start, PACKET_SIZE);
  return true;
}
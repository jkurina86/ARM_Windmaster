/**
  ******************************************************************************
  * @file    vectornav.c
  * @brief   VectorNav GPS/IMU functions
  * @note    Handles communication with the VectorNav VN-300 IMU via UART and DMA
  ******************************************************************************
  */

#include "vectornav.h"
#include "systime.h"
#include "recorder.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_hal.h"
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

/* TX DMA buffer for commands - must be in DMA-accessible RAM */
static uint8_t vn_tx_buffer[256];
volatile uint8_t vn_tx_complete = 1;  /* Signal from DMA ISR when TX complete */

/* Private function prototypes -----------------------------------------------*/
static void send_command(const char* cmd);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the VectorNav
  * @param  None
  * @retval None
  * @note   Sets up the UART and DMA for receiving data, configuring
  *         hardware peripherals and preparing the DMA buffer for data reception.
  *         Ensures async mode is disabled before starting DMA to avoid buffer corruption.
  *         USART3 RX uses DMA1 Channel 3, TX uses DMA1 Channel 2.
  */
void vn_init(void) {
    /* Flush UART RX buffer before sending command (clear any pending data) */
    while (LL_USART_IsActiveFlag_RXNE(USART3)) {
        (void)LL_USART_ReceiveData8(USART3);
    }

    /* Disable async mode in case it was left on from previous power cycle */
    send_command("$VNWRG,75,0,20,00*F819\n");

    /* Wait briefly for response to arrive (VN-300 responds quickly) */
    HAL_Delay(10);

    /* Flush the ASCII echo response from VectorNav */
    while (LL_USART_IsActiveFlag_RXNE(USART3)) {
        (void)LL_USART_ReceiveData8(USART3);
    }

    /* Configure USART3 for DMA RX and TX */
    LL_USART_EnableDMAReq_RX(USART3);
    LL_USART_EnableDMAReq_TX(USART3);

    /* Configure DMA RX addresses (DMA1 Channel 3) */
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
                          LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE),
                          (uint32_t)dma_buffer_imu,
                          LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, DMA_BUFFER_SIZE);

    /* Enable DMA RX interrupts */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_3);

    /* Start DMA RX reception */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

    /* Configure DMA TX (DMA1 Channel 2) - will be used on-demand by send_command */
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_MEDIUM);

    /* Initialize latest packet */
    memset(&latest_packet, 0, sizeof(VN_Packet_t));

    /* TX complete flag initialized to ready */
    vn_tx_complete = 1;
}

/** @brief  Start the vectornav
  * @param  None
  * @retval None
  * @note   Enables VectorNav async mode for 20Hz binary output.
  *         Disables DMA during command transmission to prevent ASCII echo from polluting binary data buffer.
  */
void vn_start(void) {
    if (imu_running == false) {
        /* Disable DMA to prevent ASCII echo from entering the binary data buffer */
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

        /* Flush UART RX buffer before sending command */
        while (LL_USART_IsActiveFlag_RXNE(USART3)) {
            (void)LL_USART_ReceiveData8(USART3);
        }

        /* Enable async mode (20Hz binary output) */
        send_command("$VNWRG,75,1,20,01,01EA*6441\n");

        /* Wait for response */
        HAL_Delay(10);

        /* Flush the ASCII echo response */
        while (LL_USART_IsActiveFlag_RXNE(USART3)) {
            (void)LL_USART_ReceiveData8(USART3);
        }

        /* Reset DMA buffer position tracking */
        dma_old_pos_imu = 0;

        /* Re-enable DMA to capture binary data stream */
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, DMA_BUFFER_SIZE);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

        imu_running = true;
    }
}

/** @brief  Stop the VectorNav
  * @param  None
  * @retval None
  * @note   Disables VectorNav async mode to stop binary output.
  *         Disables DMA during command transmission to prevent ASCII echo from polluting binary data buffer.
  */
void vn_stop(void) {
  if (imu_running == true) {
    /* Disable DMA to prevent ASCII echo from entering the binary data buffer */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

    /* Flush UART RX buffer before sending command */
    while (LL_USART_IsActiveFlag_RXNE(USART3)) {
        (void)LL_USART_ReceiveData8(USART3);
    }

    /* Disable async mode */
    send_command("$VNWRG,75,0,20,00*F819\n");

    /* Wait for response */
    HAL_Delay(10);

    /* Flush the ASCII echo response */
    while (LL_USART_IsActiveFlag_RXNE(USART3)) {
        (void)LL_USART_ReceiveData8(USART3);
    }

    /* Reset DMA buffer position tracking */
    dma_old_pos_imu = 0;

    /* Re-enable DMA (in case we start again later) */
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, DMA_BUFFER_SIZE);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

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
  */
bool vn_drain_and_queue(void) {
  const uint16_t WR = (uint16_t)((DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3)) & (DMA_BUFFER_SIZE - 1));
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
    uint32_t timestamp_ms = time_ms_now();
    recorder_queue_vn(&latest_packet, timestamp_ms);

    /* Advance the read pointer */
    rd = (uint16_t)((rd + PACKET_SIZE) & MASK);
    avail = (uint16_t)((WR - rd) & MASK);

    rx_any = true;
  }

  dma_old_pos_imu = rd;
  return rx_any;
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the vectornav using DMA TX
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over USART3 using DMA1 Channel 2.
  *         Waits for previous TX to complete before starting new transfer.
  *         Function blocks until DMA transfer initiates (non-blocking transfer itself).
  */
static void send_command(const char* cmd) {
  /* Wait for previous TX to complete, ensures synchronization */
  while (!vn_tx_complete);

  /* Copy command to DMA buffer */
  uint16_t len = 0;
  while (cmd[len] && len < (sizeof(vn_tx_buffer) - 1)) {
    vn_tx_buffer[len] = (uint8_t)cmd[len];
    len++;
  }

  if (len == 0) return;  /* Empty command */

  /* Mark TX as in-progress */
  vn_tx_complete = 0;

  /* Disable DMA before reconfiguring */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

  /* Configure DMA addresses and length */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
                        (uint32_t)vn_tx_buffer,
                        LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT),
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, len);

  /* Clear any pending TC flag before re-enabling */
  LL_DMA_ClearFlag_TC2(DMA1);

  /* Enable DMA Transfer Complete interrupt to signal completion */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);

  /* Enable DMA channel to start transfer */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
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
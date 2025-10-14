/**
  ******************************************************************************
  * @file    vectornav.c
  * @brief   VectorNav GPS/IMU functions
  * @note    Placeholder for IMU functionality
  ******************************************************************************
  */

#include "vectornav.h"
#include "systime.h"
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
static bool parse_packet(uint8_t* buffer_start, uint16_t length, VN_Packet_t* packet);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the VectorNav
  * @param  None
  * @retval None
  * @note   Sets up the UART and DMA for receiving data, configuring
  *         hardware peripherals and preparing the DMA buffer for data reception.
  */
void vn_init(void) {
    /* Configure UART5 for DMA RX */
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

    /* Enable UART5 RX */
    LL_USART_EnableDirectionRx(UART5);
    LL_USART_Enable(UART5);

    /* Initialize latest packet */
    memset(&latest_packet, 0, sizeof(VN_Packet_t));
}

/** @brief  Start the vectornav
  * @param  None
  * @retval None
  * @note   Sends the START command to the vectornav.
  */
void vn_start(uint64_t *start_time) {
    if (imu_running == false) {
        send_command("\n"); /* Wake up the Python script for dummy sensor. */
        send_command("START\n");
        imu_running = true;
    }
    *start_time = time_us_now();
}

/** @brief  Stop the dummy IMU
  * @param  None
  * @retval None
  * @note   Sends the STOP command to the dummy IMU.
  */
void vn_stop(uint64_t *stop_time) {
    if (imu_running == true) {
        send_command("STOP\n");
        imu_running = false;
    }
    *stop_time = time_us_now();
}

/** @brief  Get the state of the dummy IMU
  * @param  None
  * @retval bool: true if running, false otherwise
  */
bool vn_is_running(void) {
    return imu_running;
}

/** @brief  Get the latest IMU data
  * @param  data: Pointer to VN_Packet_t structure to fill with latest data
  * @retval None
  * @note   Parses the DMA buffer for new packets and updates the provided data structure.
  */
void vn_get_data(VN_Packet_t *data) {
    if (data) {
        /* Get current DMA position */
        uint16_t dma_pos = DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_2);

        /* Check for new data */
        uint16_t bytes_available = (dma_pos >= dma_old_pos_imu) ? (dma_pos - dma_old_pos_imu) : (DMA_BUFFER_SIZE - dma_old_pos_imu + dma_pos);

        if (bytes_available >= PACKET_SIZE) {
            /* Look for packet header in the buffer */
            for (uint16_t i = 0; i <= bytes_available - PACKET_SIZE; i++) {
                uint16_t check_pos = (dma_old_pos_imu + i) % DMA_BUFFER_SIZE;
                if (dma_buffer_imu[check_pos] == 0xFA) {
                    /* Found header, try to parse the packet */
                    if (parse_packet(&dma_buffer_imu[check_pos], PACKET_SIZE, data)) {
                        dma_old_pos_imu = (check_pos + PACKET_SIZE) % DMA_BUFFER_SIZE;
                        break;
                    }
                }
            }
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the vectornav
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over UART5 to control the vectornav.
  */
static void send_command(const char* cmd) {
    while (*cmd) {
        while (!LL_USART_IsActiveFlag_TXE(UART5));
        LL_USART_TransmitData8(UART5, *cmd++);
    }
}

/** @brief  Parse a VN packet from the DMA buffer
  * @param  buffer_start: Pointer to the start of the packet in the DMA buffer
  * @param  length: Length of the data available from buffer_start
  * @param  packet: Pointer to VN_Packet_t structure to fill with parsed data
  * @retval true if a valid packet was parsed, false otherwise
  * @note   Validates and extracts data from a VN packet.
  */
static bool parse_packet(uint8_t* buffer_start, uint16_t length, VN_Packet_t* packet) {
    if (length < PACKET_SIZE) return false;

    if (buffer_start[0] != 0xFA) return false;

    /* Copy packet data */
    memcpy(packet, buffer_start, PACKET_SIZE);

    /* Checksum is big-endian, swap bytes */
    packet->checksum = (packet->checksum >> 8) | (packet->checksum << 8);

    /* TODO: Implement CRC check as per protocol */

    return true;
}
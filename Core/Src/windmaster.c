/**
  ******************************************************************************
  * @file    windmaster.c
  * @brief   WindMaster functions
  * @note    Placeholder for WindMaster functionality
  ******************************************************************************
*/

#include "windmaster.h"
#include "systime.h"
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
bool parse_packet(uint8_t* buffer, uint16_t length, WM_Packet_t* data);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the WindMaster
  * @param  None
  * @retval None
  * @note   Sets up the UART and DMA for receiving data, configuring
  *         hardware peripherals and preparing the DMA buffer for data reception.
  */
void wm_init(void) {
    /* Configure UART4 for DMA RX */
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

    /* Enable UART4 RX */
    LL_USART_EnableDirectionRx(UART4);
    LL_USART_Enable(UART4);

    /* Initialize latest data */
    memset(&latest_packet, 0, sizeof(WM_Packet_t));
}

/** @brief  Start the WindMaster
  * @param  start_time: Pointer to store the start time in microseconds
  * @retval None
  * @note   Sends the START command to the WindMaster and records the start time.
  */
void wm_start(uint64_t *start_time) {
    if (!wm_running) {
        send_command("\n"); /* Wake up the Python script for dummy sensor. */
        send_command("START\n");
        wm_running = true;
    }
    *start_time = time_us_now();
}

/** @brief  Stop the WindMaster
  * @param  stop_time: Pointer to store the stop time in microseconds
  * @retval None
  * @note   Sends the STOP command to the WindMaster and records the stop time.
  */
void wm_stop(uint64_t *stop_time) {
    if (wm_running) {
        send_command("STOP\n");
        wm_running = false;
    }
    *stop_time = time_us_now();
}

/** @brief  Check if the dummy WindMaster is running
  * @param  None
  * @retval true if running, false otherwise
  */
bool wm_is_running(void) {
    return wm_running;
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the dummy WindMaster
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over UART4 to control the dummy WindMaster.
  */
static void send_command(const char* cmd) {
    while (*cmd) {
        while (!LL_USART_IsActiveFlag_TXE(UART4));
        LL_USART_TransmitData8(UART4, *cmd++);
    }
}

/** @brief  Parse a WindMaster data packet from the DMA buffer
  * @param  buffer: Pointer to the start of the packet in the DMA buffer
  * @param  length: Length of the data available from buffer
  * @param  data: Pointer to WM_Data_t structure to fill with parsed data
  * @retval true if a valid packet was parsed, false otherwise
  * @note   Validates and extracts data from a WindMaster packet.
  */
bool parse_wm_packet(uint8_t* buffer_start, uint16_t length, WM_Packet_t* data) {
    if (length < PACKET_SIZE) return false;

    // Copy packet data handling circular buffer wrap-around
    uint8_t packet[PACKET_SIZE];
    for (int i = 0; i < PACKET_SIZE; i++) {
        packet[i] = dma_buffer_wm[(buffer_start - dma_buffer_wm + i) % DMA_BUFFER_SIZE];
    }

    // Check header
    if (packet[0] != 0xB4 || packet[1] != 0xB4) {
        return false;
    }

    // Extract data (little-endian)
    data->header = (uint16_t)(packet[0] | (packet[1] << 8));
    data->status = (int16_t)(packet[2] | (packet[3] << 8));
    data->U_axis_speed = (int16_t)(packet[4] | (packet[5] << 8));
    data->V_axis_speed = (int16_t)(packet[6] | (packet[7] << 8));
    data->W_axis_speed = (int16_t)(packet[8] | (packet[9] << 8));
    data->SoS = (int16_t)(packet[10] | (packet[11] << 8));
    data->A1 = (int16_t)(packet[12] | (packet[13] << 8));
    data->A2 = (int16_t)(packet[14] | (packet[15] << 8));
    data->A3 = (int16_t)(packet[16] | (packet[17] << 8));
    data->A4 = (int16_t)(packet[18] | (packet[19] << 8));
    data->Temp = (int16_t)(packet[20] | (packet[21] << 8));
    data->checksum = packet[22];

    // Verify checksum
    uint8_t checksum = 0;
    for (int i = 2; i < PACKET_SIZE - 1; i++) {
        checksum ^= packet[i];
    }
    if (checksum != packet[PACKET_SIZE - 1]) {
        return false;
    }

    return true;
}
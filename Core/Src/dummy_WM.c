/**
  ******************************************************************************
  * @file    dummy_WM.c
  * @brief   Dummy WindMaster functions
  * @note    Placeholder for WindMaster functionality
  ******************************************************************************
  */

#include "dummy_WM.h"
#include "systime.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define DMA_BUFFER_SIZE 16384 /* 16 KB for WM sensor data */
#define PACKET_SIZE 23
#define TEST_PACKET_SIZE 10 /* 10 bytes for test packet: 2 header bytes + 4 ASCII identifier bytes + 4 packet number bytes */
#define PACKET_SIZE_WITH_TIMESTAMP (PACKET_SIZE + 8) /* 23 bytes data + 8 bytes timestamp */
#define HEADER_SIZE 2
#define DATA_SIZE 18
#define CHECKSUM_SIZE 1

/* Private variables ---------------------------------------------------------*/
static bool wm_running = false;
static WM_Data_t latest_data = {0};
uint8_t dma_buffer_wm[DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer_wm")));
uint16_t dma_old_pos_wm = 0;
static char log_buffer[256];

/* Private function prototypes -----------------------------------------------*/
static void send_command(const char* cmd);
static bool parse_packet(uint8_t* buffer, uint16_t length, WM_Data_t* data);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the dummy WindMaster
  * @param  None
  * @retval None
  * @note   Sets up the UART and DMA for receiving data, configuring
  *         hardware peripherals and preparing the DMA buffer for data reception.
  */
void dummy_WM_init(void) {
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
    memset(&latest_data, 0, sizeof(WM_Data_t));
}

/** @brief  Start the dummy WindMaster
  * @param  start_time: Pointer to store the start time in microseconds
  * @retval None
  * @note   Sends the START command to the dummy WindMaster and records the start time.
  */
void dummy_WM_start(uint64_t *start_time) {
    if (!wm_running) {
        send_command("\n"); /* Wake up the Python script for dummy sensor. */
        send_command("START\n");
        wm_running = true;
    }
    *start_time = time_us_now();
}

/** @brief  Stop the dummy WindMaster
  * @param  stop_time: Pointer to store the stop time in microseconds
  * @retval None
  * @note   Sends the STOP command to the dummy WindMaster and records the stop time.
  */
void dummy_WM_stop(uint64_t *stop_time) {
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
bool dummy_WM_is_running(void) {
    return wm_running;
}

/** @brief  Get the latest WindMaster data
  * @param  data: Pointer to WM_Data_t structure to fill with latest data
  * @retval None
  * @note   Parses the DMA buffer for new packets and updates the provided data structure.
  */
void dummy_WM_get_data(WM_Data_t* data) {
    if (data) {
        /* Get current DMA position */
        uint16_t dma_pos = DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_5);

        /* Check for new data */
        uint16_t bytes_available = (dma_pos >= dma_old_pos_wm) ? (dma_pos - dma_old_pos_wm) : (DMA_BUFFER_SIZE - dma_old_pos_wm + dma_pos);

        if (bytes_available >= PACKET_SIZE) {
            /* Look for packet header in the buffer */
            for (uint16_t i = 0; i <= bytes_available - PACKET_SIZE; i++) {
                uint16_t check_pos = (dma_old_pos_wm + i) % DMA_BUFFER_SIZE;
                if (dma_buffer_wm[check_pos] == 0xB4 &&
                    dma_buffer_wm[(check_pos + 1) % DMA_BUFFER_SIZE] == 0xB4) {
                    /* Found header, try to parse the packet */
                    if (parse_packet(&dma_buffer_wm[check_pos], PACKET_SIZE, &latest_data)) {
                        dma_old_pos_wm = (check_pos + PACKET_SIZE) % DMA_BUFFER_SIZE;
                        break;
                    }
                }
            }
        }

        memcpy(data, &latest_data, sizeof(WM_Data_t));
    }
}

/** @brief  Get the latest WindMaster test data
  * @param  test: Pointer to WM_Test_t structure to fill with latest test data
  * @retval None
  * @note   Parses the DMA buffer for new test packets and updates the provided test structure.
  */
void dummy_WM_get_test_data(WM_Test_t* test) {
    if (test) {
        /* Get current DMA position */
        uint16_t dma_pos = DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_5);

        /* Check for new data */
        uint16_t bytes_available = (dma_pos >= dma_old_pos_wm) ? (dma_pos - dma_old_pos_wm) : (DMA_BUFFER_SIZE - dma_old_pos_wm + dma_pos);

        if (bytes_available >= TEST_PACKET_SIZE) {
            /* Look for test packet header in the buffer */
            for (uint16_t i = 0; i <= bytes_available - TEST_PACKET_SIZE; i++) {
                uint16_t check_pos = (dma_old_pos_wm + i) % DMA_BUFFER_SIZE;
                if (dma_buffer_wm[check_pos] == 0xB4 &&
                    dma_buffer_wm[(check_pos + 1) % DMA_BUFFER_SIZE] == 0xB4) {
                    /* Found header, try to parse the test packet */
                    if (parse_packet_test(&dma_buffer_wm[check_pos], TEST_PACKET_SIZE, test)) {
                        dma_old_pos_wm = (check_pos + TEST_PACKET_SIZE) % DMA_BUFFER_SIZE;
                        break;
                    }
                }
            }
        }
    }
}

/** @brief  Log the latest WindMaster data as a formatted string
  * @param  data: Pointer to WM_Data_t structure containing the data to log
  * @retval const char*: Pointer to a static buffer containing the formatted log string
  * @note   Formats the WindMaster data into a human-readable string for logging purposes.
  */
const char* dummy_WM_log(WM_Data_t* data) {
    if (data) {
        snprintf(log_buffer, sizeof(log_buffer),
                "WM Data: Status=%d, U=%d, V=%d, W=%d, SoS=%d, A1=%d, A2=%d, A3=%d, A4=%d, Temp=%d",
                data->status, data->U_axis_speed, data->V_axis_speed, data->W_axis_speed,
                data->SoS, data->A1, data->A2, data->A3, data->A4, data->Temp);
    } else {
        snprintf(log_buffer, sizeof(log_buffer), "WM Data: No data available");
    }
    return log_buffer;
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
bool parse_packet(uint8_t* buffer_start, uint16_t length, WM_Data_t* data) {
    if (length < PACKET_SIZE) return false;

    /* Copy packet data handling circular buffer wrap-around */
    uint8_t packet[PACKET_SIZE];
    for (int i = 0; i < PACKET_SIZE; i++) {
        packet[i] = dma_buffer_wm[(buffer_start - dma_buffer_wm + i) % DMA_BUFFER_SIZE];
    }

    /* Check header */
    if (packet[0] != 0xB4 || packet[1] != 0xB4) {
        return false;
    }

    /* Extract data (little-endian) */
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

    /* Verify checksum */
    uint8_t checksum = 0;
    for (int i = 0; i < PACKET_SIZE - 1; i++) {
        checksum ^= packet[i];
    }
    if (checksum != packet[PACKET_SIZE - 1]) {
        return false;
    }

    return true;
}

/** @brief  Parse a WindMaster test packet from the DMA buffer
  * @param  buffer: Pointer to the start of the packet in the DMA buffer
  * @param  length: Length of the data available from buffer
  * @param  test: Pointer to WM_Test_t structure to fill with parsed test data
  * @retval true if a valid test packet was parsed, false otherwise
  * @note   Validates and extracts data from a WindMaster test packet.
  */
bool parse_packet_test(uint8_t* buffer_start, uint16_t length, WM_Test_t* test) {
    if (length < TEST_PACKET_SIZE) return false;

    /* Copy packet data handling circular buffer wrap-around */
    uint8_t packet[TEST_PACKET_SIZE];
    for (int i = 0; i < TEST_PACKET_SIZE; i++) {
        packet[i] = dma_buffer_wm[(buffer_start - dma_buffer_wm + i) % DMA_BUFFER_SIZE];
    }

    /* Check header */
    if (packet[0] != 0xB4 || packet[1] != 0xB4) {
        return false;
    }

    /* Extract test data */
    test->identifier[0] = packet[2];
    test->identifier[1] = packet[3];
    test->identifier[2] = packet[4];
    test->identifier[3] = packet[5];
    test->identifier[4] = '\0'; /* Null-terminate the string */
    test->packet_number = (uint32_t)(packet[6] | (packet[7] << 8) | (packet[8] << 16) | (packet[9] << 24));

    return true;
}

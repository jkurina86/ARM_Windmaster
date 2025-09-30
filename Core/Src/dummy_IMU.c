/**
  ******************************************************************************
  * @file    dummy_IMU.c
  * @brief   Dummy IMU functions
  * @note    Placeholder for IMU functionality
  ******************************************************************************
  */

#include "dummy_IMU.h"
#include "systime.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define DMA_BUFFER_SIZE 16384 /* 16 KB for IMU sensor data */
#define TEST_PACKET_SIZE 11 /* 11 bytes for test packet: 2 header bytes + 5 ASCII identifier bytes + 4 packet number bytes + 1 null terminator (not sent) */

/* Private variables ---------------------------------------------------------*/
static bool imu_running = false;
static IMU_Test_t latest_test = {0};
uint8_t dma_buffer_imu[DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer_imu")));
uint16_t dma_old_pos_imu = 0;

/* Private function prototypes -----------------------------------------------*/
static void send_command(const char* cmd);
static bool parse_packet_test(uint8_t* buffer_start, uint16_t length, IMU_Test_t* test);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the dummy IMU
  * @param  None
  * @retval None
  * @note   Sets up the UART and DMA for receiving data, configuring
  *         hardware peripherals and preparing the DMA buffer for data reception.
  */
void dummy_IMU_init(void) {
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

    /* Initialize latest test */
    memset(&latest_test, 0, sizeof(IMU_Test_t));
}

/** @brief  Start the dummy IMU
  * @param  None
  * @retval None
  * @note   Sends the START command to the dummy IMU.
  */
void dummy_IMU_start(uint64_t *start_time) {
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
void dummy_IMU_stop(uint64_t *stop_time) {
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
bool dummy_IMU_is_running(void) {
    return imu_running;
}

/** @brief  Get the latest IMU test data
  * @param  test: Pointer to IMU_Test_t structure to fill with latest test data
  * @retval None
  * @note   Parses the DMA buffer for new test packets and updates the provided test structure.
  */
void dummy_IMU_get_test_data(IMU_Test_t *test) {
    if (test) {
        /* Get current DMA position */
        uint16_t dma_pos = DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_CHANNEL_2);

        /* Check for new data */
        uint16_t bytes_available = (dma_pos >= dma_old_pos_imu) ? (dma_pos - dma_old_pos_imu) : (DMA_BUFFER_SIZE - dma_old_pos_imu + dma_pos);

        if (bytes_available >= TEST_PACKET_SIZE) {
            /* Look for test packet header in the buffer */
            for (uint16_t i = 0; i <= bytes_available - TEST_PACKET_SIZE; i++) {
                uint16_t check_pos = (dma_old_pos_imu + i) % DMA_BUFFER_SIZE;
                if (dma_buffer_imu[check_pos] == 0xB4 &&
                    dma_buffer_imu[(check_pos + 1) % DMA_BUFFER_SIZE] == 0xB4) {
                    /* Found header, try to parse the test packet */
                    if (parse_packet_test(&dma_buffer_imu[check_pos], TEST_PACKET_SIZE, test)) {
                        dma_old_pos_imu = (check_pos + TEST_PACKET_SIZE) % DMA_BUFFER_SIZE;
                        break;
                    }
                }
            }
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the dummy IMU
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over UART5 to control the dummy IMU.
  */
static void send_command(const char* cmd) {
    while (*cmd) {
        while (!LL_USART_IsActiveFlag_TXE(UART5));
        LL_USART_TransmitData8(UART5, *cmd++);
    }
}

/** @brief  Parse a IMU test packet from the DMA buffer
  * @param  buffer_start: Pointer to the start of the packet in the DMA buffer
  * @param  length: Length of the data available from buffer_start
  * @param  test: Pointer to IMU_Test_t structure to fill with parsed test data
  * @retval true if a valid test packet was parsed, false otherwise
  * @note   Validates and extracts data from a IMU test packet.
  */
static bool parse_packet_test(uint8_t* buffer_start, uint16_t length, IMU_Test_t* test) {
    if (length < TEST_PACKET_SIZE) return false;

    /* Copy packet data handling circular buffer wrap-around */
    uint8_t packet[TEST_PACKET_SIZE];
    for (int i = 0; i < TEST_PACKET_SIZE; i++) {
        packet[i] = buffer_start[i];
    }

    /* Check header */
    if (packet[0] != 0xB4 || packet[1] != 0xB4) return false;

    /* Extract test data */
    test->identifier[0] = packet[2];
    test->identifier[1] = packet[3];
    test->identifier[2] = packet[4];
    test->identifier[3] = packet[5];
    test->identifier[4] = packet[6];
    test->identifier[5] = '\0'; /* Null-terminate the string */
    test->packet_number = (uint32_t)(packet[7] | (packet[8] << 8) | (packet[9] << 16) | (packet[10] << 24));

    return true;
}
/**
  ******************************************************************************
  * @file    vectornav.c
  * @brief   VectorNav GPS/IMU functions
  * @note    Handles communication with the VectorNav VN-300 IMU via UART and DMA
  ******************************************************************************
  */

#include "vectornav.h"
#include "windmaster.h"
#include "systime.h"
#include "recorder.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/
#define DMA_BUFFER_SIZE 1024 /* 1 KB for incoming IMU sensor data */
#define PACKET_SIZE 86

/* Private variables ---------------------------------------------------------*/
static bool imu_running = false;
static VN_Packet_t latest_packet = {0};
uint8_t dma_buffer_imu[DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer_imu")));
uint16_t dma_old_pos_imu = 0;

double gps_tow;
uint16_t gps_week;
uint8_t vn_rx_buffer[256];

/* Private function prototypes -----------------------------------------------*/
static void send_command(const char* cmd);
static void flush_rx(void);
static RTC_DateTime_t convert_to_utc(uint16_t gps_week, double gps_tow);

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
    flush_rx();

    /* Disable async mode in case it was left on from previous power cycle */
    send_command("$VNWRG,75,0,20,00*F819\n");

    /* Wait for the command echo and flush it */
    HAL_Delay(10);
    
    flush_rx();

    /* Configure USART3 for DMA RX (TX will use polling) */
    LL_USART_EnableDMAReq_RX(USART3);

    /* Configure DMA RX addresses (DMA1 Channel 3) */
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
                          LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE),
                          (uint32_t)dma_buffer_imu,
                          LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, DMA_BUFFER_SIZE);

    /* DMA will be enabled when the VectorNav starts */

    /* Initialize latest packet */
    memset(&latest_packet, 0, sizeof(VN_Packet_t));
}

/** @brief  Start the vectornav
  * @param  None
  * @retval None
  * @note   Enables VectorNav async mode for 50Hz binary output (rate divisor = 8).
  *         Starts the DMA channel for receiving binary data.
  */
void vn_start(void) {
    if (imu_running == false) {
        /* Ensure that the DMA channel is disabled before sending command */
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

        /* Flush UART RX buffer before sending command */
        flush_rx();

        /* Enable async mode (50Hz binary output, divisor = 8) */
        send_command("$VNWRG,75,1,8,01,01EA*07CF\n");

        /* Wait for response */
        HAL_Delay(10);

        /* Flush the ASCII echo response */
        flush_rx();

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
    flush_rx();

    /* Disable async mode */
    send_command("$VNWRG,75,0,20,00*F819\n");

    /* Wait for response */
    HAL_Delay(10);

    /* Flush the ASCII echo response */
    flush_rx();

    /* Reset DMA buffer position tracking */
    dma_old_pos_imu = 0;

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
  const uint16_t MASK = DMA_BUFFER_SIZE - 1;
  const uint16_t wr = (DMA_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3)) & MASK;
  uint16_t rd = dma_old_pos_imu;
  uint16_t avail = (wr - rd) & MASK;
  bool packets_drained = false;
  uint16_t iterations = 0;
  const uint16_t MAX_ITERATIONS = 16;

  /* Loop while full packets are available, with an iteration cap */
  while (avail >= PACKET_SIZE && iterations < MAX_ITERATIONS) {
    iterations++;

    /* Expect start-of-packet byte (0xFA) */
    uint8_t h0 = dma_buffer_imu[rd];
    if (h0 != 0xFA) {
      rd = (rd + 1) & MASK;
      avail--;
      continue;
    }

    /* Copy one full packet from ring buffer (split if wrap) */
    uint8_t tmp[PACKET_SIZE];
    uint16_t head_len = DMA_BUFFER_SIZE - rd;

    if (head_len >= PACKET_SIZE) {
      memcpy(tmp, &dma_buffer_imu[rd], PACKET_SIZE);
    } else {
      memcpy(tmp, &dma_buffer_imu[rd], head_len);
      memcpy(tmp + head_len, &dma_buffer_imu[0], (size_t)(PACKET_SIZE - head_len));
    }

    /* Update the latest packet */
    size_t copy_len;
    if (sizeof(VN_Packet_t) < PACKET_SIZE) {
      copy_len = sizeof(VN_Packet_t);
    } else {
      copy_len = PACKET_SIZE;
    }

    memcpy(&latest_packet, tmp, copy_len);

    /* Queue the packet for recording with current system timestamp */
    recorder_queue_vn(&latest_packet);

    /* Advance by exactly one packet */
    rd = (rd + PACKET_SIZE) & MASK;
    avail = (wr - rd) & MASK;
    packets_drained = true;
  }

  dma_old_pos_imu = rd;
  return packets_drained;
}

/** @brief  Check if a valid GPS fix is acquired
  * @param  None
  * @retval true if GPS fix is valid, false otherwise
  * @note   Reads the GNSS LLA register and parses the fix status.
  */
bool vn_gps_fix(void) {
  /* Read the GNSS LLA register to update fix status */
  vn_read_gnss_lla();
  
  /* Check the response string to see if the fix status is >0 (valid fix) */
  char* token;
  token = strtok((char*)vn_rx_buffer, ",");
  uint8_t field_index = 0;
  bool fix_acquired = false;

  while (token != NULL) {
      field_index++;
      if (field_index == 5) { /* Fix status is the 5th field */
          uint8_t fix_status = atoi(token);
          if (fix_status > 0) {
              fix_acquired = true;
          }
          break;
      }
      token = strtok(NULL, ",");
  }

  return fix_acquired;
}


RTC_DateTime_t vn_get_utc_datetime(void) {
    /* Read the GNSS LLA Register to get the current time */
    vn_read_gnss_lla();

    /* Parse the response to extract GPS week and TOW */
    char* token;
    token = strtok((char*)vn_rx_buffer, ",");
    uint8_t field_index = 0;
    bool have_tow = false;
    bool have_week = false;
    double tow_local = 0.0;
    uint16_t week_local = 0;

    while (token != NULL) {
        field_index++;
        if (field_index == 3) { /* GPS TOW is the 3rd field */
        tow_local = atof(token);
        have_tow = true;
        } else if (field_index == 4) { /* GPS Week is the 4th field */
        week_local = (uint16_t)atoi(token);
        have_week = true;
            break; /* We have both fields we need */
        }
        token = strtok(NULL, ",");
    }

    if (!have_tow || !have_week) {
      RTC_DateTime_t invalid = {0};
      return invalid;
    }

    /* Preserve parsed values for any other code that reads these globals */
    gps_tow = tow_local;
    gps_week = week_local;

    return convert_to_utc(week_local, tow_local);
}

/* Private functions ---------------------------------------------------------*/

/** @brief  Send a command to the vectornav using polling TX (blocking)
  * @param  cmd: Null-terminated command string to send
  * @retval None
  * @note   Transmits the command string over USART3 using polling mode.
  *         Suitable for short ASCII configuration commands.
  */
static void send_command(const char* cmd) {
  while (*cmd) {
    /* Wait for TX empty */
    while (!LL_USART_IsActiveFlag_TXE(USART3));

    /* Send character */
    LL_USART_TransmitData8(USART3, (uint8_t)*cmd);

    /* Wait for transmission complete */
    while (!LL_USART_IsActiveFlag_TC(USART3));

    cmd++;
  }
}

/** @brief  Flush RX FIFO to clear command echoes (USART3)
  * @param  None
  * @retval None
  * @note   Reads and discards all available data and clears IDLE if set.
  */
static void flush_rx(void) {
  while (LL_USART_IsActiveFlag_RXNE(USART3)) {
    (void)LL_USART_ReceiveData8(USART3);
  }

  if (LL_USART_IsActiveFlag_IDLE(USART3)) {
    LL_USART_ClearFlag_IDLE(USART3);
  }
}

/** @brief  Validate a received VN packet
  * @param  pkt: Pointer to the start of the packet buffer
  * @param  struct: Pointer to VN_Packet_t structure to populate
  * @retval true if the packet is valid, false otherwise
  */
bool vn_validate_packet(uint8_t* buffer_start, uint16_t length, VN_Packet_t* packet) {
  /* Validate header and length */
  if (length < PACKET_SIZE || buffer_start[0] != 0xFA) {
    return false;
  }

  /* Compute checksum */
  uint16_t checksum = 0;
  for (uint8_t i = 0; i < PACKET_SIZE - 2; i++) {
    checksum += buffer_start[i];
  }

  /* Validate checksum (Big Endian conversion) */
  uint16_t received_checksum = (buffer_start[PACKET_SIZE - 2] << 8) | buffer_start[PACKET_SIZE - 1];
  if (checksum != received_checksum) {
    return false;
  }
  
  memcpy(packet, buffer_start, PACKET_SIZE);
  return true;
}

/**  @brief  Read the GNSS Solution - LLA Register
  *  @param  None
  *  @retval None
  *  @note   Sends a command to read the GNSS LLA register and stores the response in vn_gnss_rx_buffer.
  */
void vn_read_gnss_lla(void) {
    /* Flush RX buffer before sending command */
    flush_rx();

    /* Clear the rx buffer */
    memset(vn_rx_buffer, 0, sizeof(vn_rx_buffer));

    /* Send command to read GNSS LLA register */
    send_command("$VNRRG,58*XX\n");

    /* Wait for response */
    HAL_Delay(1);

    /* Read response into vn_rx_buffer */
    uint8_t index = 0;
    while (LL_USART_IsActiveFlag_RXNE(USART3) && index < sizeof(vn_rx_buffer) - 1) {
        vn_rx_buffer[index++] = LL_USART_ReceiveData8(USART3);
    }

    /* Null-terminate the string */
    vn_rx_buffer[index] = '\0';
}

static RTC_DateTime_t convert_to_utc(uint16_t gps_week, double gps_tow) {
  /* Round the GPS Time of Week to nearest second */
  uint32_t tow_rounded = (uint32_t)(gps_tow + 0.5);

  /* Calculate total seconds since GPS epoch */
  /* 604800 seconds per GPS week. Guard against overflow of 32-bit seconds. */
  const uint32_t SECONDS_PER_WEEK = 604800U;
  if ((uint32_t)gps_week > (UINT32_MAX / SECONDS_PER_WEEK)) {
    RTC_DateTime_t invalid = {0};
    return invalid;
  }

  uint32_t total_gps_seconds = ((uint32_t)gps_week * SECONDS_PER_WEEK) + tow_rounded;

  return gps_to_datetime(total_gps_seconds);
}
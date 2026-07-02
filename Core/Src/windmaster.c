/**
  ******************************************************************************
  * @file    windmaster.c
  * @brief   WindMaster functions
  * @note    Implements WindMaster Mode 10 - Binary UVW Long protocol
  *          UART4 @ 57600 baud, 23-byte packets at 20Hz
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
#include <limits.h>

/* Private defines -----------------------------------------------------------*/
#define DMA_BUFFER_SIZE     1024        /* 1 KB for incoming WM sensor data */
#define PACKET_SIZE         23          /* 23 Bytes for M10, 13 Bytes for M8 */
#define HEADER_SIZE         2
#define HEADER_VALUE        0xB4        /* 0xB4 for M10, 0xB2 for M8 */
#define DATA_SIZE           18          /* 18 Bytes for M10, 8 Bytes for M8 */
#define CHECKSUM_SIZE       1
#define WM_UART             UART4
#define WM_UART_IRQn        UART4_IRQn
#define WM_DMA              DMA2
#define WM_DMA_RX_CHANNEL   LL_DMA_CHANNEL_5

/* Private variables ---------------------------------------------------------*/
static bool wm_running = false;
static WM_Packet_t latest_packet = {0};
static uint32_t wm_bad_data = 0;
uint8_t dma_buffer_wm[DMA_BUFFER_SIZE] __attribute__((section(".dma_buffer_wm")));
uint16_t dma_old_pos_wm = 0;

/* Private function prototypes -----------------------------------------------*/
static void send_command(const char* cmd);
static void flush_rx(void);
bool wm_validate_packet(const uint8_t* pkt, WM_Packet_t* packet);
static bool wm_data_valid(const WM_Packet_t* packet);

/* Public functions ----------------------------------------------------------*/

/** @brief  Initialize the WindMaster
  * @param  None
  * @retval None
  * @note   Sends '*\r' command to enter configuration mode, flushes echo,
  *         then configures DMA2 Ch5 for RX (but doesn't enable it).
  *         Leaves the WindMaster in config mode (not sending binary data).
  *         Call wm_start() to begin measurement mode and enable DMA.
  *         UART4 RX uses DMA2 Channel 5.
  *         UART4 @ 57600 baud, 8-N-1.
  */
void wm_init(void) {
  /* Clear any pending flags and data */
  LL_USART_ClearFlag_TC(WM_UART);
  flush_rx();

  /* Send '*\r' to enter configuration mode (stops any output) */
  send_command("*\r");

  /* Wait for echo to arrive and flush it */
  HAL_Delay(10);
  flush_rx();

  /* Now configure DMA for binary data reception */
  /* Ensure DMA channel is disabled before touching counters/addresses */
  LL_DMA_DisableChannel(WM_DMA, WM_DMA_RX_CHANNEL);

  /* Configure DMA RX addresses (DMA2 Channel 5) */
  LL_DMA_ConfigAddresses(WM_DMA, WM_DMA_RX_CHANNEL,
                        LL_USART_DMA_GetRegAddr(WM_UART, LL_USART_DMA_REG_DATA_RECEIVE),
                        (uint32_t)dma_buffer_wm,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetDataLength(WM_DMA, WM_DMA_RX_CHANNEL, DMA_BUFFER_SIZE);

  /* Leave RX DMA requests disabled until wm_start() enables the channel. */
  LL_USART_DisableDMAReq_RX(WM_UART);
}

/** @brief  Start the WindMaster
  * @param  None
  * @retval None
  * @note   Enables RX DMA before sending 'Q\r' so command echo and the first
  *         binary bytes are captured by the ring buffer instead of overrunning
  *         the UART receive register.
  */
void wm_start(void) {
  /* Return if already running */
  if (wm_running) {
    return;
  }

  /* Start with a clean UART and DMA state. */
  LL_USART_DisableDMAReq_RX(WM_UART);
  LL_DMA_DisableChannel(WM_DMA, WM_DMA_RX_CHANNEL);
  flush_rx();
  memset(dma_buffer_wm, 0, DMA_BUFFER_SIZE);
  dma_old_pos_wm = 0;

  /* Reset DMA transfer counter (must be done while channel is disabled) */
  LL_DMA_SetDataLength(WM_DMA, WM_DMA_RX_CHANNEL, DMA_BUFFER_SIZE);
  LL_DMA_ClearFlag_TC5(WM_DMA);
  LL_DMA_ClearFlag_HT5(WM_DMA);
  LL_DMA_ClearFlag_TE5(WM_DMA);

  /* Enable DMA before starting measurement mode so echoed 'Q' bytes are safe. */
  LL_DMA_EnableChannel(WM_DMA, WM_DMA_RX_CHANNEL);
  LL_USART_EnableDMAReq_RX(WM_UART);

  /* Send 'Q\r' to start measurement mode. Echo/CRLF will be skipped by parser. */
  send_command("Q\r");

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
    LL_USART_DisableDMAReq_RX(WM_UART);
    LL_DMA_DisableChannel(WM_DMA, WM_DMA_RX_CHANNEL);
    flush_rx();

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
  const uint16_t wr = (DMA_BUFFER_SIZE - LL_DMA_GetDataLength(WM_DMA, WM_DMA_RX_CHANNEL)) & MASK;
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
      /* Corrupted packet - record raw bytes and skip */
      WM_Packet_t bad_pkt;
      memcpy(&bad_pkt, pkt, PACKET_SIZE);
      recorder_queue_bad_wm(&bad_pkt);
      rd = (rd + 1) & MASK;
      avail--;
      continue;
    }

    /* Check data quality (status word + sentinel values) */
    if (!wm_data_valid(&latest_packet)) {
      /* Structurally valid but bad data — record and skip, flush stale VN data */
      recorder_queue_bad_wm(&latest_packet);
      recorder_flush_vn_queue();
      rd = (rd + PACKET_SIZE) & MASK;
      avail = (wr - rd) & MASK;
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
  * @note   Transmits the command string over UART4 using polling mode.
  *         Used for short configuration commands ('*\r', 'Q\r').
  *         Includes 2ms inter-character delay to allow WindMaster to echo and process each byte.
  */
static void send_command(const char* cmd) {
  while (*cmd) {
    /* Wait for TX empty */
    while (!LL_USART_IsActiveFlag_TXE(WM_UART));

    /* Send character */
    LL_USART_TransmitData8(WM_UART, *cmd);

    /* Wait for transmission complete */
    while (!LL_USART_IsActiveFlag_TC(WM_UART));

    cmd++;
  }
}

/** @brief  Flush RX FIFO to clear command echoes
  * @param  None
  * @retval None
  * @note   Reads and discards all available data from UART4 RX.
  *         Used after sending configuration commands to clear echoes.
  */
static void flush_rx(void) {
  /* Drain RX FIFO */
  while (LL_USART_IsActiveFlag_RXNE(WM_UART)) {
    (void)LL_USART_ReceiveData8(WM_UART);
  }

  /* Clear IDLE flag if set */
  if (LL_USART_IsActiveFlag_IDLE(WM_UART)) {
    LL_USART_ClearFlag_IDLE(WM_UART);
  }

  /* Clear line errors that can otherwise block subsequent DMA reception. */
  if (LL_USART_IsActiveFlag_ORE(WM_UART)) {
    LL_USART_ClearFlag_ORE(WM_UART);
  }
  if (LL_USART_IsActiveFlag_FE(WM_UART)) {
    LL_USART_ClearFlag_FE(WM_UART);
  }
  if (LL_USART_IsActiveFlag_NE(WM_UART)) {
    LL_USART_ClearFlag_NE(WM_UART);
  }
  if (LL_USART_IsActiveFlag_PE(WM_UART)) {
    LL_USART_ClearFlag_PE(WM_UART);
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

/** @brief Check data quality of a structurally valid WM packet
  * @param packet: Pointer to validated WM_Packet_t
  * @retval true if data is usable, false if sample error or sentinel values
  * @note   Status byte 0x01-0x09 = sample failure / HW error (reject).
  *         0x00 = OK, 0x0A = gain at max (results OK), 0x0B = retries used (accept).
  *         U/V/W/SoS == INT16_MAX (32767) = sentinel for invalid measurement (reject).
  *         Temp is NOT checked — no PRT sensor installed, always reads INT16_MAX.
  */
static bool wm_data_valid(const WM_Packet_t* packet) {
  uint8_t status_low = packet->status & 0xFF;

  /* Reject status codes 0x01–0x09 (sample failures and hardware errors) */
  if (status_low >= 0x01 && status_low <= 0x09) {
    wm_bad_data++;
    return false;
  }

  /* Reject sentinel values indicating invalid measurement */
  if (packet->U_axis_speed == INT16_MAX ||
      packet->V_axis_speed == INT16_MAX ||
      packet->W_axis_speed == INT16_MAX ||
      packet->SoS == INT16_MAX) {
    wm_bad_data++;
    return false;
  }

  return true;
}

/** @brief Get count of rejected bad-data packets
  * @retval Number of structurally valid packets rejected for bad data
  */
uint32_t wm_get_bad_data_count(void) {
  return wm_bad_data;
}

/** @brief  Check if the WindMaster is alive and responding on UART4
  * @param  None
  * @retval true if the WM responded to a D3 (request config) command, false otherwise
  * @note   Sends 'D3\r' to request current configuration and polls for a response.
  *         The WM must already be in config mode (wm_init() sends '*\r').
  *         Disables the UART4 IRQ during the exchange so the response is
  *         handled entirely by this polling path.
  */
bool wm_check_alive(void)
{
  if (wm_running) {
    return true;
  }

  /* Disable UART4 IRQ during the polled response exchange */
  NVIC_DisableIRQ(WM_UART_IRQn);

  /* Make sure the polled response exchange owns UART RX. */
  LL_USART_DisableDMAReq_RX(WM_UART);

  /* Clear any pending flags, errors, and data */
  LL_USART_ClearFlag_TC(WM_UART);
  LL_USART_ClearFlag_IDLE(WM_UART);
  LL_USART_ClearFlag_ORE(WM_UART);
  LL_USART_ClearFlag_FE(WM_UART);
  LL_USART_ClearFlag_NE(WM_UART);
  LL_USART_ClearFlag_PE(WM_UART);
  (void)WM_UART->RDR;

  /* Send 'D3\r' to request current configuration (works in config mode) */
  send_command("D3\r");

  /* Poll for response bytes with timeout */
  char rx_buf[128];
  uint16_t rx_idx = 0;
  uint32_t timeout_start = HAL_GetTick();
  const uint32_t TIMEOUT_MS = 200;

  while ((HAL_GetTick() - timeout_start) < TIMEOUT_MS && rx_idx < (sizeof(rx_buf) - 1)) {
    if (LL_USART_IsActiveFlag_RXNE(WM_UART)) {
      rx_buf[rx_idx] = (char)LL_USART_ReceiveData8(WM_UART);
      rx_idx++;
      /* Reset timeout to wait for next byte */
      timeout_start = HAL_GetTick();
    }
    if (LL_USART_IsActiveFlag_IDLE(WM_UART)) {
      LL_USART_ClearFlag_IDLE(WM_UART);
      HAL_Delay(1);
      break;
    }
  }
  rx_buf[rx_idx] = '\0';

  /* Leave RX DMA disabled until wm_start() enables the DMA channel. */
  NVIC_EnableIRQ(WM_UART_IRQn);

  /* Diagnostic: print what we received */
  shell_printf("\r\n[WM DBG] received=%u bytes\r\n", rx_idx);
  if (rx_idx > 0) {
    shell_printf("[WM DBG] hex:");
    uint16_t show = rx_idx < 64 ? rx_idx : 64;
    for (uint16_t i = 0; i < show; i++) {
      shell_printf(" %02X", (uint8_t)rx_buf[i]);
    }
    shell_printf("\r\n");
    shell_printf("[WM DBG] str: %.80s\r\n", rx_buf);
  }

  /* Echo is enabled on this system, so skip echoed "D3" and find the config line. */
  for (uint16_t i = 0; i + 1 < rx_idx; i++) {
    bool line_start = (i == 0) || rx_buf[i - 1] == '\r' || rx_buf[i - 1] == '\n';
    bool mode_field = rx_buf[i] == 'M' && rx_buf[i + 1] >= '0' && rx_buf[i + 1] <= '9';
    if (line_start && mode_field) {
      return true;
    }
  }

  return false;
}

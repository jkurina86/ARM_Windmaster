/**
  ******************************************************************************
  * @file    vn_cmd.h
  * @brief   VectorNav pass-through command interface
  * @note    Direct UART5 communication for sensor configuration and testing
  ******************************************************************************
  */

#ifndef INC_VN_CMD_H_
#define INC_VN_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Exported defines ----------------------------------------------------------*/
#define VN_CMD_MAX_LEN      128  /* Maximum command length */
#define VN_RESP_MAX_LEN     256  /* Maximum response buffer size */
#define VN_RESP_TIMEOUT_MS  3000 /* 3-second timeout for response */

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Send a passthrough command to VectorNav and receive response
 * @param cmd: Command string to send (e.g., "$VNRRG,0*XX\r\n")
 * @retval true if response received, false on timeout/error
 * @note  Blocks until response received or timeout
 * @note  Response is printed directly to shell via shell_printf()
 */
bool vn_cmd_passthrough(const char *cmd);

/**
 * @brief Receive response from VectorNav with timeout
 * @param buffer: Buffer to store response
 * @param max_len: Maximum bytes to receive
 * @param timeout_ms: Timeout in milliseconds
 * @retval Number of bytes received, 0 on timeout
 */
uint16_t vn_cmd_recv_response(uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* INC_VN_CMD_H_ */

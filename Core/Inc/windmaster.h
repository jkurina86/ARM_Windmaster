/**
  ******************************************************************************
  * @file    windmaster.h
  * @brief   WindMaster functions
  * @note    Placeholder for WindMaster functionality
  ******************************************************************************
  */

#ifndef INC_WINDMASTER_H_
#define INC_WINDMASTER_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Exported Types -----------------------------------------------------------*/
typedef enum {
    WM_STATE_STOPPED,
    WM_STATE_RUNNING
} WM_State_t;

/* WindMaster Data Structure: Based on Mode 10 - Binary, UVW, Long in WM manual pg. 41 */
typedef struct __attribute__((packed)) {
  uint16_t header;        // 46260, or 0xB4B4 (2 bytes)
  uint16_t status;        // Status word
  int16_t U_axis_speed;   // U-axis wind speed
  int16_t V_axis_speed;   // V-axis wind speed
  int16_t W_axis_speed;   // W-axis wind speed
  int16_t SoS;            // Speed of Sound
  int16_t A1;             // Analog Input 1
  int16_t A2;             // Analog Input 2
  int16_t A3;             // Analog Input 3
  int16_t A4;             // Analog Input 4
  int16_t Temp;           // Temperature from PRT
  uint8_t checksum;       // Checksum: Checksum (exclusive OR of bytes between header and checksum)
} WM_Packet_t;

/* Function Prototypes -------------------------------------------------------*/
void wm_init(void);
void wm_start(void);
void wm_stop(void);
bool wm_is_running(void);
bool wm_drain_and_queue(void);

#endif /* INC_WINDMASTER_H_ */
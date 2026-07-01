/**
  ******************************************************************************
  * @file    vectornav.h
  * @brief   VectorNav GPS/IMU functions
  * @note    Placeholder for IMU functionality
  ******************************************************************************
  */

#ifndef INC_VECTORNAV_H_
#define INC_VECTORNAV_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ab-rtcmc-rtc.h"

/* Exported Types -----------------------------------------------------------*/
typedef enum {
    VECTORNAV_STATE_STOPPED,
    VECTORNAV_STATE_RUNNING
} IMU_State_t;

/* VectorNav Packet: All output in little-endian format, except the checksum (big-endian).
   If you compute the CRC across the entire packet, exclude the sync byte but include the checksum,
   the resulting CRC will evaluate to 0x0000.
*/
typedef struct __attribute__((packed)) {
  uint8_t sync;            // 0xFA (1 byte)
  uint8_t group;           /* 0x01 for now */
  uint16_t types_common;   /* */
  uint64_t timegps;     // GPS time in nanoseconds since 1-1-1980 00:00:00 (8 bytes)
  float yaw;
  float pitch;
  float roll;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  double latitude;
  double longitude;
  double altitude;
  float vel_n;
  float vel_e;
  float vel_d;
  float acc_x;
  float acc_y;
  float acc_z;
  uint16_t checksum;       // Checksum (2 bytes)
} VN_Packet_t;

/* Function Prototypes -------------------------------------------------------*/
void vn_init(void);
void vn_start(void);
void vn_stop(void);
bool vn_is_running(void);
bool vn_drain_and_queue(void);
bool vn_gps_fix(void);
RTC_DateTime_t vn_get_gps_datetime(void);
bool vn_check_alive(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* INC_VECTORNAV_H_ */
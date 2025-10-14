/**
  ******************************************************************************
  * @file    recorder.h
  * @brief   Recorder module functions
  * @note    Handles data recording from WindMaster and IMU sensors
  ******************************************************************************
  */

#ifndef INC_RECORDER_H_
#define INC_RECORDER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "windmaster.h"
#include "vectornav.h"


/* Exported Types ------------------------------------------------------------*/

/* Combined data structure for recording both WindMaster and VectorNav data */
typedef struct {
  uint8_t header_padding[12];
  uint32_t log_index;
  uint64_t timegps;     // GPS time in nanoseconds since 1-1-1980 00:00:00 (8 bytes)
  float yaw;
  float pitch;
  float roll;
  float vel_n;
  float vel_e;
  float vel_d;
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  double latitude;
  double longitude;
  double altitude;
  int16_t U_axis_speed;   // U-axis wind speed
  int16_t V_axis_speed;   // V-axis wind speed
  int16_t W_axis_speed;   // W-axis wind speed
  int16_t SoS;            // Speed of Sound
  int16_t Temp;           // Temperature from PRT
  uint8_t footer_padding[12];    // Padding to reach 128 bytes
} Recorder_Data_t;

/* Function Prototypes -------------------------------------------------------*/

void recorder_init(void);
void recorder_start(void);
void recorder_stop(void);
void recorder_service(void);
void recorder_flush(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_RECORDER_H_ */
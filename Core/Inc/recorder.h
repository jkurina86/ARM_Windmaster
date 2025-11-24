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
#include <stdbool.h>
#include "windmaster.h"
#include "vectornav.h"



/* Exported Types ------------------------------------------------------------*/

typedef struct {
  WM_Packet_t wm_packet;
  uint64_t timestamp_us;
} WM_QueueEntry_t;

typedef struct {
  VN_Packet_t vn_packet;
  uint64_t timestamp_us;
} VN_QueueEntry_t;

/* Combined data structure for recording both WindMaster and VectorNav data */
typedef struct {
  uint32_t magic_number; // Unique identifier for the start of a record
  uint32_t log_index;
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
  int16_t U_axis_speed;   // U-axis wind speed
  int16_t V_axis_speed;   // V-axis wind speed
  int16_t W_axis_speed;   // W-axis wind speed
  int16_t SoS;            // Speed of Sound
  int16_t Temp;                   // Temperature from PRT
  uint8_t footer_padding[30];    // Padding to reach 128 bytes
} Recorder_Data_t;

/* Stats structure for debug/monitoring */
typedef struct {
  bool recording;                 // Currently recording
  uint32_t records_written;       // Total records written to SD
  uint32_t active_buffer_records; // Records in active buffer
  uint32_t active_buffer_capacity;// Active buffer capacity (32)
  uint8_t wm_queue_count;         // Current WindMaster queue entries
  uint8_t wm_queue_capacity;      // WindMaster queue capacity (64)
  uint8_t vn_queue_count;         // Current VectorNav queue entries
  uint8_t vn_queue_capacity;      // VectorNav queue capacity (64)
  uint8_t wm_queue_max;           // Max WM queue depth seen
  uint8_t vn_queue_max;           // Max VN queue depth seen
  uint32_t wm_drops;              // WindMaster queue overflow drops
  uint32_t vn_drops;              // VectorNav queue overflow drops
  char filename[64];              // Current log filename
} recorder_stats_t;

/* Function Prototypes -------------------------------------------------------*/

void recorder_init(void);
void recorder_start(void);
void recorder_stop(void);
void recorder_service(void);
recorder_stats_t recorder_get_stats(void);
void recorder_clear_stats(void);
bool recorder_is_recording(void);

void recorder_queue_vn(const VN_Packet_t *pkt, uint64_t t_us);
void recorder_queue_wm(const WM_Packet_t *pkt, uint64_t t_us);

#ifdef __cplusplus
}
#endif
#endif /* INC_RECORDER_H_ */
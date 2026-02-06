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
  uint32_t timestamp_s;     // Seconds from systime_snapshot()
  uint16_t timestamp_ms;    // Milliseconds from systime_snapshot() (0-999)
  WM_Packet_t wm_packet;
} WM_QueueEntry_t;

typedef struct {
  uint32_t timestamp_s;     // Seconds from systime_snapshot()
  uint16_t timestamp_ms;    // Milliseconds from systime_snapshot() (0-999)
  VN_Packet_t vn_packet;
} VN_QueueEntry_t;

/* Combined data structure for recording both WindMaster and VectorNav data.
 * Embeds raw packed sensor packets to preserve all fields (status, checksums,
 * analog inputs, VN headers) without field-by-field extraction.
 *
 * Layout (128 bytes):
 *   Offset  Size  Field
 *     0       4   magic_number
 *     4       4   log_index
 *     8       4   epoch_seconds
 *    12       2   ms
 *    14       2   timing_offset_ms
 *    16      23   wm_packet (packed)
 *    39      86   vn_packet (packed)
 *   125       3   trailing padding (compiler-inserted, struct align 4)
 */
typedef struct {
  uint32_t magic_number;
  uint32_t log_index;
  uint32_t epoch_seconds;
  uint16_t ms;
  int16_t timing_offset_ms;   // WM timestamp - VN timestamp (signed, +/- 25ms max)
  WM_Packet_t wm_packet;
  VN_Packet_t vn_packet;
} Recorder_Data_t;

_Static_assert(sizeof(Recorder_Data_t) == 128, "Recorder_Data_t must remain 128 bytes");

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
  uint32_t vn_drops;              // VectorNav queue overflow drops (data loss)
  uint32_t vn_discards;           // VectorNav intentional discards (nearest-neighbor pairing)
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
void recorder_debug_queue(void);

void recorder_queue_vn(const VN_Packet_t *pkt);
void recorder_queue_wm(const WM_Packet_t *pkt);
void recorder_flush_vn_queue(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_RECORDER_H_ */
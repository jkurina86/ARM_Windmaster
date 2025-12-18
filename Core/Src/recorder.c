/**
  ******************************************************************************
  * @file    recorder.c
  * @brief   Recorder module functions
  * @note    Handles data recording from WindMaster and IMU sensors
  ******************************************************************************
  */

#include "recorder.h"
#include "filesystem.h"
#include "systime.h"
#include "shell.h"
#include <stdbool.h>

/* Definitions ------------------------------------------------------------------*/
#define WM_LEN 23
#define VN_LEN 86
#define WM_Q_LEN 16 /* Must be power of two, ~800ms buffer @20Hz */
#define VN_Q_LEN 16 /* Must be power of two, ~320ms buffer @50Hz */
#define RECORD_BUFFER_SIZE 4096
#define MAX_RECORDS_PER_SERVICE 32  /* 32*128 = 4KB, Only one full buffer will be processed at a time at most. */
#define MAX_OFFSET_MS 25  /* 25ms tolerance for nearest-neighbor pairing (worst case phase offset between async sensors) */

/* Queue Structures, location in RAM2 specified in linker script */
static WM_QueueEntry_t wm_queue[WM_Q_LEN] __attribute__((section(".queue_wm")));
static VN_QueueEntry_t vn_queue[VN_Q_LEN] __attribute__((section(".queue_vn")));

/* Queue indices */
static uint8_t wm_q_head = 0;
static uint8_t wm_q_tail = 0;
static uint8_t vn_q_head = 0;
static uint8_t vn_q_tail = 0;
static bool desync_active = false;          /* True until first successful pair after a drop */

/* Double buffers for recording, location in RAM2 specified in linker script */
uint8_t record_buffer_a[RECORD_BUFFER_SIZE]__attribute__((section(".record_buffer_a")));
uint8_t record_buffer_b[RECORD_BUFFER_SIZE]__attribute__((section(".record_buffer_b")));

/* Buffer management state */
static uint8_t* active_buffer = record_buffer_a;     /* Currently being filled */
static uint8_t* flush_buffer = record_buffer_b;      /* Ready to write to SD */
static uint32_t active_buf_index = 0;                /* Number of records in active buffer (0-31) */
static bool flush_pending = false;                   /* True if flush in progress */

static bool recording = false;
static uint32_t record_index = 0;

/* File handle for log file */
static FIL log_fil;
char filename[128];

/* Statistics tracking */
static uint32_t wm_drops = 0;
static uint32_t vn_drops = 0;
static uint32_t vn_discards = 0;
static uint8_t wm_queue_max = 0;
static uint8_t vn_queue_max = 0;

/* Private function prototypes -----------------------------------------------*/
static void flip_buffers(void);
static Recorder_Data_t build_record(const VN_QueueEntry_t *vn_entry, const WM_QueueEntry_t *wm_entry, int16_t timing_offset_ms);
static uint8_t get_queue_count(uint8_t head, uint8_t tail, uint8_t len);
static void clear_queues(void);


void recorder_init(void) {
  /* Initialize recording state */
  recording = false;
  record_index = 0;
  active_buffer = record_buffer_a;
  flush_buffer = record_buffer_b;
  active_buf_index = 0;
  flush_pending = false;
  desync_active = false;

  /* Initialize statistics */
  wm_drops = 0;
  vn_drops = 0;
  vn_discards = 0;
  wm_queue_max = 0;
  vn_queue_max = 0;
  desync_active = false;

  /* Clear buffers */
  memset(record_buffer_a, 0, RECORD_BUFFER_SIZE);
  memset(record_buffer_b, 0, RECORD_BUFFER_SIZE);
}

void recorder_start(void) {
  /* Check if the filesystem is mounted */
  if (!filesystem_is_mounted()) {
    shell_printf("[REC] ERROR: Filesystem not mounted! Aborting.\r\n");
    return;
  }

  /* Start recording */
  recording = true;
  record_index = 0;
  active_buffer = record_buffer_a;
  flush_buffer = record_buffer_b;
  active_buf_index = 0;
  flush_pending = false;

  /* Generate log filename with timestamp: YYYY-MM-DD-HH-MM-SS.bin */
  uint32_t full_epoch_sec = time_s_now();
  RTC_DateTime_t dt = epoch_to_datetime(full_epoch_sec);
  snprintf(filename, sizeof(filename), "%04d-%02d-%02d-%02d-%02d-%02d.bin",
           dt.years + 2000, dt.months, dt.days,
           dt.hours, dt.minutes, dt.seconds);

  /* Open log file for writing (create it if it doesn't exist) */
  FS_Result_t fs_res = filesystem_open_log(&log_fil, filename);

  /* Check if file open was successful */
  if (fs_res != FS_OK) {
    /* Try fallback open */
    FRESULT fr = f_open(&log_fil, filename, FA_WRITE | FA_CREATE_ALWAYS);

    if (fr != FR_OK) {
      /* File open failed */
      shell_printf("[REC] ERROR: File open failed (code %d)! Aborting.\r\n", fr);
      recording = false;
      return;
    }
  }

  /* Start the sensors */
  shell_printf("[REC] Starting sensors...\r\n");
  wm_start();
  shell_printf("[REC] WindMaster started.\r\n");
  vn_start();
  shell_printf("[REC] VectorNav started.\r\n");

  shell_printf("[REC] Recording started: %s\r\n", filename);
}

void recorder_stop(void) {
  /* Stop recording */
  recording = false;

  /* Stop the sensors */
  wm_stop();
  vn_stop();

  /* Flush any remaining data in the active buffer */
  if (active_buf_index > 0) {
    uint32_t bytes_to_write = active_buf_index * sizeof(Recorder_Data_t);
    uint32_t bytes_written = 0;
    while (bytes_written < bytes_to_write) {
      UINT bw;
      FRESULT res = f_write(&log_fil, active_buffer + bytes_written, bytes_to_write - bytes_written, &bw);
      if (res != FR_OK) {
        shell_printf("[REC] ERROR: File write failed (code %d)! Aborting.\r\n", res);
        break;
      }
      bytes_written += bw;
    }
    active_buf_index = 0;
  }

  /* Sync to ensure all data is written */
  f_sync(&log_fil);

  /* Close the log file */
  f_close(&log_fil);

}

/** @brief Queue a WindMaster packet for recording with timestamp
  * @param pkt: Pointer to WM_Packet_t structure with WindMaster data
  * @retval None
  */
void recorder_queue_wm(const WM_Packet_t *pkt)
{
  /* Equivalent to (wm_q_head + 1) % WM_Q_LEN */
  uint8_t next = (wm_q_head + 1) & (WM_Q_LEN - 1);

  /* Check if queue is full */
  if (next == wm_q_tail) {
    /* Queue full, drop packet and log it */
    wm_drops++;
    desync_active = true;
    clear_queues();
    return;
  }

  /* Capture arrival timestamp */
  systime_snapshot(&wm_queue[wm_q_head].timestamp_s, &wm_queue[wm_q_head].timestamp_ms);

  /* Enqueue the packet */
  wm_queue[wm_q_head].wm_packet = *pkt;
  /* Advance head pointer */
  wm_q_head = next;

  /* DEBUG: Track max queue depth */
  uint8_t count = get_queue_count(wm_q_head, wm_q_tail, WM_Q_LEN);
  if (count > wm_queue_max) {
    /* Update max */
    wm_queue_max = count;
  }
}

/* @brief Queue a VectorNav packet for recording with timestamp
 * @param pkt: Pointer to VN_Packet_t structure with IMU data
 * @retval None
  */
void recorder_queue_vn(const VN_Packet_t *pkt)
{
  /* Equivalent to (vn_q_head + 1) % VN_Q_LEN */
  uint8_t next = (vn_q_head + 1) & (VN_Q_LEN - 1);

  /* Check if queue is full */
  if (next == vn_q_tail) {
    /* Queue full, drop packet and log it */
    vn_drops++;
    desync_active = true;
    clear_queues();
    return;
  }

  /* Capture arrival timestamp */
  systime_snapshot(&vn_queue[vn_q_head].timestamp_s, &vn_queue[vn_q_head].timestamp_ms);

  /* Enqueue the packet */
  vn_queue[vn_q_head].vn_packet = *pkt;

  /* Advance head pointer */
  vn_q_head = next;

  /* DEBUG: Track max queue depth */
  uint8_t count = get_queue_count(vn_q_head, vn_q_tail, VN_Q_LEN);
  if (count > vn_queue_max) {
    /* Update max */
    vn_queue_max = count;
  }
}

void recorder_service(void) {
  if (!recording) {
    /* Not recording, return fast */
    return;
  }

  /* Drain DMA buffers and queue packets */
  wm_drain_and_queue();
  vn_drain_and_queue();

  /* Process up to MAX_RECORDS_PER_SERVICE pairs using nearest-neighbor matching */
  uint8_t records_processed = 0;

  /* WindMaster-driven pairing: process WM packets in order, find nearest VN match */
  while (wm_q_tail != wm_q_head && records_processed < MAX_RECORDS_PER_SERVICE) {

    /* Get next WindMaster packet (always process in order) */
    WM_QueueEntry_t *wm_entry = &wm_queue[wm_q_tail];

    /* Convert WM timestamp to total milliseconds (64-bit to avoid wraparound) */
    uint64_t wm_time_ms = (uint64_t)wm_entry->timestamp_s * 1000ULL + wm_entry->timestamp_ms;

    /* Search VectorNav queue for nearest neighbor */
    int8_t best_vn_idx = -1;
    uint32_t min_offset = UINT32_MAX;

    uint8_t vn_idx = vn_q_tail;
    while (vn_idx != vn_q_head) {
      VN_QueueEntry_t *vn_entry = &vn_queue[vn_idx];

      /* Convert VN timestamp to total milliseconds (64-bit to avoid wraparound) */
      uint64_t vn_time_ms = (uint64_t)vn_entry->timestamp_s * 1000ULL + vn_entry->timestamp_ms;

      /* Calculate signed time difference (64-bit) */
      int64_t diff_ms = (int64_t)wm_time_ms - (int64_t)vn_time_ms;

      /* Calculate absolute offset (safe cast since we expect small offsets) */
      uint32_t offset_ms;
      if (diff_ms >= 0) {
        offset_ms = (uint32_t)diff_ms;
      } else {
        offset_ms = (uint32_t)(-diff_ms);
      }

      /* Track closest match */
      if (offset_ms < min_offset) {
        min_offset = offset_ms;
        best_vn_idx = vn_idx;
      }

      /* Early exit if VN is too far in the future */
      if (diff_ms < -(int64_t)MAX_OFFSET_MS) {
        break;
      }

      /* Advance to next VN packet, equivalent to (vn_idx + 1) % VN_Q_LEN */
      vn_idx = (vn_idx + 1) & (VN_Q_LEN - 1);
    }

    /* Check if match found within tolerance */
    if (best_vn_idx >= 0 && min_offset <= MAX_OFFSET_MS) {
      /* Build record with matched pair */
      VN_QueueEntry_t *vn_entry = &vn_queue[best_vn_idx];

      /* Calculate signed timing offset for recording (64-bit to avoid wraparound) */
      uint64_t vn_time_ms = (uint64_t)vn_entry->timestamp_s * 1000ULL + vn_entry->timestamp_ms;
      int64_t timing_offset_full = (int64_t)wm_time_ms - (int64_t)vn_time_ms;
      int16_t timing_offset_ms = (int16_t)timing_offset_full;  // Safe cast: we verified offset <= 25ms

      Recorder_Data_t record = build_record(vn_entry, wm_entry, timing_offset_ms);

      /* Copy the record into the active buffer */
      Recorder_Data_t* dest = (Recorder_Data_t*)(active_buffer + (active_buf_index * sizeof(Recorder_Data_t)));
      memcpy(dest, &record, sizeof(Recorder_Data_t));

      active_buf_index++;
      records_processed++;

      /* Dequeue matched WindMaster packet, equivalent to (wm_q_tail + 1) % WM_Q_LEN */
      wm_q_tail = (wm_q_tail + 1) & (WM_Q_LEN - 1);

      /* Remove matched VN packet and all older packets */
      uint8_t vn_discard_idx = vn_q_tail;
      while (vn_discard_idx != best_vn_idx) {
        vn_discard_idx = (vn_discard_idx + 1) & (VN_Q_LEN - 1);
        vn_discards++;
      }

      /* Advance past matched packet, equivalent to (best_vn_idx + 1) % VN_Q_LEN */
      vn_q_tail = (best_vn_idx + 1) & (VN_Q_LEN - 1); 

      /* First successful pair after a drop clears desync flag */
      desync_active = false;

      /* Check if active buffer is full (32 records = 4KB) */
      if (active_buf_index >= 32) {
        /* Buffer is full, so swap the active buffer */
        flip_buffers();

        /* Flush the full buffer to the SD card */
        uint32_t bytes_to_write = RECORD_BUFFER_SIZE;
        uint32_t bytes_written = 0;

        while (bytes_written < bytes_to_write) {
          UINT bw;
          FRESULT res = f_write(&log_fil, flush_buffer + bytes_written, bytes_to_write - bytes_written, &bw);
          if (res != FR_OK) {
            shell_printf("[REC] ERROR: File write failed (code %d)! Aborting.\r\n", res);
            break;
          }
          bytes_written += bw;
        }

        flush_pending = false;

        /* After SD write, exit to main loop */
        break;
      }

    } else {
      /* No match found - WM packet is too old or VN queue empty */
      /* If we're in desync recovery, keep flushing to wait for fresh aligned data */
      if (desync_active) {
        clear_queues();
      }
      break;
    }
  }
}
  
/* Private functions -----------------------------------------------*/

/** @brief  Atomically swap active and flush buffers
  * @param  None
  * @retval None
  * @note   Called when active buffer is full (32 records = 4KB)
  */
static void flip_buffers(void) {
  uint8_t* tmp = active_buffer;
  active_buffer = flush_buffer;
  flush_buffer = tmp;
  active_buf_index = 0;
  flush_pending = true;
}

/** @brief  Build a data record from the given VN and WM data
  * @param  vn_entry: Pointer to VN_QueueEntry_t structure with IMU data and timestamp
  * @param  wm_entry: Pointer to WM_QueueEntry_t structure with WindMaster data and timestamp
  * @param  timing_offset_ms: Signed timing offset (WM timestamp - VN timestamp) in milliseconds
  * @retval Recorder_Data_t: Filled recorder data structure
  */
Recorder_Data_t build_record(const VN_QueueEntry_t *vn_entry, const WM_QueueEntry_t *wm_entry, int16_t timing_offset_ms) {
  /* Create and zero-initialize a new record */
  Recorder_Data_t record;
  memset(&record, 0, sizeof(Recorder_Data_t));

  /* Give it a magic number and log index */
  record.magic_number = 0xFACEFACE;
  record.log_index = record_index++;

  /* Use the systime snapshot for timestamping */
  systime_snapshot(&record.epoch_seconds, &record.ms);

  /* Extract pointers to the VN and WM data packets */
  const VN_Packet_t *vn_data = &vn_entry->vn_packet;
  const WM_Packet_t *wm_data = &wm_entry->wm_packet;

  /* Copy data from VN and WM packets into the record */
  record.timegps = vn_data->timegps;
  record.yaw = vn_data->yaw;
  record.pitch = vn_data->pitch;
  record.roll = vn_data->roll;
  record.gyro_x = vn_data->gyro_x;
  record.gyro_y = vn_data->gyro_y;
  record.gyro_z = vn_data->gyro_z;
  record.latitude = vn_data->latitude;
  record.longitude = vn_data->longitude;
  record.altitude = vn_data->altitude;
  record.vel_n = vn_data->vel_n;
  record.vel_e = vn_data->vel_e;
  record.vel_d = vn_data->vel_d;
  record.acc_x = vn_data->acc_x;
  record.acc_y = vn_data->acc_y;
  record.acc_z = vn_data->acc_z;
  record.U_axis_speed = wm_data->U_axis_speed;
  record.V_axis_speed = wm_data->V_axis_speed;
  record.W_axis_speed = wm_data->W_axis_speed;
  record.SoS = wm_data->SoS;
  record.Temp = wm_data->Temp;
  record.timing_offset_ms = timing_offset_ms;

  return record;
}

/** @brief Calculate queue count (number of entries)
  * @param head: Queue head index
  * @param tail: Queue tail index
  * @param len: Queue length
  * @retval Number of entries in queue
  */
static uint8_t get_queue_count(uint8_t head, uint8_t tail, uint8_t len)
{
  /* Equivalent to (head - tail) % len */
  return (uint8_t)((head - tail) & (len - 1));
}

/** @brief Clear both queues and reset queue-related stats
  * @param None
  * @retval None
  */
static void clear_queues(void)
{
  wm_q_head = wm_q_tail = 0;
  vn_q_head = vn_q_tail = 0;
  wm_queue_max = 0;
  vn_queue_max = 0;
}

/** @brief Get recorder statistics
  * @param None
  * @retval recorder_stats_t: Statistics structure
  */
recorder_stats_t recorder_get_stats(void)
{
  /* Create a stats struct */
  recorder_stats_t stats;

  /* Populate recorder and buffer state */
  stats.recording = recording;
  stats.records_written = record_index;
  stats.active_buffer_records = active_buf_index;
  stats.active_buffer_capacity = 32;

  /* Populate queue state */
  stats.wm_queue_count = get_queue_count(wm_q_head, wm_q_tail, WM_Q_LEN);
  stats.wm_queue_capacity = WM_Q_LEN;
  stats.vn_queue_count = get_queue_count(vn_q_head, vn_q_tail, VN_Q_LEN);
  stats.vn_queue_capacity = VN_Q_LEN;

  /* Populate drop and max queue stats */
  stats.wm_queue_max = wm_queue_max;
  stats.vn_queue_max = vn_queue_max;
  stats.wm_drops = wm_drops;
  stats.vn_drops = vn_drops;
  stats.vn_discards = vn_discards;

  /* Copy filename safely */
  strncpy(stats.filename, filename, sizeof(stats.filename) - 1);
  stats.filename[sizeof(stats.filename) - 1] = '\0';

  return stats;
}

/** @brief Print queue debug information (timestamps and offsets)
  * @param None
  * @retval None
  * @note Shows first 3 entries from each queue to diagnose pairing issues
  */
void recorder_debug_queue(void)
{
  shell_print("\r\n=== Queue Debug Info ===\r\n");

  /* WindMaster queue */
  uint8_t wm_count = get_queue_count(wm_q_head, wm_q_tail, WM_Q_LEN);
  shell_printf("WM Queue: %u entries (head=%u, tail=%u)\r\n", wm_count, wm_q_head, wm_q_tail);

  if (wm_count > 0) {
    /* Show first 3 entries or all if fewer than 3 */
    uint8_t show_count = (wm_count > 3) ? 3 : wm_count;

    /* Loop through first few entries */
    for (uint8_t i = 0; i < show_count; i++) {
      /* Advance index with wraparound */
      uint8_t idx = (wm_q_tail + i) & (WM_Q_LEN - 1);
      
      /* Print WindMaster queue entry timestamp */
      shell_printf("  WM[%u]: t=%lu.%03u s\r\n",
                   idx,
                   wm_queue[idx].timestamp_s,
                   wm_queue[idx].timestamp_ms);
    }
  } else {
    shell_print("  (empty)\r\n");
  }

  /* VectorNav queue */
  uint8_t vn_count = get_queue_count(vn_q_head, vn_q_tail, VN_Q_LEN);
  shell_printf("\r\nVN Queue: %u entries (head=%u, tail=%u)\r\n", vn_count, vn_q_head, vn_q_tail);

  if (vn_count > 0) {
    uint8_t show_count = (vn_count > 3) ? 3 : vn_count;
    for (uint8_t i = 0; i < show_count; i++) {
      uint8_t idx = (vn_q_tail + i) & (VN_Q_LEN - 1);
      shell_printf("  VN[%u]: t=%lu.%03u s\r\n",
                   idx,
                   vn_queue[idx].timestamp_s,
                   vn_queue[idx].timestamp_ms);
    }
  } else {
    shell_print("  (empty)\r\n");
  }

  /* Calculate offset between first entries if both exist */
  if (wm_count > 0 && vn_count > 0) {
    /* Use 64-bit timestamps to avoid wraparound */
    uint64_t wm_time_ms = (uint64_t)wm_queue[wm_q_tail].timestamp_s * 1000ULL + wm_queue[wm_q_tail].timestamp_ms;
    uint64_t vn_time_ms = (uint64_t)vn_queue[vn_q_tail].timestamp_s * 1000ULL + vn_queue[vn_q_tail].timestamp_ms;
    int64_t diff_ms = (int64_t)wm_time_ms - (int64_t)vn_time_ms;

    shell_printf("\r\nFirst entry offset: WM - VN = %lld ms\r\n", diff_ms);

    if (diff_ms < 0) {
      shell_printf("  (WM is %lld ms BEFORE VN)\r\n", -diff_ms);
    } else {
      shell_printf("  (WM is %lld ms AFTER VN)\r\n", diff_ms);
    }

    uint32_t abs_diff = (diff_ms >= 0) ? (uint32_t)diff_ms : (uint32_t)(-diff_ms);
    if (abs_diff <= MAX_OFFSET_MS) {
      shell_printf("  [OK] Within %ums tolerance for pairing\r\n", MAX_OFFSET_MS);
    } else {
      shell_printf("  [FAIL] Exceeds %ums tolerance - will NOT pair\r\n", MAX_OFFSET_MS);
    }
  }

  shell_print("========================\r\n");
}

/** @brief Clear recorder statistics
  * @param None
  * @retval None
  */
void recorder_clear_stats(void)
{
  wm_drops = 0;
  vn_drops = 0;
  vn_discards = 0;
  wm_queue_max = 0;
  vn_queue_max = 0;
}

/** @brief Check if recorder is currently recording
  * @param None
  * @retval bool: true if recording, false otherwise
  */
bool recorder_is_recording(void)
{
  return recording;
}
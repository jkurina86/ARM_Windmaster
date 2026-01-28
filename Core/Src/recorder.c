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
#include "calculations.h"
#include <stdbool.h>

/* Definitions ------------------------------------------------------------------*/
#define WM_LEN 23
#define VN_LEN 86
#define WM_Q_LEN 16 /* Must be power of two, ~800ms buffer @20Hz */
#define VN_Q_LEN 16 /* Must be power of two, ~320ms buffer @50Hz */
#define RECORD_BUFFER_SIZE 4096
#define RECORD_BUFFER_CAPACITY (RECORD_BUFFER_SIZE / sizeof(Recorder_Data_t))  /* 32 records */
#define MAX_RECORDS_PER_SERVICE 32  /* 32*128 = 4KB, Only one full buffer will be processed at a time at most. */
#define MAX_OFFSET_MS 25  /* 25ms tolerance for nearest-neighbor pairing (worst case phase offset between async sensors) */
#define LOG_ROLLOVER_BYTES (20 * 1024 * 1024) /* 20MB per log file to limit corruption window and transfer size */
#define RECORDER_MAGIC_NUMBER 0xFACEFACE

/* Queue Structures, location in RAM2 specified in linker script */
static WM_QueueEntry_t wm_queue[WM_Q_LEN] __attribute__((section(".queue_wm")));
static VN_QueueEntry_t vn_queue[VN_Q_LEN] __attribute__((section(".queue_vn")));

/* Queue indices - volatile because modified in ISR context (recorder_queue_wm/vn)
 * and read from main loop (recorder_service) */
static volatile uint8_t wm_q_head = 0;
static volatile uint8_t wm_q_tail = 0;
static volatile uint8_t vn_q_head = 0;
static volatile uint8_t vn_q_tail = 0;

/* Desynchronization flag - gets set to true to clear the queues after a packet drop */
static bool desync_active = false; 

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

/* Log rollover and size tracking */
static uint32_t log_bytes_written = 0;

/* Private function prototypes -----------------------------------------------*/
static void flip_buffers(void);
static Recorder_Data_t build_record(const VN_QueueEntry_t *vn_entry, const WM_QueueEntry_t *wm_entry, int16_t timing_offset_ms);
static uint8_t get_queue_count(uint8_t head, uint8_t tail, uint8_t len);
static void clear_queues(void);
static void handle_write_error(FRESULT res, const char* where);
static bool open_log_file(void);
static bool write_with_rollover(const uint8_t* data, uint32_t len, const char* where);

/* Helper functions for code simplification */
static inline uint64_t timestamp_to_ms(uint32_t s, uint16_t ms);
static inline uint8_t queue_next_idx(uint8_t idx, uint8_t capacity);
static inline uint32_t abs64_to_u32(int64_t val);
static inline void update_queue_max(uint8_t count, uint8_t* max_ptr);

/* Extracted functions from recorder_service() for better separation of concerns */
static bool find_nearest_vn_match(uint64_t wm_time_ms, int8_t* out_vn_idx, uint32_t* out_offset_ms);
static void discard_vn_entries_up_to(uint8_t target_idx);
static bool buffer_record(VN_QueueEntry_t* vn_entry, WM_QueueEntry_t* wm_entry, int16_t timing_offset_ms);

/** @brief Initialize the recorder module
  * @retval None
  */
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

  /* Clear buffers */
  memset(record_buffer_a, 0, RECORD_BUFFER_SIZE);
  memset(record_buffer_b, 0, RECORD_BUFFER_SIZE);
}

/** @brief Start the recorder: initialize and begin recording
  * @retval None
  */
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
  log_bytes_written = 0;

  /* Open initial log file with timestamped name */
  if (!open_log_file()) {
    recording = false;
    return;
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
    if (!write_with_rollover(active_buffer, bytes_to_write, "stop flush")) {
      return;
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
  /* Check if queue is full */
  uint8_t next = queue_next_idx(wm_q_head, WM_Q_LEN);
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
  update_queue_max(count, &wm_queue_max);
}

/** @brief Queue a VectorNav packet for recording with timestamp
  * @param pkt: Pointer to VN_Packet_t structure with IMU data
  * @retval None
  */
void recorder_queue_vn(const VN_Packet_t *pkt)
{
  /* Check if queue is full */
  uint8_t next = queue_next_idx(vn_q_head, VN_Q_LEN);
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
  update_queue_max(count, &vn_queue_max);
}

/** @brief Service the recorder: process queues and write data to SD card
  * @retval None
  */
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
    uint64_t wm_time_ms = timestamp_to_ms(wm_entry->timestamp_s, wm_entry->timestamp_ms);

    /* Search VectorNav queue for nearest neighbor */
    int8_t best_vn_idx;
    uint32_t min_offset;
    bool match_found = find_nearest_vn_match(wm_time_ms, &best_vn_idx, &min_offset);

    /* Check if match found within tolerance */
    if (match_found) {
      /* Build record with matched pair */
      VN_QueueEntry_t *vn_entry = &vn_queue[best_vn_idx];

      /* Calculate signed timing offset for recording */
      uint64_t vn_time_ms = timestamp_to_ms(vn_entry->timestamp_s, vn_entry->timestamp_ms);
      int64_t timing_offset_full = (int64_t)wm_time_ms - (int64_t)vn_time_ms;
      
      /* Safe to cast to int16_t since offset <= MAX_OFFSET_MS */
      int16_t timing_offset_ms = (int16_t)timing_offset_full;

      /* Build and buffer the record */
      if (!buffer_record(vn_entry, wm_entry, timing_offset_ms)) {
        return;
      }

      records_processed++;

      /* Dequeue matched WindMaster packet */
      wm_q_tail = queue_next_idx(wm_q_tail, WM_Q_LEN);

      /* Remove matched VN packet and all older packets */
      discard_vn_entries_up_to(best_vn_idx);

      /* First successful pair after a drop clears desync flag */
      desync_active = false;

      /* After buffer flip, exit to main loop to allow other tasks to run */
      if (flush_pending) {
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
  
/* Helper functions -----------------------------------------------*/

/** @brief  Convert seconds and milliseconds to total milliseconds (64-bit)
  * @param  s: Seconds from systime_snapshot()
  * @param  ms: Milliseconds from systime_snapshot() (0-999)
  * @retval Total milliseconds as 64-bit unsigned integer
  */
static inline uint64_t timestamp_to_ms(uint32_t s, uint16_t ms)
{
  return (uint64_t)s * 1000ULL + ms;
}

/** @brief  Calculate next circular queue index with power-of-two wrapping
  * @param  idx: Current index
  * @param  capacity: Queue capacity (must be power of two)
  * @retval Next index with wraparound
  */
static inline uint8_t queue_next_idx(uint8_t idx, uint8_t capacity)
{
  return (idx + 1) & (capacity - 1);
}

/** @brief  Calculate absolute value of int64_t and safely cast to uint32_t
  * @param  val: Signed 64-bit value
  * @retval Absolute value as unsigned 32-bit integer
  * @note   Safe to cast since we expect small offsets (< MAX_OFFSET_MS)
  */
static inline uint32_t abs64_to_u32(int64_t val)
{
  return (val >= 0) ? (uint32_t)val : (uint32_t)(-val);
}

/** @brief  Update queue max depth tracking
  * @param  count: Current queue count
  * @param  max_ptr: Pointer to max tracking variable
  * @retval None
  */
static inline void update_queue_max(uint8_t count, uint8_t* max_ptr)
{
  if (count > *max_ptr) {
    *max_ptr = count;
  }
}

/* Extracted functions from recorder_service() -----------------------------------------------*/

/** @brief  Find nearest VectorNav packet to match with WindMaster timestamp
  * @param  wm_time_ms: WindMaster timestamp in milliseconds (64-bit)
  * @param  out_vn_idx: Output parameter for best matching VN queue index
  * @param  out_offset_ms: Output parameter for timing offset in milliseconds
  * @retval true if match found within tolerance, false otherwise
  */
static bool find_nearest_vn_match(uint64_t wm_time_ms, int8_t* out_vn_idx, uint32_t* out_offset_ms)
{
  int8_t best_vn_idx = -1;
  uint32_t min_offset = UINT32_MAX;
  uint8_t vn_idx = vn_q_tail;

  while (vn_idx != vn_q_head) {
    VN_QueueEntry_t *vn_entry = &vn_queue[vn_idx];
    uint64_t vn_time_ms = timestamp_to_ms(vn_entry->timestamp_s, vn_entry->timestamp_ms);
    int64_t diff_ms = (int64_t)wm_time_ms - (int64_t)vn_time_ms;
    uint32_t offset_ms = abs64_to_u32(diff_ms);

    if (offset_ms < min_offset) {
      min_offset = offset_ms;
      best_vn_idx = vn_idx;
    }

    /* Early exit if VN is too far in the future */
    if (diff_ms < -(int64_t)MAX_OFFSET_MS) {
      break;
    }

    vn_idx = queue_next_idx(vn_idx, VN_Q_LEN);
  }

  *out_vn_idx = best_vn_idx;
  *out_offset_ms = min_offset;
  return (best_vn_idx >= 0 && min_offset <= MAX_OFFSET_MS);
}

/** @brief  Discard VN queue entries up to (but not including) target index
  * @param  target_idx: Target index to discard up to
  * @retval None
  * @note   Updates vn_discards counter and advances vn_q_tail
  */
static void discard_vn_entries_up_to(uint8_t target_idx)
{
  uint8_t vn_discard_idx = vn_q_tail;
  while (vn_discard_idx != target_idx) {
    vn_discard_idx = queue_next_idx(vn_discard_idx, VN_Q_LEN);
    vn_discards++;
  }
  vn_q_tail = queue_next_idx(target_idx, VN_Q_LEN);
}

/** @brief  Build and buffer a record from matched VN and WM entries
  * @param  vn_entry: Pointer to VN queue entry
  * @param  wm_entry: Pointer to WM queue entry
  * @param  timing_offset_ms: Signed timing offset in milliseconds
  * @retval true on success, false if buffer write failed
  * @note   Handles buffer flipping and SD writing when buffer is full
  */
static bool buffer_record(VN_QueueEntry_t* vn_entry, WM_QueueEntry_t* wm_entry, int16_t timing_offset_ms)
{
  Recorder_Data_t record = build_record(vn_entry, wm_entry, timing_offset_ms);
  Recorder_Data_t* dest = (Recorder_Data_t*)active_buffer + active_buf_index;
  memcpy(dest, &record, sizeof(Recorder_Data_t));

  /* Feed paired data to calculations module (with attitude + gyro for motion correction) */
  CalcSample_t calc_sample = {
    .u = wm_entry->wm_packet.U_axis_speed,
    .v = wm_entry->wm_packet.V_axis_speed,
    .w = wm_entry->wm_packet.W_axis_speed,
    .roll = vn_entry->vn_packet.roll,
    .pitch = vn_entry->vn_packet.pitch,
    .yaw = vn_entry->vn_packet.yaw,
    .gyro_x = vn_entry->vn_packet.gyro_x,
    .gyro_y = vn_entry->vn_packet.gyro_y,
    .gyro_z = vn_entry->vn_packet.gyro_z,
    .latitude = vn_entry->vn_packet.latitude,
    .longitude = vn_entry->vn_packet.longitude
  };
  calc_add_sample(&calc_sample);

  active_buf_index++;
  
  /* Check if active buffer is full */
  if (active_buf_index >= RECORD_BUFFER_CAPACITY) {
    /* Buffer is full, so swap the active buffer */
    flip_buffers();

    /* Flush the full buffer to the SD card (with rollover) */
    if (!write_with_rollover(flush_buffer, RECORD_BUFFER_SIZE, "service flush")) {
      return false;
    }

    flush_pending = false;
  }
  
  return true;
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
  record.magic_number = RECORDER_MAGIC_NUMBER;
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
  return (head - tail) & (len - 1);
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

/** @brief Open a log file with rollover-aware name
  * @param sequence: Rollover index to append to base filename
  * @retval true on success, false on failure
  */
static bool open_log_file(void)
{
  /* Build timestamped filename: YYYY-MM-DD-HH-MM-SS.bin */
  uint32_t full_epoch_sec = time_s_now();
  RTC_DateTime_t dt = epoch_to_datetime(full_epoch_sec);
  snprintf(filename, sizeof(filename), "%04d-%02d-%02d-%02d-%02d-%02d.bin",
           dt.years + 2000, dt.months, dt.days,
           dt.hours, dt.minutes, dt.seconds);

  log_bytes_written = 0;

  FS_Result_t fs_res = filesystem_open_log(&log_fil, filename);
  if (fs_res == FS_OK) {
    return true;
  }

  FRESULT fr = f_open(&log_fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
  if (fr == FR_OK) {
    return true;
  }

  shell_printf("[REC] ERROR: File open failed (code %d)! Aborting.\r\n", fr);
  return false;
}

/** @brief Handle SD write errors by stopping recording and sensors
  * @param res: FRESULT error code from failed write
  * @param where: Description of where the error occurred
  * @retval None
  */
static void handle_write_error(FRESULT res, const char* where)
{
  shell_printf("[REC] ERROR: SD write failed (%s, code %d). Stopping recording.\r\n", where, res);

  /* Prevent further writes and stop sensors */
  recording = false;
  flush_pending = false;
  wm_stop();
  vn_stop();

  /* Best-effort close to avoid leaving the file open */
  f_sync(&log_fil);
  f_close(&log_fil);
}

/** @brief Write data to SD with rollover and periodic sync
  * @param data: Pointer to data buffer
  * @param len: Number of bytes to write
  * @param where: Context string for error reporting
  * @retval true on success, false on failure
  */
static bool write_with_rollover(const uint8_t* data, uint32_t len, const char* where)
{
  uint32_t remaining = len;

  while (remaining > 0) {
    if (log_bytes_written >= LOG_ROLLOVER_BYTES) {
      f_sync(&log_fil);
      f_close(&log_fil);
      if (!open_log_file()) {
        handle_write_error(FR_DISK_ERR, "rollover open");
        return false;
      }
    }

    uint32_t space = LOG_ROLLOVER_BYTES - log_bytes_written;
    uint32_t chunk = (remaining < space) ? remaining : space;

    UINT bw = 0;
    FRESULT res = f_write(&log_fil, data, chunk, &bw);
    if (res != FR_OK || bw != chunk) {
      handle_write_error(res != FR_OK ? res : FR_DISK_ERR, where);
      return false;
    }

    log_bytes_written += bw;
    remaining -= bw;
    data += bw;
  }

  /* Persist periodically to reduce corruption risk */
  f_sync(&log_fil);
  return true;
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
  stats.active_buffer_capacity = RECORD_BUFFER_CAPACITY;

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
    uint64_t wm_time_ms = timestamp_to_ms(wm_queue[wm_q_tail].timestamp_s, wm_queue[wm_q_tail].timestamp_ms);
    uint64_t vn_time_ms = timestamp_to_ms(vn_queue[vn_q_tail].timestamp_s, vn_queue[vn_q_tail].timestamp_ms);
    int64_t diff_ms = (int64_t)wm_time_ms - (int64_t)vn_time_ms;

    shell_printf("\r\nFirst entry offset: WM - VN = %lld ms\r\n", diff_ms);

    if (diff_ms < 0) {
      shell_printf("  (WM is %lld ms BEFORE VN)\r\n", -diff_ms);
    } else {
      shell_printf("  (WM is %lld ms AFTER VN)\r\n", diff_ms);
    }

    uint32_t abs_diff = abs64_to_u32(diff_ms);
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
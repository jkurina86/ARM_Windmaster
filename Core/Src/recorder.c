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
#define WM_Q_LEN 64
#define VN_Q_LEN 64
#define RECORD_BUFFER_SIZE 4096  
#define MAX_RECORDS_PER_SERVICE 32  /* 32*128 = 4KB, Only one full buffer will be processed at a time at most. */

/* Queue Structures */
static WM_QueueEntry_t wm_queue[WM_Q_LEN];
static VN_QueueEntry_t vn_queue[VN_Q_LEN];

/* Queue indices */
static uint8_t wm_q_head = 0;
static uint8_t wm_q_tail = 0;
static uint8_t vn_q_head = 0;
static uint8_t vn_q_tail = 0;

uint8_t record_buffer_a[RECORD_BUFFER_SIZE]__attribute__((section(".record_buffer_a")));
uint8_t record_buffer_b[RECORD_BUFFER_SIZE]__attribute__((section(".record_buffer_b")));

/* Buffer management state */
static uint8_t* active_buffer = record_buffer_a;     // Currently being filled
static uint8_t* flush_buffer = record_buffer_b;      // Ready to write to SD
static uint32_t active_buf_index = 0;                // Number of records in active buffer (0-31)
static bool flush_pending = false;                   // True if flush in progress

static bool recording = false;
static uint32_t record_index = 0;
static FIL log_fil;  // File object for logging

char filename[128];

/* Statistics tracking */
static uint32_t wm_drops = 0;
static uint32_t vn_drops = 0;
static uint8_t wm_queue_max = 0;
static uint8_t vn_queue_max = 0;

/* Private function prototypes -----------------------------------------------*/
static void flip_buffers(void);
static Recorder_Data_t build_record(const VN_QueueEntry_t *vn_entry, const WM_QueueEntry_t *wm_entry);
static uint8_t get_queue_count(uint8_t head, uint8_t tail, uint8_t len);


void recorder_init(void) {
  /* Initialize recording state */
  recording = false;
  record_index = 0;
  active_buffer = record_buffer_a;
  flush_buffer = record_buffer_b;
  active_buf_index = 0;
  flush_pending = false;

  /* Initialize statistics */
  wm_drops = 0;
  vn_drops = 0;
  wm_queue_max = 0;
  vn_queue_max = 0;

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

  if (fs_res != FS_OK) {
    FRESULT fr = f_open(&log_fil, filename, FA_WRITE | FA_CREATE_ALWAYS);

    if (fr != FR_OK) {
      shell_printf("[REC] ERROR: File open failed (code %d)! Aborting.\r\n", fr);
      recording = false;
      return;
    }
  }

  /* Start the sensors */
  wm_start();
  vn_start();
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
        /* Handle error */
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

/* @brief Queue a WindMaster packet for recording
 * @param pkt: Pointer to WM_Packet_t structure with WindMaster data
 * @param t_ms: Timestamp in milliseconds
 * @retval None
  */
void recorder_queue_wm(const WM_Packet_t *pkt)
{
  uint8_t next = (wm_q_head + 1) & (WM_Q_LEN - 1);
  if (next == wm_q_tail) {
    /* Queue full, drop packet and log */
    wm_drops++;
    return;
  }

  wm_queue[wm_q_head].wm_packet = *pkt;

  wm_q_head = next;

  /* Track max queue depth */
  uint8_t count = get_queue_count(wm_q_head, wm_q_tail, WM_Q_LEN);
  if (count > wm_queue_max) {
    wm_queue_max = count;
  }
}

/* @brief Queue a VectorNav packet for recording
 * @param pkt: Pointer to VN_Packet_t structure with IMU data
 * @param t_ms: Timestamp in milliseconds
 * @retval None
  */
void recorder_queue_vn(const VN_Packet_t *pkt)
{
  uint8_t next = (vn_q_head + 1) & (VN_Q_LEN - 1);
  if (next == vn_q_tail) {
    vn_drops++; // Track queue overflow
    return; // queue full
  }

  vn_queue[vn_q_head].vn_packet = *pkt;

  vn_q_head = next;

  /* Track max queue depth */
  uint8_t count = get_queue_count(vn_q_head, vn_q_tail, VN_Q_LEN);
  if (count > vn_queue_max) {
    vn_queue_max = count;
  }
}

void recorder_service(void) {
  if (!recording) {
    return;
  }

  /* Drain DMA buffers and queue packets */
  wm_drain_and_queue();
  vn_drain_and_queue();

  /* Process up to MAX_RECORDS_PER_SERVICE pairs and then yield to allow
   * cooperative multitasking with shell and tasker in main loop.
   */
  uint8_t records_processed = 0;

  while (wm_q_tail != wm_q_head &&
         vn_q_tail != vn_q_head &&
         records_processed < MAX_RECORDS_PER_SERVICE) {

    WM_QueueEntry_t *wm = &wm_queue[wm_q_tail];
    VN_QueueEntry_t *vn = &vn_queue[vn_q_tail];

    /* Pair the WindMaster and VectorNav data */
    Recorder_Data_t record = build_record(vn, wm);

    /* Copy the record into the active buffer at the current position */
    Recorder_Data_t* dest = (Recorder_Data_t*)(active_buffer + (active_buf_index * sizeof(Recorder_Data_t)));
    memcpy(dest, &record, sizeof(Recorder_Data_t));

    active_buf_index++;
    records_processed++;

    /* Dequeue both packets (advance tail pointers) */
    wm_q_tail = (wm_q_tail + 1) & (WM_Q_LEN - 1);
    vn_q_tail = (vn_q_tail + 1) & (VN_Q_LEN - 1);

    /* Check if active buffer is full (32 records = 4KB) */
    if (active_buf_index >= 32) {
      /* Swap buffers */
      flip_buffers();

      /* Write the flush buffer to SD card */
      uint32_t bytes_to_write = RECORD_BUFFER_SIZE;
      uint32_t bytes_written = 0;

      while (bytes_written < bytes_to_write) {
        UINT bw;
        FRESULT res = f_write(&log_fil, flush_buffer + bytes_written, bytes_to_write - bytes_written, &bw);
        if (res != FR_OK) {
          // Handle write error (optional: log or set error flag)
          break;
        }
        bytes_written += bw;
      }

      flush_pending = false;

      /* After SD write, exit this service call to allow main loop to service shell/tasker.
       * Next call to recorder_service() will continue draining queues if needed. */
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

/* @brief  Build a data record from the given VN and WM data
  * @param  vn_data: Pointer to VN_Packet_t structure with IMU data
  * @param  wm_data: Pointer to WM_Packet_t structure with WindMaster data
  * @retval Recorder_Data_t: Filled recorder data structure
  */
Recorder_Data_t build_record(const VN_QueueEntry_t *vn_entry, const WM_QueueEntry_t *wm_entry) {
  Recorder_Data_t record;
  memset(&record, 0, sizeof(Recorder_Data_t));

  record.magic_number = 0xFACEFACE; // Unique identifier for the start of a record
  record.log_index = record_index++;
  
  /* Use the systime snapshot for timestamping */
  systime_snapshot(&record.epoch_seconds, &record.ms);

  const VN_Packet_t *vn_data = &vn_entry->vn_packet;
  const WM_Packet_t *wm_data = &wm_entry->wm_packet;

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

    return record;
}

/* @brief Calculate queue count (number of entries)
  * @param head: Queue head index
  * @param tail: Queue tail index
  * @param len: Queue length
  * @retval Number of entries in queue
  */
static uint8_t get_queue_count(uint8_t head, uint8_t tail, uint8_t len)
{
  return (uint8_t)((head - tail) & (len - 1));
}

/* @brief Get recorder statistics
  * @param None
  * @retval recorder_stats_t: Statistics structure
  */
recorder_stats_t recorder_get_stats(void)
{
  recorder_stats_t stats;
  
  stats.recording = recording;
  stats.records_written = record_index;
  stats.active_buffer_records = active_buf_index;
  stats.active_buffer_capacity = 32; /* 4096 / 128 bytes */
  
  stats.wm_queue_count = get_queue_count(wm_q_head, wm_q_tail, WM_Q_LEN);
  stats.wm_queue_capacity = WM_Q_LEN;
  stats.vn_queue_count = get_queue_count(vn_q_head, vn_q_tail, VN_Q_LEN);
  stats.vn_queue_capacity = VN_Q_LEN;
  
  stats.wm_queue_max = wm_queue_max;
  stats.vn_queue_max = vn_queue_max;
  stats.wm_drops = wm_drops;
  stats.vn_drops = vn_drops;
  
  strncpy(stats.filename, filename, sizeof(stats.filename) - 1);
  stats.filename[sizeof(stats.filename) - 1] = '\0';
  
  return stats;
}

/*  @brief Clear recorder statistics
  * @param None
  * @retval None
  */
void recorder_clear_stats(void) 
{
  wm_drops = 0;
  vn_drops = 0;
  wm_queue_max = 0;
  vn_queue_max = 0;
}

/*  @brief Check if recorder is currently recording
  * @param None
  * @retval bool: true if recording, false otherwise
  */
bool recorder_is_recording(void)
{
  return recording;
}
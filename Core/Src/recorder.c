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
#include <stdbool.h>

/* Definitions ------------------------------------------------------------------*/
#define WM_LEN 23
#define VN_LEN 86
#define WM_Q_LEN 64
#define VN_Q_LEN 64
#define RECORD_BUFFER_SIZE 4096  // 4 KB buffer size

/* Queue Structures */
static WM_QueueEntry_t wm_queue[WM_Q_LEN];
static VN_QueueEntry_t vn_queue[VN_Q_LEN];

/* Queue indices */
static volatile uint8_t wm_q_head = 0;
static volatile uint8_t wm_q_tail = 0;
static volatile uint8_t vn_q_head = 0;
static volatile uint8_t vn_q_tail = 0;

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

char filename[64];

/* Private function prototypes -----------------------------------------------*/
static void flip_buffers(void);
static Recorder_Data_t build_record(VN_Packet_t *vn_data, WM_Packet_t *wm_data);


void recorder_init(void) {
  /* Initialize recording state */
  recording = false;
  record_index = 0;
  active_buffer = record_buffer_a;
  flush_buffer = record_buffer_b;
  active_buf_index = 0;
  flush_pending = false;

  /* Clear buffers */
  memset(record_buffer_a, 0, RECORD_BUFFER_SIZE);
  memset(record_buffer_b, 0, RECORD_BUFFER_SIZE);
}

void recorder_start(void) {
  /* Start recording */
  recording = true;
  record_index = 0;
  active_buffer = record_buffer_a;
  flush_buffer = record_buffer_b;
  active_buf_index = 0;
  flush_pending = false;

  /* Generate log filename with timestamp */
  snprintf(filename, sizeof(filename), "log_%08llu.bin", time_s_now());

  /* Open log file for appending */
  filesystem_open_log(&log_fil, filename);

  /* Enable the IDLE interrupts */
  LL_USART_EnableIT_IDLE(UART4);
  LL_USART_EnableIT_IDLE(UART5);

  /* Send a start command to the sensors */
  wm_start();
  vn_start();

}

void recorder_stop(void) {
  /* Stop recording */
  recording = false;

  /* Flush any remaining data in the active buffer */
  if (active_buf_index > 0) {
    uint32_t bytes_to_write = active_buf_index * sizeof(Recorder_Data_t);
    uint32_t bytes_written = 0;
    while (bytes_written < bytes_to_write) {
      UINT bw;
      FRESULT res = f_write(&log_fil, active_buffer + bytes_written, bytes_to_write - bytes_written, &bw);
      if (res != FR_OK) {
        // Handle write error (optional)
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

  /* Disable the IDLE interrupts */
  LL_USART_DisableIT_IDLE(UART4);
  LL_USART_DisableIT_IDLE(UART5);

}

/* @brief Queue a WindMaster packet for recording 
 * @param pkt: Pointer to WM_Packet_t structure with WindMaster data
 * @param t_us: Timestamp in microseconds
 * @retval None
  */
void recorder_queue_wm(const WM_Packet_t *pkt, uint64_t t_us)
{
  uint8_t next = (wm_q_head + 1) & (WM_Q_LEN - 1);
  if (next == wm_q_tail) return; // queue full
  wm_queue[wm_q_head].wm_packet = *pkt;
  wm_queue[wm_q_head].timestamp_us = t_us;
  wm_q_head = next;
}

/* @brief Queue a VectorNav packet for recording 
 * @param pkt: Pointer to VN_Packet_t structure with IMU data
 * @param t_us: Timestamp in microseconds
 * @retval None
  */
void recorder_queue_vn(const VN_Packet_t *pkt, uint64_t t_us)
{
  uint8_t next = (vn_q_head + 1) & (VN_Q_LEN - 1);
  if (next == vn_q_tail) return; // queue full
  vn_queue[vn_q_head].vn_packet = *pkt;
  vn_queue[vn_q_head].timestamp_us = t_us;
  vn_q_head = next;
}

void recorder_service(void) {
  if (!recording) {
    return;
  }

  /* Process all available pairs from the queues */
  while (wm_q_tail != wm_q_head && vn_q_tail != vn_q_head) {
    WM_QueueEntry_t *wm = &wm_queue[wm_q_tail];
    VN_QueueEntry_t *vn = &vn_queue[vn_q_tail];

    /* Pair the WindMaster and VectorNav data */
    Recorder_Data_t record = build_record(&vn->vn_packet, &wm->wm_packet);

    /* TEMPORARY: for testing, average the timestamps and put it in the timegps field */
    record.timegps = (wm->timestamp_us + vn->timestamp_us) / 2;

    /* Copy the record into the active buffer at the current position */
    Recorder_Data_t* dest = (Recorder_Data_t*)(active_buffer + (active_buf_index * sizeof(Recorder_Data_t)));
    memcpy(dest, &record, sizeof(Recorder_Data_t));
    
    active_buf_index++;
    record_index++;

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
Recorder_Data_t build_record(VN_Packet_t *vn_data, WM_Packet_t *wm_data) {
    Recorder_Data_t record;
    memset(&record, 0, sizeof(Recorder_Data_t));

    record.magic_number = 0xFACEFACE; // Unique identifier for the start of a record
    record.log_index = record_index++;
    
    record.timegps = vn_data->timegps;
    record.yaw = vn_data->yaw;
    record.pitch = vn_data->pitch;
    record.roll = vn_data->roll;
    record.vel_n = vn_data->vel_n;
    record.vel_e = vn_data->vel_e;
    record.vel_d = vn_data->vel_d;
    record.acc_x = vn_data->acc_x;
    record.acc_y = vn_data->acc_y;
    record.acc_z = vn_data->acc_z;
    record.gyro_x = vn_data->gyro_x;
    record.gyro_y = vn_data->gyro_y;
    record.gyro_z = vn_data->gyro_z;
    record.latitude = vn_data->latitude;
    record.longitude = vn_data->longitude;
    record.altitude = vn_data->altitude;

    record.U_axis_speed = wm_data->U_axis_speed;
    record.V_axis_speed = wm_data->V_axis_speed;
    record.W_axis_speed = wm_data->W_axis_speed;
    record.SoS = wm_data->SoS;
    record.Temp = wm_data->Temp;

    return record;
}
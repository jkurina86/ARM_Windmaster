/**
  ******************************************************************************
  * @file    task_rec.c
  * @brief   Recorder task handlers implementation
  * @note    Contains handlers for the recorder task
  ******************************************************************************
  */

#include "task_rec.h"
#include "recorder.h"
#include "shell.h"
#include "tasker.h"

/* Scheduler Functions -------------------------------------------------------*/

/**
  * @brief Schedule a recorder start task
  * @param None
  * @retval None
  */
void schedule_rec_start(void)
{
    tasker_schedule_task(TASK_REC_START, NULL);
}

/**
  * @brief Schedule a recorder stop task
  * @param None
  * @retval None
  */
void schedule_rec_stop(void)
{
    tasker_schedule_task(TASK_REC_STOP, NULL);
}

/**
  * @brief Schedule a recorder statistics task
  * @param None
  * @retval None
  */
void schedule_rec_stats(void)
{
    tasker_schedule_task(TASK_REC_STATS, NULL);
}

/* Task Handler Functions ----------------------------------------------------*/

/**
  * @brief Start recording task handler
  * @param task_data: Pointer to task data (unused for this task)
  * @retval None
  * @note Called from main loop via tasker
  * @note Shell output BEFORE recorder_start() to avoid UART deadlock
  */
void handle_rec_start(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_REC_START);

    /* Print messages BEFORE starting recorder to avoid UART interrupt conflicts */
    shell_print("Starting recorder...\r\n");
    shell_print("Recorder started! Use rec-stop to stop recording.\r\n");
    shell_print(SHELL_PROMPT);

    /* Now start recorder - sensors will begin transmitting immediately */
    recorder_start();
}

/**
  * @brief Stop recording task handler
  * @param task_data: Pointer to task data (unused for this task)
  * @retval None
  * @note Called from main loop via tasker
  */
void handle_rec_stop(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_REC_STOP);
    
    shell_print("Stopping recorder...\r\n");
    recorder_stop();
    shell_print("Recorder stopped!\r\n");
    shell_print(SHELL_PROMPT);
}

/**
  * @brief Display recorder statistics task handler
  * @param task_data: Pointer to task data (unused for this task)
  * @retval None
  * @note Called from main loop via tasker
  */
void handle_rec_stats(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_REC_STATS);
    
    // Get statistics from recorder
    recorder_stats_t stats = recorder_get_stats();
    
    shell_print("\r\n");
    shell_print("===============================================\r\n");
    shell_print("       RECORDER STATISTICS\r\n");
    shell_print("===============================================\r\n");
    
    shell_printf("Status: %s\r\n", stats.recording ? "RECORDING" : "STOPPED");
    shell_printf("Records written: %lu\r\n", stats.records_written);
    shell_printf("Active buffer: %lu/%lu records\r\n", 
                 stats.active_buffer_records, stats.active_buffer_capacity);
    
    shell_print("\r\n--- Queue Status ---\r\n");
    shell_printf("WindMaster queue: %u/%u entries (%.1f%%)\r\n",
                 stats.wm_queue_count, stats.wm_queue_capacity,
                 (float)stats.wm_queue_count * 100.0f / stats.wm_queue_capacity);
    shell_printf("VectorNav queue:  %u/%u entries (%.1f%%)\r\n",
                 stats.vn_queue_count, stats.vn_queue_capacity,
                 (float)stats.vn_queue_count * 100.0f / stats.vn_queue_capacity);
    
    shell_print("\r\n--- Performance ---\r\n");
    shell_printf("Max WM queue depth: %u\r\n", stats.wm_queue_max);
    shell_printf("Max VN queue depth: %u\r\n", stats.vn_queue_max);
    
    if (stats.wm_drops > 0 || stats.vn_drops > 0) {
        shell_print("\r\nWARNING: Packet drops detected!\r\n");
        shell_printf("WindMaster drops: %lu\r\n", stats.wm_drops);
        shell_printf("VectorNav drops:  %lu\r\n", stats.vn_drops);
    } else {
        shell_print("\r\nNo packet drops\r\n");
    }
    
    if (stats.recording) {
        shell_printf("\r\nCurrent log file: %s\r\n", stats.filename);
    }
    
    shell_print("===============================================\r\n");
    shell_print(SHELL_PROMPT);
}
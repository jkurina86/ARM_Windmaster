/**
  ******************************************************************************
  * @file    task_rec.c
  * @brief   Recorder task handlers implementation
  * @note    Contains handlers for the recorder task
  ******************************************************************************
  */

#include "task_rec.h"
#include "recorder.h"
#include "windmaster.h"
#include "shell.h"
#include "tasker.h"

/* Task Handler Functions ----------------------------------------------------*/

/**
  * @brief Start recording task handler
  * @param arg Unused (pass NULL)
  * @note Called from main loop via tasker
  * @note Shell output BEFORE recorder_start() to avoid UART deadlock
  */
void handle_rec_start(const void *arg)
{
    (void)arg;

    /* Check if already recording */
    if (recorder_is_recording()) {
        shell_print("Recorder is already running.\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }

    shell_print("Starting recorder...\r\n");
    shell_print(SHELL_PROMPT);

    /* Now start recorder - sensors will begin transmitting immediately */
    recorder_start();
}

/**
  * @brief Stop recording task handler
  * @param arg Unused (pass NULL)
  * @note Called from main loop via tasker
  */
void handle_rec_stop(const void *arg)
{
    (void)arg;

    /* Check if recording */
    if (!recorder_is_recording()) {
        shell_print("Recorder is not running.\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("Stopping recorder...\r\n");
    recorder_stop();    
    shell_print("Recorder stopped!\r\n");

    /* Final stats */
    tasker_enqueue(handle_rec_stats, NULL, 0);

    /* Clear stats */
    recorder_clear_stats();

    shell_print(SHELL_PROMPT);
}

/**
  * @brief Display recorder statistics task handler
  * @param arg Unused (pass NULL)
  * @note Called from main loop via tasker
  */
void handle_rec_stats(const void *arg)
{
    (void)arg;

    /* Get statistics from recorder */
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

    shell_print("\r\n--- Packet Statistics ---\r\n");
    if (stats.wm_drops > 0 || stats.vn_drops > 0) {
        shell_print("WARNING: Data loss detected!\r\n");
        shell_printf("WindMaster drops: %lu (queue overflow)\r\n", stats.wm_drops);
        shell_printf("VectorNav drops:  %lu (queue overflow)\r\n", stats.vn_drops);
    } else {
        shell_print("No packet drops (no data loss)\r\n");
    }
    shell_printf("VectorNav discards: %lu (nearest-neighbor pairing)\r\n", stats.vn_discards);
    shell_printf("WindMaster bad data: %lu (status/sentinel rejection)\r\n", wm_get_bad_data_count());

    if (stats.recording) {
        shell_printf("\r\nCurrent log file: %s\r\n", stats.filename);
    }

    shell_print("===============================================\r\n");
    shell_print(SHELL_PROMPT);
}

/**
  * @brief Display queue debug information task handler
  * @param arg Unused (pass NULL)
  * @note Called from main loop via tasker
  */
void handle_queue_debug(const void *arg)
{
    (void)arg;
    recorder_debug_queue();
    shell_print(SHELL_PROMPT);
}
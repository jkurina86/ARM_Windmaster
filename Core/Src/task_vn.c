/**
  ******************************************************************************
  * @file    task_vn.c
  * @brief   VectorNav task handlers implementation
  * @note    Contains handlers for VectorNav passthrough commands
  ******************************************************************************
  */

#include "task_vn.h"
#include "tasker.h"
#include "shell.h"
#include "vn_cmd.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define VN_CMD_STORAGE_SIZE 128  /* Size of stored command */

/* Private variables ---------------------------------------------------------*/
static char vn_cmd_storage[VN_CMD_STORAGE_SIZE];

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Handle VectorNav passthrough task
 * @param task_data: Pointer to task data (unused, command stored in vn_cmd_storage)
 * @retval None
 */
void handle_vn_passthrough(const task_data_t *task_data)
{
    (void)task_data;  /* Unused */

    /* Clear task flag */
    tasker_clear_task_pending(TASK_VN_PASSTHROUGH);

    /* Check if command is empty */
    if (vn_cmd_storage[0] == '\0') {
        shell_print("Error: No command stored\r\n");
        return;
    }

    /* Send command and get response */
    vn_cmd_passthrough(vn_cmd_storage);
}

/**
 * @brief Schedule VectorNav passthrough task
 * @param command: Command string to send to VectorNav
 * @retval None
 */
void schedule_vn_passthrough(const char *command)
{
    if (!command || strlen(command) >= (VN_CMD_STORAGE_SIZE - 3)) {
        shell_print("Error: Invalid or too-long command\r\n");
        return;
    }

    /* Store command in static buffer with CR+LF termination */
    strncpy(vn_cmd_storage, command, VN_CMD_STORAGE_SIZE - 3);
    strcat(vn_cmd_storage, "\r\n");
    vn_cmd_storage[VN_CMD_STORAGE_SIZE - 1] = '\0';

    /* Schedule the task */
    task_data_t task_data = {0};
    tasker_schedule_task(TASK_VN_PASSTHROUGH, &task_data);
}

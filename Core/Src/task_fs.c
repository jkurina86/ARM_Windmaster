/**
  ******************************************************************************
  * @file    task_fs.c
  * @brief   Filesystem task handlers implementation
  * @note    Contains handlers for filesystem-related tasks
  ******************************************************************************
  */

#include "task_fs.h"
#include "tasker.h"
#include "shell.h"
#include "filesystem.h"

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Handle filesystem mount task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_mount(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_MOUNT);
    
    shell_print("Mounting file system...\r\n");
    FS_Result_t result = filesystem_mount();
    switch (result) {
        case FS_OK:
            shell_print("File system mounted successfully\r\n");
            break;
        case FS_ALREADY_MOUNTED:
            shell_print("File system already mounted\r\n");
            break;
        default:
            shell_printf("Error mounting file system: %d\r\n", result);
            break;
    }
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem unmount task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_unmount(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_UNMOUNT);
    
    shell_print("Unmounting file system...\r\n");
    FS_Result_t result = filesystem_unmount();
    switch (result) {
        case FS_OK:
            shell_print("File system unmounted successfully\r\n");
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        default:
            shell_printf("Error unmounting file system: %d\r\n", result);
            break;
    }
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem df (disk free) task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_df(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_DF);
    
    shell_print("File System Information:\r\n");
    shell_print("=======================\r\n");
    
    uint32_t total_bytes, free_bytes, used_percent;
    FS_Result_t result = filesystem_df(&total_bytes, &free_bytes, &used_percent);
    
    switch (result) {
        case FS_OK:
            shell_printf("Total space: %lu bytes\r\n", total_bytes);
            shell_printf("Free space: %lu bytes\r\n", free_bytes);
            shell_printf("Used space: %lu bytes\r\n", total_bytes - free_bytes);
            shell_printf("Used: %lu%%\r\n", used_percent);
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        default:
            shell_printf("Error getting file system info: %d\r\n", result);
            break;
    }
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem ls (list directory) task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_ls(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_LS);
    
    shell_print("Directory Listing:\r\n");
    shell_print("=================\r\n");

    FS_Result_t result = filesystem_ls(shell_print);
    if (result == FS_NOT_MOUNTED) {
        shell_print("File system not mounted\r\n");
    } else if (result != FS_OK) {
        shell_printf("Error listing directory: %d\r\n", result);
    }
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem cat (read file) task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_cat(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_CAT);
    
    FS_Buffers_t *buffers = filesystem_get_buffers();
    
    if (buffers->filename[0] == '\0') {
        shell_print("Usage: fs-cat <filename>\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("Reading file:\r\n");
    shell_print("============\r\n");
    
    FS_Result_t result = filesystem_cat(buffers->filename, shell_print);
    switch (result) {
        case FS_OK:
            shell_print("\r\nEOF\r\n");
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        case FS_FILE_NOT_FOUND:
            shell_print("File not found\r\n");
            break;
        default:
            shell_printf("Error reading file: %d\r\n", result);
            break;
    }
    
    buffers->filename[0] = '\0'; /* Clear filename buffer after use */
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem write task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_write(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_WRITE);
    
    FS_Buffers_t *buffers = filesystem_get_buffers();
    
    if (buffers->filename[0] == '\0' || buffers->file_data[0] == '\0') {
        shell_print("Usage: fs-write <filename> <data>\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("Writing to file:\r\n");
    shell_print("================\r\n");
    
    FS_Result_t result = filesystem_write(buffers->filename, buffers->file_data);
    switch (result) {
        case FS_OK:
            shell_printf("Data written to %s successfully\r\n", buffers->filename);
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        case FS_ACCESS_DENIED:
            shell_print("Access denied\r\n");
            break;
        default:
            shell_printf("Error writing to file: %d\r\n", result);
            break;
    }
    
    buffers->file_data[0] = '\0'; /* Clear data buffer after write */
    buffers->filename[0] = '\0'; /* Clear filename buffer after use */
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem rm (remove file) task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_rm(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_RM);
    
    FS_Buffers_t *buffers = filesystem_get_buffers();
    
    if (buffers->filename[0] == '\0') {
        shell_print("Usage: fs-rm <filename>\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("Deleting file:\r\n");
    shell_print("==============\r\n");
    
    FS_Result_t result = filesystem_rm(buffers->filename);
    switch (result) {
        case FS_OK:
            shell_printf("File %s deleted successfully\r\n", buffers->filename);
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        case FS_FILE_NOT_FOUND:
            shell_print("File not found\r\n");
            break;
        case FS_ACCESS_DENIED:
            shell_print("Access denied\r\n");
            break;
        default:
            shell_printf("Error deleting file: %d\r\n", result);
            break;
    }
    
    buffers->filename[0] = '\0'; /* Clear filename buffer after use */
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem mkdir (make directory) task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_mkdir(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_MKDIR);
    
    FS_Buffers_t *buffers = filesystem_get_buffers();
    
    if (buffers->dirname[0] == '\0') {
        shell_print("Usage: fs-mkdir <dirname>\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("Creating directory:\r\n");
    shell_print("===================\r\n");
    
    FS_Result_t result = filesystem_mkdir(buffers->dirname);
    switch (result) {
        case FS_OK:
            shell_printf("Directory %s created successfully\r\n", buffers->dirname);
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        case FS_ACCESS_DENIED:
            shell_print("Access denied or directory already exists\r\n");
            break;
        default:
            shell_printf("Error creating directory: %d\r\n", result);
            break;
    }
    
    buffers->dirname[0] = '\0'; /* Clear dirname buffer after use */
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem rmdir (remove directory) task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_rmdir(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_RMDIR);
    
    FS_Buffers_t *buffers = filesystem_get_buffers();
    
    if (buffers->dirname[0] == '\0') {
        shell_print("Usage: fs-rmdir <dirname>\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("Removing directory:\r\n");
    shell_print("===================\r\n");

    FS_Result_t result = filesystem_rmdir(buffers->dirname);
    switch (result) {
        case FS_OK:
            shell_printf("Directory %s removed successfully\r\n", buffers->dirname);
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        case FS_FILE_NOT_FOUND:
            shell_print("Directory not found\r\n");
            break;
        case FS_ACCESS_DENIED:
            shell_print("Access denied or directory not empty\r\n");
            break;
        default:
            shell_printf("Error removing directory: %d\r\n", result);
            break;
    }
    
    buffers->dirname[0] = '\0'; /* Clear dirname buffer after use */
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle filesystem cp (copy file) task
 * @param task_data: Pointer to task data (unused for this task)
 * @retval None
 */
void handle_fs_cp(const task_data_t *task_data)
{
    /* Clear task flag first */
    tasker_clear_task_pending(TASK_FS_CP);
    
    FS_Buffers_t *buffers = filesystem_get_buffers();
    
    if (buffers->filename[0] == '\0' || buffers->dest_filename[0] == '\0') {
        shell_print("Usage: fs-cp <source> <destination>\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("Copying file:\r\n");
    shell_print("=============\r\n");

    FS_Result_t result = filesystem_cp(buffers->filename, buffers->dest_filename);
    switch (result) {
        case FS_OK:
            shell_printf("File copied from %s to %s successfully\r\n", 
                       buffers->filename, buffers->dest_filename);
            break;
        case FS_NOT_MOUNTED:
            shell_print("File system not mounted\r\n");
            break;
        case FS_FILE_NOT_FOUND:
            shell_print("Source file not found\r\n");
            break;
        case FS_ACCESS_DENIED:
            shell_print("Access denied\r\n");
            break;
        default:
            shell_printf("Error copying file: %d\r\n", result);
            break;
    }
    
    buffers->filename[0] = '\0';      /* Clear source filename buffer */
    buffers->dest_filename[0] = '\0'; /* Clear destination filename buffer */
    shell_print(SHELL_PROMPT);
}

/* Scheduling Functions ------------------------------------------------------*/

/**
 * @brief Schedule a file system mount task
 * @param None
 * @retval None
 */
void schedule_fs_mount(void)
{
    tasker_schedule_task(TASK_FS_MOUNT, NULL);
}

/**
 * @brief Schedule a file system unmount task
 * @param None
 * @retval None
 */
void schedule_fs_unmount(void)
{
    tasker_schedule_task(TASK_FS_UNMOUNT, NULL);
}

/**
 * @brief Schedule a df task
 * @param None
 * @retval None
 */
void schedule_fs_df(void)
{
    tasker_schedule_task(TASK_FS_DF, NULL);
}

/**
 * @brief Schedule an ls task
 * @param None
 * @retval None
 */
void schedule_fs_ls(void)
{
    tasker_schedule_task(TASK_FS_LS, NULL);
}

/**
 * @brief Schedule a cat task
 * @param None
 * @retval None
 */
void schedule_fs_cat(void)
{
    tasker_schedule_task(TASK_FS_CAT, NULL);
}

/**
 * @brief Schedule a write task
 * @param None
 * @retval None
 */
void schedule_fs_write(void)
{
    tasker_schedule_task(TASK_FS_WRITE, NULL);
}

/**
 * @brief Schedule an rm task
 * @param None
 * @retval None
 */
void schedule_fs_rm(void)
{
    tasker_schedule_task(TASK_FS_RM, NULL);
}

/**
 * @brief Schedule a mkdir task
 * @param None
 * @retval None
 */
void schedule_fs_mkdir(void)
{
    tasker_schedule_task(TASK_FS_MKDIR, NULL);
}

/**
 * @brief Schedule an rmdir task
 * @param None
 * @retval None
 */
void schedule_fs_rmdir(void)
{
    tasker_schedule_task(TASK_FS_RMDIR, NULL);
}

/**
 * @brief Schedule a cp task
 * @param None
 * @retval None
 */
void schedule_fs_cp(void)
{
    tasker_schedule_task(TASK_FS_CP, NULL);
}

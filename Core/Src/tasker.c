/**
  ******************************************************************************
  * @file    tasker.c
  * @brief   Task scheduler implementation
  * @note    Simple task scheduler for command execution
  ******************************************************************************
  */

#include "tasker.h"
#include "shell.h"
#include "main.h"
#include "task_rtc.h"

/* Private types -------------------------------------------------------------*/

/**
 * @brief Task control block
 */
typedef struct {
    uint8_t pending;                /* Task pending flag */
    task_data_t data;               /* Task-specific data */
} task_t;

/* Private variables ---------------------------------------------------------*/

/* Array of task control blocks indexed by task type, this gets populated during init. */
static volatile task_t tasks[TASK_MAX];

/**
 * @brief Task dispatch table: Maps each task type to its handler and auto-clear behavior.
 *        This table drives the loop in tasker_run(), ensuring compile-time alignment with task_type_t.
 *        - Handler: Function to execute for the task.
 *        - Auto-clear: 1 to clear after execution, 0 to leave pending.
 * @note When adding a new task: Update task_type_t enum in tasker.h and add an entry here.
 */
static const task_descriptor_t task_table[TASK_MAX] = {
    [TASK_NONE] = { NULL, 1 },

    /* General Tasks */
    [TASK_RESET] = { handle_reset, 0 },
    [TASK_HELLO] = { handle_hello, 1 },
    [TASK_VERSION] = { handle_version, 1 },
    [TASK_HELP] = { handle_help, 1 },
    [TASK_CLEAR] = { handle_clear, 1 },
    [TASK_STATUS] = { handle_status, 1 },
    
    /* Filesystem Tasks */
    [TASK_FS_MOUNT] = { handle_fs_mount, 1 },
    [TASK_FS_UNMOUNT] = { handle_fs_unmount, 1 },
    [TASK_FS_DF] = { handle_fs_df, 1 },
    [TASK_FS_LS] = { handle_fs_ls, 1 },
    [TASK_FS_CAT] = { handle_fs_cat, 1 },
    [TASK_FS_WRITE] = { handle_fs_write, 1 },
    [TASK_FS_RM] = { handle_fs_rm, 1 },
    [TASK_FS_MKDIR] = { handle_fs_mkdir, 1 },
    [TASK_FS_RMDIR] = { handle_fs_rmdir, 1 },
    [TASK_FS_CP] = { handle_fs_cp, 1 },
    
    /* RTC Tasks */
    [TASK_RTC_TIMER_STATUS] = { handle_rtc_timer_status, 1 },
    [TASK_RTC_TIMER_STOP] = { handle_rtc_timer_stop, 1 },
    [TASK_RTC_TIMER_SET] = { handle_rtc_timer_set, 1 },
    [TASK_RTC_TEMP] = { handle_rtc_temp, 1 },
    [TASK_RTC_GETTIME] = { handle_rtc_gettime, 1 },
    [TASK_RTC_SETTIME] = { handle_rtc_settime, 1 },
};

/* Private function prototypes -----------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize the task scheduler
 * @param None
 * @retval None
 */
void tasker_init(void)
{
    /* Clear all task control blocks */
    for (int i = 0; i < TASK_MAX; i++) {
        /* Clear pending flags */
        tasks[i].pending = 0;
        /* Set task data to zero */
        memset((void*)&tasks[i].data, 0, sizeof(task_data_t));
    }
}

/**
 * @brief Run pending tasks
 * @param None
 * @retval None
 */
void tasker_run(void)
{
    /* Loop through all tasks */
    for (int i = 0; i < TASK_MAX; i++) {
        /* Check if task is pending and has a valid handler */
        if (tasks[i].pending && task_table[i].handler != NULL) {
            /* Call the task handler with its data */
            task_table[i].handler((const task_data_t*)&tasks[i].data);
            /* Auto-clear pending flag if configured */
            if (task_table[i].auto_clear) {
                tasks[i].pending = 0;
            }
        }
    }
}

/**
 * @brief Schedule a task for execution
 * @param task_type: Type of task to schedule
 * @param task_data: Optional task data (can be NULL)
 * @retval None
 */
void tasker_schedule_task(task_type_t task_type, const task_data_t *task_data)
{
    /* Check for valid task type */
    if (task_type >= TASK_MAX) {
        return;
    }
    
    /* Copy task data if provided */
    if (task_data != NULL) {
        memcpy((void*)&tasks[task_type].data, task_data, sizeof(task_data_t));
    }
    
    /* Set task as pending */
    tasks[task_type].pending = 1;
}

/**
 * @brief Check if a specific task is pending
 * @param task_type: Type of task to check
 * @retval 1 if task is pending, 0 otherwise
 */
uint8_t tasker_get_pending(task_type_t task_type)
{
    /* Check for valid task type */
    if (task_type >= TASK_MAX) {
        return 0;
    }

    return tasks[task_type].pending;
}

/**
 * @brief Clear a pending task flag
 * @param task_type: Type of task to clear
 * @retval None
 */
void tasker_clear_task_pending(task_type_t task_type)
{
    /* Check for valid task type */
    if (task_type >= TASK_MAX) {
        return;
    }

    tasks[task_type].pending = 0;
}

/**
 * @brief Get task data for a specific task
 * @param task_type: Type of task to get data for
 * @retval Pointer to task data
 * @note Returns a const pointer to prevent modification of task data
 */
const task_data_t* tasker_get_task_data(task_type_t task_type)
{
    /* Check for valid task type */ 
    if (task_type >= TASK_MAX) {
        return NULL;
    }

    /* Get a pointer to the task data and cast it to const */
    const task_data_t* task_data = (const task_data_t*)&tasks[task_type].data;
    
    return task_data;
}
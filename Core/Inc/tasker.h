/**
  ******************************************************************************
  * @file    tasker.h
  * @brief   Task scheduler header file
  * @note    Simple task scheduler for command execution
  ******************************************************************************
  */
#ifndef INC_TASKER_H_
#define INC_TASKER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "task_gen.h"
#include "task_fs.h"
#include "task_rtc.h"

/* Exported types ------------------------------------------------------------*/

/* Task types enum */
typedef enum {
    TASK_NONE = 0,
    
    /* General Tasks */
    TASK_RESET,
    TASK_HELLO,
    TASK_VERSION,
    TASK_HELP,
    TASK_CLEAR,
    TASK_STATUS,
    
    /* Filesystem Tasks */
    TASK_FS_MOUNT,
    TASK_FS_UNMOUNT,
    TASK_FS_DF,
    TASK_FS_LS,
    TASK_FS_CAT,
    TASK_FS_WRITE,
    TASK_FS_RM,
    TASK_FS_MKDIR,
    TASK_FS_RMDIR,
    TASK_FS_CP,
    
    /* RTC Tasks */
    TASK_RTC_TIMER_STATUS,
    TASK_RTC_TIMER_STOP,
    TASK_RTC_TIMER_SET,
    TASK_RTC_TEMP,
    TASK_RTC_GETTIME,
    TASK_RTC_SETTIME,
    
    TASK_MAX
} task_type_t;

/* Task data structure for passing parameters */
typedef struct task_data_t {
    /* Union to hold generic task data, it's only as big as the largest member in memory */
    union {
        /* For reset task */
        struct {
            uint32_t reset_due_ms;
        } reset;
        
        /* For hello task */
        struct {
            uint8_t uart_num;
        } hello;
        
        /* For RTC timer set task */
        struct {
            uint16_t seconds;
        } rtc_timer;
        
        /* For RTC set time task */
        struct {
            uint16_t year;          /* Full year YYYY */
            uint8_t months;         /* 1-12 */
            uint8_t days;           /* 1-31 */
            uint8_t hours;          /* 0-23 */
            uint8_t minutes;        /* 0-59 */
            uint8_t seconds;        /* 0-59 */
            uint8_t weekdays;       /* 1-7 (1=Sunday) */
            uint8_t argc;           /* Argument count for validation */
        } rtc_settime;
    };
} task_data_t;


/* Function pointer type for task handlers. Take a const pointer to task data and return void. */
typedef void (*task_handler_t)(const task_data_t *);

/* Descriptor for a task in the scheduler table. */
typedef struct {
    /* Pointer to the function that handles the task. */
    task_handler_t handler;  
    /* Flag: 1 to clear task after execution, 0 to leave pending. */
    uint8_t auto_clear;
} task_descriptor_t;

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

void tasker_init(void);
void tasker_run(void);
void tasker_schedule_task(task_type_t task_type, const task_data_t *task_data);
uint8_t tasker_get_pending(task_type_t task_type);

/* Task array access functions for task modules */
void tasker_clear_task_pending(task_type_t task_type);
const task_data_t* tasker_get_task_data(task_type_t task_type);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASKER_H_ */
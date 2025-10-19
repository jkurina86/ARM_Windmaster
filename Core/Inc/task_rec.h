/**
  ******************************************************************************
  * @file    task_rec.h
  * @brief   Recorder task handlers header file
  * @note    Contains handlers for the recorder task
  ******************************************************************************
  */

#ifndef INC_TASK_REC_H_
#define INC_TASK_REC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "tasker.h"

/* Exported function prototypes ----------------------------------------------*/

/* Scheduler functions - called from shell commands */
void schedule_rec_start(void);
void schedule_rec_stop(void);
void schedule_rec_stats(void);

/* Task handler functions - called by tasker */
void handle_rec_start(const task_data_t *task_data);
void handle_rec_stop(const task_data_t *task_data);
void handle_rec_stats(const task_data_t *task_data);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASK_REC_H_ */
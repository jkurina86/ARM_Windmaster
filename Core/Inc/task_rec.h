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

/* Task handler functions - called by tasker */
void handle_rec_start(const void *arg);
void handle_rec_stop(const void *arg);
void handle_rec_stats(const void *arg);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASK_REC_H_ */
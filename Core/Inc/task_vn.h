/**
  ******************************************************************************
  * @file    task_vn.h
  * @brief   VectorNav task handlers header file
  * @note    Contains handlers for VectorNav passthrough commands
  ******************************************************************************
  */
#ifndef INC_TASK_VN_H_
#define INC_TASK_VN_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "tasker.h"

/* Forward declarations ------------------------------------------------------------------*/
typedef struct task_data_t task_data_t;

/* Exported function prototypes ----------------------------------------------*/
void handle_vn_passthrough(const task_data_t *task_data);

/* Scheduling function prototypes ---------------------------------------------*/
void schedule_vn_passthrough(const char *command);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASK_VN_H_ */

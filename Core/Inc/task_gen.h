/**
  ******************************************************************************
  * @file    task_gen.h
  * @brief   General task handlers header file
  * @note    Contains handlers for general system tasks
  ******************************************************************************
  */
#ifndef INC_TASK_GEN_H_
#define INC_TASK_GEN_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Forward declarations ------------------------------------------------------------------*/
typedef struct task_data_t task_data_t;

/* Exported function prototypes ----------------------------------------------*/
void handle_reset(const task_data_t *task_data);
void handle_hello(const task_data_t *task_data);
void handle_version(const task_data_t *task_data);
void handle_help(const task_data_t *task_data);
void handle_clear(const task_data_t *task_data);
void handle_status(const task_data_t *task_data);

/* Scheduling function prototypes ---------------------------------------------*/
void schedule_reset(uint32_t delay_ms);
void schedule_hello(uint8_t uart_num);
void schedule_version(void);
void schedule_help(void);
void schedule_clear(void);
void schedule_status(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASK_GEN_H_ */
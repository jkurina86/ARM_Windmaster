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

/* Argument structures for tasks that need parameters -----------------------*/

/** Arguments for reset task */
typedef struct {
    uint32_t reset_due_ms;
} reset_args_t;

/** Arguments for hello task */
typedef struct {
    uint8_t uart_num;
} hello_args_t;

/** Arguments for snooze task */
typedef struct {
    uint16_t seconds;
} snooze_args_t;

/* Task handler prototypes ---------------------------------------------------*/

void handle_reset(const void *arg);
void handle_hello(const void *arg);
void handle_version(const void *arg);
void handle_help(const void *arg);
void handle_clear(const void *arg);
void handle_status(const void *arg);
void handle_snooze(const void *arg);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASK_GEN_H_ */
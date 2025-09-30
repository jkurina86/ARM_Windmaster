/**
  ******************************************************************************
  * @file    task_rtc.h
  * @brief   RTC task handlers
  ******************************************************************************/

#ifndef INC_TASK_RTC_H_
#define INC_TASK_RTC_H_

#include "tasker.h"

/* Function prototypes */
void handle_rtc_timer_status(const task_data_t *task_data);
void handle_rtc_timer_stop(const task_data_t *task_data);
void handle_rtc_timer_set(const task_data_t *task_data);
void handle_rtc_temp(const task_data_t *task_data);
void handle_rtc_gettime(const task_data_t *task_data);
void handle_rtc_settime(const task_data_t *task_data);

void schedule_rtc_timer_status(void);
void schedule_rtc_timer_stop(void);
void schedule_rtc_timer_set(uint16_t seconds);
void schedule_rtc_temp(void);
void schedule_rtc_gettime(void);
void schedule_rtc_settime(int argc, char **argv);

#endif /* INC_TASK_RTC_H_ */
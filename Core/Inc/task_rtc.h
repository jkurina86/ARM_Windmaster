/**
  ******************************************************************************
  * @file    task_rtc.h
  * @brief   RTC task handlers
  ******************************************************************************/

#ifndef INC_TASK_RTC_H_
#define INC_TASK_RTC_H_

#include <stdint.h>

/* Argument structures -------------------------------------------------------*/

/** @brief Arguments for rtc-timer-set command */
typedef struct {
    uint16_t seconds;
} rtc_timer_args_t;

/** @brief Arguments for rtc-settime command (parsed values) */
typedef struct {
    uint16_t year;
    uint8_t months;
    uint8_t days;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t weekdays;
    uint8_t valid;  /* Non-zero if values are valid (argc was correct) */
} rtc_settime_args_t;

/* Handler prototypes --------------------------------------------------------*/
void handle_rtc_timer_status(const void *arg);
void handle_rtc_timer_stop(const void *arg);
void handle_rtc_timer_set(const void *arg);
void handle_rtc_temp(const void *arg);
void handle_rtc_gettime(const void *arg);
void handle_rtc_settime(const void *arg);

#endif /* INC_TASK_RTC_H_ */
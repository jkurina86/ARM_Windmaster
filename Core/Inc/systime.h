/**
  ******************************************************************************
  * @file    systime.h
  * @brief   System time functions
  * @note    Timekeeping
  ******************************************************************************
  */

#ifndef INC_SYSTIME_H_
#define INC_SYSTIME_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "ab-rtcmc-rtc.h"

/* Function Prototypes -------------------------------------------------------*/
uint64_t datetime_to_epoch(const RTC_DateTime_t* dt);
void systime_init(const RTC_DateTime_t* dt);
void systime_request_update(uint64_t new_epoch);
void systime_pps_event(void);
uint64_t time_us_now(void);
uint64_t time_s_now(void);
const char* timestamp(uint64_t usecs);
int32_t systime_ppm_estimate(void);
bool systime_have_lock(void);
uint64_t systime_get_pps_count(void);
RTC_DateTime_t epoch_to_datetime(uint64_t epoch);

#endif /* INC_SYSTIME_H_ */

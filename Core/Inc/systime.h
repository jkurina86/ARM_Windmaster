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
void systime_request_update(uint32_t new_epoch_ms);
void systime_pps_event(void);
uint32_t time_ms_now(void);
uint32_t time_s_now(void);
const char* timestamp(uint32_t ms);
int32_t systime_ppm_estimate(void);
bool systime_have_lock(void);
uint64_t systime_get_pps_count(void);
uint32_t systime_get_cycle_count(void);
uint32_t systime_get_cycle_at_start(void);
RTC_DateTime_t epoch_to_datetime(uint64_t epoch);

#endif /* INC_SYSTIME_H_ */

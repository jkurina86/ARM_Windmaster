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
uint32_t datetime_to_epoch(const RTC_DateTime_t* dt);
void systime_init(const RTC_DateTime_t* dt);
void systime_request_update(uint32_t new_epoch_sec);
void systime_pps_event(void);
uint32_t time_s_now(void);
uint64_t time_ms_now(void);
const char* timestamp(uint32_t s);
int32_t systime_ppm_estimate(void);
bool systime_have_lock(void);
uint32_t systime_get_pps_count(void);
RTC_DateTime_t epoch_to_datetime(uint32_t epoch);
RTC_DateTime_t gps_to_datetime(uint32_t gps_seconds);
void systime_snapshot(uint32_t *epoch_seconds, uint16_t *ms);

#endif /* INC_SYSTIME_H_ */

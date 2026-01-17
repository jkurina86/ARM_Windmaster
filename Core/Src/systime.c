/**
  ******************************************************************************
  * @file    systime.c
  * @brief   System time functions
  * @note    Timekeeping using RTC and TIM2 with PPS synchronization
  ******************************************************************************
  */
#include "systime.h"
#include "tim.h"
#include <stdbool.h>

/* Internal State ----------------------------------------------------------- */
static volatile uint32_t g_epoch_sec = 0;      /* Seconds since 2000-01-01 00:00:00 */
static volatile uint32_t g_pps_t2 = 0;         /* TIM2 counter at last PPS */
static volatile uint32_t g_tps_est = 1000000;  /* TIM2 ticks per second estimate (nominal 1 MHz) */
static volatile uint32_t g_pps_count = 0;      /* Number of PPS events seen */
static volatile bool g_set_pending = false;    /* Request to set time on next PPS */
static volatile uint32_t g_set_epoch_sec = 0;  /* Pending epoch seconds to apply on next PPS */

/* Private inline helper functions -------------------------------------------*/

/** @brief Check if a year is a leap year
  * @param y Year (e.g., 2024)
  * @retval true if leap year, false otherwise
*/
static inline bool is_leap(uint32_t y){ 
    return (y % 4 == 0) && (y % 100 != 0 || y % 400 == 0); 
}

/** @brief Get cumulative days at start of month
  * @param m Month (1-12)
  * @param leap true if leap year, false otherwise
  * @retval Cumulative days at start of month
  * @note January=0, February=31, March=59 (or 60 if leap), etc.
*/
static inline uint32_t month_offset(uint32_t m, bool leap){
    static const uint16_t cumulative_days[12]      = {0,31,59,90,120,151,181,212,243,273,304,334};
    static const uint16_t cumulative_days_leap[12] = {0,31,60,91,121,152,182,213,244,274,305,335};
    if (leap) {
        return cumulative_days_leap[m-1];
    } else {
        return cumulative_days[m-1];
    }
}

/* Public functions -----------------------------------------------*/

/** @brief Initialize system time with given date/time
  * @param initial_dt Pointer to RTC_DateTime_t structure with initial date/time
  * @retval None
  * @note Initializes internal state and TIM2 counter reference
  * @note Converts seconds to milliseconds (no Q32 fixed-point needed)
*/
void systime_init(const RTC_DateTime_t* initial_dt) {
    g_epoch_sec = datetime_to_epoch(initial_dt);
    g_pps_t2 = __HAL_TIM_GET_COUNTER(&htim2);
    g_tps_est = 1000000;
    g_pps_count = 0;
    g_set_pending = false;
    g_set_epoch_sec = 0;
}

/** @brief Request to set system time at next PPS event
  * @param new_epoch_sec New epoch time in seconds since 2000-01-01
  * @retval None
  * @note Time will be set on next PPS event
*/
void systime_update(uint32_t new_epoch_sec) {
    g_set_epoch_sec = new_epoch_sec;
    g_set_pending = true;
}

/** @brief Handle PPS event from inside the TIM3 interrupt
 *  @param None
 *  @retval None
 *  @note Updates epoch seconds and TIM2 reference for sub-second interpolation
 */
void systime_pps_event(void) {
    /* Sample the TIM2 counter */
    uint32_t t2_now = __HAL_TIM_GET_COUNTER(&htim2);
    
    /* Calculate ticks since last PPS (handle wraparound) */
    uint32_t delta_t2;
    if (t2_now >= g_pps_t2) {
        delta_t2 = t2_now - g_pps_t2;
    } else {
        delta_t2 = 0xFFFFFFFF - g_pps_t2 + t2_now + 1;
    }
    
    /* Update TIM2 reference point */
    g_pps_t2 = t2_now;

    /* Update ticks-per-second estimate (EMA over 16 samples) */
    if (g_pps_count > 0) {
        g_tps_est = ((g_tps_est * 15U) + delta_t2) >> 4;
    }

    /* Advance epoch by one second */
    g_epoch_sec += 1U;

    /* Handle pending time-set request */
    if (g_set_pending) {
        g_epoch_sec = g_set_epoch_sec;
        g_set_pending = false;
    }

    g_pps_count++;
}

/** @brief Get current time in seconds since 2000-01-01 00:00:00
  * @param None
  * @retval Current time in seconds
  * @note Simplified: just convert milliseconds to seconds (all 32-bit)
*/
uint32_t time_s_now(void) {
    return g_epoch_sec;
}

/** @brief Get current time in milliseconds since 2000-01-01 00:00:00
  * @param None
  * @retval Current time in milliseconds
  * @note Combines seconds and milliseconds, 64-bit arithmetic
*/
uint64_t time_ms_now(void) {
    uint32_t epoch_sec;
    uint16_t ms;
    systime_snapshot(&epoch_sec, &ms);
    return ((uint64_t)epoch_sec * 1000ULL) + (uint64_t)ms;
}

/** @brief Get formatted timestamp string for a given time in seconds
  * @param s Time in seconds since 2000-01-01 00:00:00
  * @retval Formatted timestamp string in ISO 8601 format (YYYY-MM-DDTHH:MM:SSZ)
  * @note Returns static string buffer
*/
const char* timestamp(uint32_t s) {
    static char buffer[32];

    RTC_DateTime_t dt = epoch_to_datetime(s);

    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             dt.years + 2000, dt.months, dt.days,
             dt.hours, dt.minutes, dt.seconds);

    return buffer;
}

/** @brief Estimate the PPM deviation of the timer from 1 MHz
  * @param None
  * @retval PPM deviation (positive or negative)
  * @note A return value of 0 indicates perfect 1 MHz operation
*/
int32_t systime_ppm_estimate(void) {
    /* Check if estimate is exactly 1 MHz */
    if (g_tps_est == 1000000) {
        return 0;
    }
    /* Calculate PPM deviation from 1 MHz using 32-bit arithmetic */
    /* PPM = (tps_est - 1000000) / 1000 (simplified, loses some precision but adequate) */
    int32_t diff = (int32_t)g_tps_est - 1000000;
    int32_t ppm = diff / 1000;
    return ppm;
}

/** @brief Check if the system time has a valid lock
  * @param None
  * @retval true if locked, false otherwise
  * @note A valid lock is indicated by having seen at least one PPS event
*/
bool systime_have_lock(void) {
    return g_pps_count > 0;
}

/** @brief Get PPS event count for debugging
  * @param None
  * @retval Number of PPS events seen since initialization
*/
uint32_t systime_get_pps_count(void) {
    return g_pps_count;
}

/** @brief Convert RTC date/time to epoch time (seconds since 2000-01-01)
 *  @param dt Pointer to RTC_DateTime_t structure
 *  @retval Epoch time in seconds since 2000-01-01 00:00:00 (as uint32_t, wraps every ~136 years)
 *  @note Assumes valid date/time in dt
 *  @note Uses only 32-bit arithmetic - safe for dates 2000-2136
 */
uint32_t datetime_to_epoch(const RTC_DateTime_t* dt) {
    uint32_t year = dt->years + 2000;
    uint32_t month = dt->months;
    uint32_t day = dt->days;
    uint32_t hour = dt->hours;
    uint32_t minute = dt->minutes;
    uint32_t second = dt->seconds;

    /* Calculate days since 2000-01-01 using 32-bit only arithmetic */
    /* Days = (year-2000)*365 + leap_day_count + day_of_year */
    uint32_t years_since_base = year - 2000;

    /* Count leap days that have fully occurred before this date */
    uint32_t leaps_before_year = (year - 1) / 4 - (year - 1) / 100 + (year - 1) / 400;
    uint32_t leaps_before_1999 = 1999 / 4 - 1999 / 100 + 1999 / 400;
    uint32_t leap_count = leaps_before_year - leaps_before_1999;
    if (is_leap(year) && month <= 2) {
        /* Current leap day has not happened yet in this year */
        leap_count--;
    }

    uint32_t days = years_since_base * 365 + leap_count;

    /* Add days for months in current year */
    days += month_offset(month, is_leap(year));
    /* Add days within current month (day is 1-indexed) */
    days += day - 1;

    /* Total seconds - all 32-bit operations */
    uint32_t epoch = days * 86400U + hour * 3600U + minute * 60U + second;
    return epoch;
}

/** @brief Convert epoch time to RTC date/time
 *  @param epoch Epoch time in seconds since 2000-01-01 00:00:00
 *  @retval RTC_DateTime_t structure with converted date/time
 *  @note Handles leap years and valid ranges
 */
RTC_DateTime_t epoch_to_datetime(uint32_t epoch) {
    RTC_DateTime_t dt = {0};
    uint32_t total_seconds = epoch;
    uint32_t days = total_seconds / 86400U;
    uint32_t remaining_seconds = total_seconds % 86400U;

    /* Calculate weekday (1=Sunday, 7=Saturday)
     * 2000-01-01 was a Saturday (day 7 in 1=Sunday convention) */
    dt.weekdays = ((days + 6) % 7) + 1;

    /* Find year */
    uint32_t year = 2000;
    while (1) {
        uint32_t days_in_year = is_leap(year) ? 366 : 365;
        if (days < days_in_year) break;
        days -= days_in_year;
        year++;
    }
    dt.years = year - 2000;

    /* Find month */
    bool leap = is_leap(year);
    uint32_t month = 1;
    for (month = 1; month <= 12; month++) {
        uint32_t days_in_month;
        switch (month) {
            case 2:
                days_in_month = leap ? 29 : 28;
                break;
            case 4:
            case 6:
            case 9:
            case 11:
                days_in_month = 30;
                break;
            default:
                days_in_month = 31;
                break;
        }
        if (days < days_in_month) break;
        days -= days_in_month;
    }
    /* Ensure month doesn't exceed 12 (December) */
    if (month > 12) {
        month = 12;
        days = 30; /* Clamp to end of December */
    }
    dt.months = month;
    dt.days = days + 1; /* Convert from 0-indexed to 1-indexed */

    /* Time components */
    dt.hours = remaining_seconds / 3600;
    remaining_seconds %= 3600;
    dt.minutes = remaining_seconds / 60;
    dt.seconds = remaining_seconds % 60;

    return dt;
}

/** @brief Snapshot current epoch seconds and milliseconds
 *  @param epoch_seconds Pointer to store epoch seconds since 2000-01-01 00:00:00
 *  @param ms Pointer to store milliseconds within current second (0-999)
 *  @retval None
 *  @note Milliseconds are calculated from TIM2 ticks elapsed since last PPS
 */
void systime_snapshot(uint32_t *epoch_seconds, uint16_t *ms) {
    if (epoch_seconds == NULL || ms == NULL) {
        return;
    }

    uint32_t epoch_local;
    uint32_t pps_t2_local;
    uint32_t tps_est_local;
    uint32_t t2_now;
    
    /* Lock-free read: if epoch changes during read, a PPS occurred - retry */
    do {
        epoch_local = g_epoch_sec;
        pps_t2_local = g_pps_t2;
        tps_est_local = g_tps_est;
        t2_now = __HAL_TIM_GET_COUNTER(&htim2);
    } while (epoch_local != g_epoch_sec);
    
    /* Calculate ticks since last PPS (handle wraparound) */
    uint32_t delta_ticks;
    if (t2_now >= pps_t2_local) {
        delta_ticks = t2_now - pps_t2_local;
    } else {
        delta_ticks = 0xFFFFFFFF - pps_t2_local + t2_now + 1;
    }
    
    /* Convert ticks to milliseconds */
    uint32_t elapsed_ms = delta_ticks / (tps_est_local / 1000U);
    
    /* Handle second rollover (shouldn't happen normally, but be safe) */
    if (elapsed_ms >= 1000U) {
        elapsed_ms -= 1000U;
        epoch_local += 1U;
    }
    
    *epoch_seconds = epoch_local;
    *ms = (uint16_t)elapsed_ms;
}

/** @brief Convert GPS seconds since 1980-01-06 to RTC date/time
  *  @param gps_seconds GPS time in seconds since 1980-01-06 00:00:00
  *  @retval RTC_DateTime_t structure with converted date/time
  *  @note Handles leap years and valid ranges
  */
RTC_DateTime_t gps_to_datetime(uint32_t gps_seconds) {
    /* GPS epoch starts at 1980-01-06 */
    const uint32_t GPS_EPOCH_TO_2000_EPOCH = 630720000; /* Seconds from 1980-01-06 to 2000-01-01 */

    /* Convert GPS seconds to RTC epoch seconds */
    uint32_t rtc_epoch = gps_seconds - GPS_EPOCH_TO_2000_EPOCH;

    /* Use existing function to convert to RTC_DateTime_t */
    return epoch_to_datetime(rtc_epoch);
}
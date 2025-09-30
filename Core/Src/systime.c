/**
  ******************************************************************************
  * @file    systime.c
  * @brief   System time functions
  * @note    Timekeeping
  ******************************************************************************
  */
#include "systime.h"
#include "stm32l4xx_ll_tim.h"

/* Internal State (Global) -------------------------------------------------- */
static volatile uint64_t g_epoch_time = 0;                          /* RTC aligned seconds at last PPS */
static volatile uint32_t g_pps_t2 = 0;                              /* TIM2 counter at last PPS */
static volatile uint64_t g_tps_est = 1000000;                       /* TIM2 ticks per second estimate (Nominal: 1 MHz) */
static volatile int64_t g_phase = 0;                                /* Phase offset from the RTC Pulses in ticks */
static volatile uint64_t g_scale_q32 = (1000000ULL << 32);          /* fixed-point scale for microseconds per tick */
static volatile uint64_t g_pps_count = 0;                           /* Number of PPS events seen */
static volatile bool g_set_pending = false;                         /* Request to set time on next PPS */
static volatile uint64_t g_set_epoch = 0;                           /* Epoch time to set on next PPS */

/* Private defines -----------------------------------------------------------*/
#define RATE_SHIFT 4
#define PHASE_SHIFT 3
#define EPOCH_2000_OFFSET 946684800ULL /* RTC epoch start (for conversion to UNIX Epoch) */

/* Private inline helper functions -------------------------------------------*/

/** @brief Update the scale factor for microseconds per tick
  * @param tps Ticks per second
  * @retval None
  * @note Updates g_scale_q32 and avoids FPU in the ISR with Q32 representation
*/
static inline void update_scale(uint64_t tps) {
    if (!tps) tps = 1000000ULL; /* Avoid divide-by-zero. Default to nominal case. */
    
    /* First converts to Q32 format with a 32-bit left-shift.
     * 64-bit Q Notation: Integer part is upper 32 bits, fractional part is lower 32 bits.
     * Divide this by ticks per second to get a Q32 scaling factor for microseconds per tick.
    */
    g_scale_q32 = (1000000ULL << 32) / tps;
}

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
*/
void systime_init(const RTC_DateTime_t* initial_dt) {
    uint64_t initial_epoch = datetime_to_epoch(initial_dt);
    g_epoch_time = initial_epoch;
    g_pps_t2 = LL_TIM_GetCounter(TIM2);
    g_tps_est = 1000000; /* TIM2 is free-running at 1 MHz */
    update_scale(g_tps_est);
    g_phase = 0;
    g_pps_count = 0;
    g_set_pending = false;
    g_set_epoch = 0;
}

/** @brief Request to set system time at next PPS event
  * @param new_epoch New epoch time in seconds since 2000-01-01
  * @retval None
  * @note Time will be set on next PPS event
*/
void systime_request_update(uint64_t new_epoch) {
    g_set_epoch = new_epoch;
    g_set_pending = true;
}

/** @brief Handle PPS event from inside the TIM3 interrupt
  * @param None
  * @retval None
  * @note Updates epoch time, phase, and ticks per second estimate
  * @note This function avoids floating point and division in the ISR
*/
void systime_pps_event(void) {
    /* Sample the TIM2 counter */
    uint32_t t2_now = LL_TIM_GetCounter(TIM2);
    uint32_t delta_t2;
    /* Check for timer wrap-around, happens every hour and 11 minutes or so... */
    if (t2_now >= g_pps_t2) {
        /* Normal case, Delta = number of ticks since last PPS event */
        delta_t2 = t2_now - g_pps_t2;
    } else {
        /* Timer wrapped around: subtract current value from max */
        delta_t2 = 0xFFFFFFFF - g_pps_t2 + t2_now + 1;
    }
    g_pps_t2 = t2_now; /* Update the global last-PPS TIM2 counter */

    /* Update ticks-per-second estimate using an exponential moving average */
    if (g_pps_count > 0) {
        /** RATE_SHIFT = 4, so this averages over 16 PPS events.
          * Update the global TIM2 ticks per second estimate.
          * This is equivalent to g_tps_est = ((g_tps_est * 15) + delta_t2) / 16; */
        g_tps_est = ((g_tps_est * ( (1 << RATE_SHIFT) - 1)) + delta_t2) >> RATE_SHIFT;
        
        /* Update the scaling factor for microseconds per tick with the new estimate */
        update_scale(g_tps_est);
    }

    /* Update global epoch time by 1 second */
    g_epoch_time += 1;

    /* Determine the phase error: Actual - Expected ticks since last PPS */
    int64_t phase_error = (int64_t)delta_t2 - (int64_t)g_tps_est;
    
    /* Update the phase accumulator by adding the phase error. 
     * Shift the phase error right by 3 bits to avoid correcting on small differences. */
    g_phase += phase_error >> PHASE_SHIFT;

    /* Apply phase correction */
    if (g_phase > (int64_t)g_tps_est / 2) {
        /* If phase exceeds half of the estimated ticks per second:
         * Add a second to epoch time. 
         * Subtract ticks per second estimate from the phase accumulator. */
        g_phase -= g_tps_est;
        g_epoch_time += 1;
    } else if (g_phase < -(int64_t)g_tps_est / 2) {
        /* If phase is less than negative half of the estimated ticks per second:
         * Subtract 1 from epoch time. 
         * Add ticks per second estimate to the phase accumulator */
        g_phase += g_tps_est;
        if (g_epoch_time > 0) {
            g_epoch_time -= 1;
        }
    }

    /* Handle pending time-set */
    if (g_set_pending) {
        g_epoch_time = g_set_epoch;
        g_set_pending = false;

        /* Reset the phase accumulator for new time. */
        g_phase = 0;
    }

    g_pps_count++;
}

/** @brief Get current time in microseconds since 2000-01-01 00:00:00
  * @param None
  * @retval Current time in microseconds (Epoch * 1,000,000 + sub-second microseconds)
  * @note Combines RTC epoch time with TIM2 counter and phase adjustment
*/
uint64_t time_us_now(void) {
    __disable_irq();
    uint32_t counter = LL_TIM_GetCounter(TIM2);
    uint32_t pps_t2 = g_pps_t2;
    uint64_t epoch = g_epoch_time;
    uint64_t scale = g_scale_q32;
    uint64_t tps = g_tps_est;
    int64_t phase = g_phase;
    __enable_irq();

    /* Ticks since last PPS */
    int32_t delta = (int32_t)(counter - pps_t2);

    /* Apply phase correction */
    int64_t adjusted_delta = (int64_t)delta - phase;
    bool borrow = false;

    /* If the adjusted delta is negative we need to borrow from the next second */
    if (adjusted_delta < 0) {
        /* If the ticks per second estimate is valid, add it to the adjusted delta */
        if (tps) {
            adjusted_delta += tps;
            borrow = true;
        } else {
            /* Fallback to 1 MHz if tps is zero */
            adjusted_delta += 1000000ULL;
            borrow = true;
        }
    }

    /* Sub-Second Adjustment using the Q32 scaling factor. First cast to 64-bit.
     * Multiply the adjusted delta by the scale factor and shifts right by 32 bits.
     * The right-shift by 32 bits converts from Q32 fixed-point to integer microseconds.
     */
    uint64_t sub_us = ((uint64_t)adjusted_delta * scale) >> 32;

    /* Total seconds */
    uint64_t sec = epoch - (borrow ? 1ULL : 0ULL); /* Check if we're borrowing from the next second */

    /* Total time in microseconds */
    return sec * 1000000ULL + sub_us;
}

/** @brief Get current time in seconds since 2000-01-01 00:00:00
  * @param None
  * @retval Current time in seconds
  * @note Direct calculation avoiding microsecond conversion issues
*/
uint64_t time_s_now(void) {
    /* Disable interrupts to ensure atomic read of global variables */
    __disable_irq();
    uint32_t counter = LL_TIM_GetCounter(TIM2);
    uint32_t pps_t2 = g_pps_t2;
    uint64_t epoch = g_epoch_time;
    uint64_t tps = g_tps_est;
    int64_t phase = g_phase;
    __enable_irq();

    /* Ticks since last PPS */
    int32_t delta = (int32_t)(counter - pps_t2);

    /* Apply phase correction */
    int64_t adjusted_delta = (int64_t)delta - phase;

    /* Check if we need to borrow from the next second */
    bool borrow = false;
    if (adjusted_delta < 0) {
        if (tps) {
            adjusted_delta += tps;
            borrow = true;
        } else {
            adjusted_delta += 1000000ULL;
            borrow = true;
        }
    }

    /* Calculate seconds directly */
    uint64_t sec = epoch - (borrow ? 1ULL : 0ULL);
    
    return sec;
}

/** @brief Get formatted timestamp string for a given time in microseconds
  * @param usecs Time in microseconds since 2000-01-01 00:00:00
  * @retval Formatted timestamp string in format "MM-DD-YYYY,HH:MM:SS+MICROSECONDS"
  * @note Returns static buffer, not thread-safe
*/
const char* timestamp(uint64_t usecs) {
    static char buffer[64];
    
    uint64_t total_seconds = usecs / 1000000ULL;
    uint32_t microseconds = usecs % 1000000ULL;
    
    /* Normalize: ensure microseconds are in valid range */
    while (microseconds >= 1000000) {
        total_seconds += 1;
        microseconds -= 1000000;
    }
    
    RTC_DateTime_t dt = epoch_to_datetime(total_seconds);

    snprintf(buffer, sizeof(buffer), "<%02d-%02d-%04d,%02d:%02d:%02d+%06u µs>",
             dt.months, dt.days, dt.years + 2000,
             dt.hours, dt.minutes, dt.seconds,
             (unsigned int)microseconds);
    
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
    /* Calculate PPM deviation from 1 MHz */
    int64_t diff = (int64_t)g_tps_est - 1000000LL;
    int64_t numerator = diff * 1000000LL;
    int64_t ppm = numerator / 1000000LL;
    return (int32_t)ppm;
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
uint64_t systime_get_pps_count(void) {
    return g_pps_count;
}

/** @brief Convert RTC date/time to epoch time (seconds since 2000-01-01)
 *  @param dt Pointer to RTC_DateTime_t structure
 *  @retval Epoch time in seconds since 2000-01-01 00:00:00
 *  @note Assumes valid date/time in dt
 */
uint64_t datetime_to_epoch(const RTC_DateTime_t* dt) {
    uint32_t year = dt->years + 2000;
    uint32_t month = dt->months;
    uint32_t day = dt->days;
    uint32_t hour = dt->hours;
    uint32_t minute = dt->minutes;
    uint32_t second = dt->seconds;

    /* Days since 2000-01-01 */
    uint32_t days = (year - 2000) * 365 + (year - 2000) / 4 - (year - 2000) / 100 + (year - 2000) / 400;
    days += month_offset(month, is_leap(year));
    days += day - 1;

    /* Total seconds, cast to uint64_t */
    uint64_t epoch = (uint64_t)days * 86400ULL + (uint64_t)hour * 3600ULL + (uint64_t)minute * 60ULL + (uint64_t)second;
    return epoch;
}

/** @brief Convert epoch time to RTC date/time
 *  @param epoch Epoch time in seconds since 2000-01-01 00:00:00
 *  @retval RTC_DateTime_t structure with converted date/time
 *  @note Handles leap years and valid ranges
 */
RTC_DateTime_t epoch_to_datetime(uint64_t epoch) {
    RTC_DateTime_t dt;
    uint64_t total_seconds = epoch;
    uint32_t days = total_seconds / 86400ULL;
    uint32_t remaining_seconds = total_seconds % 86400ULL;

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
    dt.days = days + 1; /* Index-1 offset */

    /* Time components */
    dt.hours = remaining_seconds / 3600;
    remaining_seconds %= 3600;
    dt.minutes = remaining_seconds / 60;
    dt.seconds = remaining_seconds % 60;

    return dt;
}
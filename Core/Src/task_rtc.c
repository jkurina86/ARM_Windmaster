/**
  ******************************************************************************
  * @file    task_rtc.c
  * @brief   RTC task handlers implementation
  * @note    Contains handlers for RTC-related tasks
  ******************************************************************************
  */

#include "task_rtc.h"
#include "shell.h"
#include "ab-rtcmc-rtc.h"
#include "systime.h"
#include <stdbool.h>

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Handle RTC timer status task
 * @param arg Unused (pass NULL)
 */
void handle_rtc_timer_status(const void *arg)
{
    (void)arg;
    
    RTC_Timer_t timer;
    RTC_Status_t status = RTC_GetTimer(&timer);
    
    if (status != RTC_OK) {
        shell_printf("Error reading RTC timer status (error code: %d)\r\n", status);
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("RTC Timer Status:\r\n");
    shell_print("=================\r\n");
    shell_printf("Timer Value: %d\r\n", timer.timer_value);
    shell_printf("Division: 0x%02X ", timer.division);

    /* Decode division setting */
    switch(timer.division) {
        case RTC_TIMER_DIV_4096HZ:
            shell_print("(4096 Hz)\r\n");
            break;
        case RTC_TIMER_DIV_64HZ:
            shell_print("(64 Hz)\r\n");
            break;
        case RTC_TIMER_DIV_1HZ:
            shell_print("(1 Hz)\r\n");
            break;
        case RTC_TIMER_DIV_1_60HZ:
            shell_print("(1/60 Hz)\r\n");
            break;
        default:
            shell_print("(Unknown)\r\n");
            break;
    }
    
    shell_printf("Auto Reload: %s\r\n", timer.auto_reload ? "Enabled" : "Disabled");
    shell_printf("Timer Enabled: %s\r\n", timer.enabled ? "Yes" : "No");

    /* Check if timer flag is set */
    bool timer_flag = RTC_IsTimerTriggered();
    shell_printf("Timer Flag: %s\r\n", timer_flag ? "Set" : "Clear");
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle RTC timer stop task
 * @param arg Unused (pass NULL)
 */
void handle_rtc_timer_stop(const void *arg)
{
    (void)arg;
    
    /* Disable timer */
    RTC_Status_t status = RTC_EnableTimer(false);
    if (status != RTC_OK) {
        shell_printf("Error stopping RTC timer (error code: %d)\r\n", status);
        shell_print(SHELL_PROMPT);
        return;
    }

    /* Disable timer interrupt */
    status = RTC_EnableTimerInterrupt(false);
    if (status != RTC_OK) {
        shell_printf("Error disabling timer interrupt (error code: %d)\r\n", status);
        shell_print(SHELL_PROMPT);
        return;
    }

    /* Clear timer flag */
    RTC_ClearTimerFlag();
    
    shell_print("RTC Timer stopped\r\n");
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle RTC timer set task
 * @param arg Pointer to rtc_timer_args_t containing seconds
 */
void handle_rtc_timer_set(const void *arg)
{
    const rtc_timer_args_t *args = (const rtc_timer_args_t *)arg;
    uint16_t seconds = args->seconds;
    
    /* Validate input and provide usage information if invalid */
    if (seconds == 0 || seconds > 65535) {
        shell_print("Usage: rtc-timer-set <seconds>\r\n");
        shell_print("Set RTC timer for specified number of seconds\r\n");
        shell_print("Example: rtc-timer-set 10\r\n");
        shell_print("Error: Timer value must be between 1 and 65535 seconds\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }

    /* Configure timer structure */
    RTC_Timer_t timer;
    timer.timer_value = seconds;
    timer.division = RTC_TIMER_DIV_1HZ;  /* 1 Hz for seconds counting */
    timer.auto_reload = false;           /* Single shot timer */
    timer.enabled = true;

    /* Set the timer configuration */
    RTC_Status_t status = RTC_SetTimer(&timer);
    if (status != RTC_OK) {
        shell_printf("Error setting RTC timer (error code: %d)\r\n", status);
        shell_print(SHELL_PROMPT);
        return;
    }

    /* Clear any existing timer flag before enabling interrupts so we get a clean edge */
    RTC_ClearTimerFlag();

    /* Enable timer interrupt */
    status = RTC_EnableTimerInterrupt(true);
    if (status != RTC_OK) {
        shell_printf("Error enabling timer interrupt (error code: %d)\r\n", status);
        shell_print(SHELL_PROMPT);
        return;
    }
    
    /* Get the current time */
    RTC_DateTime_t current_time;
    RTC_GetDateTime(&current_time);

    shell_printf("RTC Timer started at: %02d:%02d:%02d\r\n",
                current_time.hours, current_time.minutes, current_time.seconds);
    shell_printf("RTC Timer started for %d seconds\r\n", seconds);
    shell_print("Timer will trigger an interrupt when it expires\r\n");
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle RTC temperature read task
 * @param arg Unused (pass NULL)
 */
void handle_rtc_temp(const void *arg)
{
    (void)arg;
    
    int8_t temperature;
    RTC_Status_t status = RTC_GetTemperature(&temperature);
    
    if (status != RTC_OK) {
        shell_printf("Error reading RTC temperature (error code: %d)\r\n", status);
        shell_print(SHELL_PROMPT);
        return;
    }
    
    shell_print("RTC Temperature:\r\n");
    shell_print("================\r\n");
    shell_printf("Temperature: %d°C\r\n", temperature);
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle RTC get time task
 * @param arg Unused (pass NULL)
 */
void handle_rtc_gettime(const void *arg)
{
    (void)arg;
    
    /* Create a DateTime struct and call the RTC to fill it */
    RTC_DateTime_t datetime;
    RTC_Status_t status = RTC_GetDateTime(&datetime);
    
    if (status != RTC_OK) {
        shell_printf("Error reading RTC time (error code: %d)\r\n", status);
        shell_print(SHELL_PROMPT);
        return;
    }
    
    /* Display the current date and time */
    const char* weekdays[] = {"Unknown", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    
    shell_print("Current RTC Date/Time:\r\n");
    shell_print("=====================\r\n");
    
    shell_printf("Date: 20%02d-%02d-%02d (%s)\r\n",
                datetime.years, datetime.months, datetime.days,
                weekdays[datetime.weekdays]);

    shell_printf("Time: %02d:%02d:%02d (24h)\r\n",
                datetime.hours, datetime.minutes, datetime.seconds);
    shell_print(SHELL_PROMPT);
}

/**
 * @brief Handle shell-driven RTC set time task
 * @param arg Pointer to rtc_settime_args_t containing parsed date/time
 */
void handle_rtc_settime(const void *arg)
{
    const rtc_settime_args_t *args = (const rtc_settime_args_t *)arg;
    
    /* Check if valid args were provided */
    if (!args->valid) {
        shell_print("Usage: rtc-settime YYYY MM DD HH MM SS WD\r\n");
        shell_print("WD: 1=Sunday, 2=Monday, 3=Tuesday, 4=Wednesday, 5=Thursday, 6=Friday, 7=Saturday\r\n");
        shell_print("Example: rtc-settime 2025 08 28 14 30 00 4\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    /* Get parsed parameters from args */
    uint16_t year = args->year;
    uint8_t months = args->months;
    uint8_t days = args->days;
    uint8_t hours = args->hours;
    uint8_t minutes = args->minutes;
    uint8_t seconds = args->seconds;
    uint8_t weekdays = args->weekdays;
    
    /* Validate input */
    if (year < 2000 || year > 2099) {
        shell_print("Error: Year must be between 2000-2099\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    if (months == 0 || months > 12) {
        shell_print("Error: Month must be between 1-12\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    if (days == 0 || days > 31) {
        shell_print("Error: Day must be between 1-31\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    if (hours > 23) {
        shell_print("Error: Hour must be between 0-23\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    if (minutes > 59 || seconds > 59) {
        shell_print("Error: Minutes and seconds must be between 0-59\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }

    if (weekdays < 1 || weekdays > 7) {
        shell_print("Error: Weekday must be between 1-7 (1=Sunday, 2=Monday, ..., 7=Saturday)\r\n");
        shell_print(SHELL_PROMPT);
        return;
    }
    
    /* Create datetime structure */
    RTC_DateTime_t datetime;
    datetime.years = (uint8_t)(year - 2000);  /* Convert to 2-digit year for the RTC */
    datetime.months = months;
    datetime.days = days;
    datetime.hours = hours;
    datetime.minutes = minutes;
    datetime.seconds = seconds;
    datetime.weekdays = weekdays;
    datetime.is_12h_format = false;
    datetime.is_pm = false;

    /* Set the RTC time */
    RTC_Status_t status = RTC_SetDateTime(&datetime);
    
    /* Update system time as well */
    if (status == RTC_OK) {
        systime_update(datetime_to_epoch(&datetime));
    }

    if (status == RTC_OK) {
        const char* weekdays_str[] = {"Unknown", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
        shell_printf("RTC time set to: 20%02d-%02d-%02d %02d:%02d:%02d (%s)\r\n",
                    datetime.years, datetime.months, datetime.days,
                    datetime.hours, datetime.minutes, datetime.seconds,
                    weekdays_str[datetime.weekdays]);
    } else {
        shell_printf("Error setting RTC time (error code: %d)\r\n", status);
    }
    
    shell_print(SHELL_PROMPT);
}

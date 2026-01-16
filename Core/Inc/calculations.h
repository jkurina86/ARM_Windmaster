/**
  ******************************************************************************
  * @file    calculations.h
  * @brief   Report calculations module
  * @note    Uses running sums updated per-sample. Non-blocking finalization.
  * 
  ******************************************************************************
  */
#ifndef __CALCULATIONS_H__
#define __CALCULATIONS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Definitions ---------------------------------------------------------------*/
#define CALC_PERIOD_SAMPLES     1200    /* 1 minute at 20Hz */
#define CALC_GUST_WINDOW        60      /* 3 seconds at 20Hz for gust calculation */
#define CALC_REPORT_BUFFER_SIZE 30      /* 30 minutes of reports in RAM2 */

/* Exported Types ------------------------------------------------------------*/

/**
 * @brief Sample data structure passed to calc_add_sample()
 * @note  Contains WindMaster and VectorNav fields for motion-corrected Earth-frame transform
 */
typedef struct {
    /* From WindMaster (platform frame) */
    int16_t u;          /* U-axis wind speed (mm/s) */
    int16_t v;          /* V-axis wind speed (mm/s) */
    int16_t w;          /* W-axis wind speed (mm/s) */
    /* From VectorNav - attitude (for Euler rotation) */
    float roll;         /* Roll angle (degrees) */
    float pitch;        /* Pitch angle (degrees) */
    float yaw;          /* Yaw/heading angle (degrees) */
    /* From VectorNav - angular velocity (for lever arm correction) */
    float gyro_x;       /* Angular velocity about X-axis (rad/s) */
    float gyro_y;       /* Angular velocity about Y-axis (rad/s) */
    float gyro_z;       /* Angular velocity about Z-axis (rad/s) */
    /* From VectorNav - position */
    double latitude;    /* Latitude (degrees) */
    double longitude;   /* Longitude (degrees) */
} CalcSample_t;

/**
 * @brief Results structure containing calculated wind statistics
 * @note  Updated every 60 seconds when accumulation period completes
 */
typedef struct {
    /* Wind FROM direction - degrees (0-360, 0=from North) */
    float wind_from_mean;
    float wind_from_stddev;

    /* Wind speed (horizontal magnitude) - m/s */
    float wind_speed_mean;
    float wind_speed_stddev;

    /* U wind component (East) - m/s */
    float u_mean;
    float u_stddev;

    /* V wind component (North) - m/s */
    float v_mean;
    float v_stddev;

    /* W wind component (Up) - m/s */
    float w_mean;
    float w_stddev;

    /* Gust (max 3-sec mean in 1 minute) - m/s */
    float gust_mean;
    float gust_stddev;

    /* Metadata */
    uint32_t timestamp_s;       /* Epoch seconds when calculation completed */
    uint16_t sample_count;      /* Number of samples in this period */
    uint8_t valid;              /* 1 if results are valid, 0 otherwise */
} CalcResults_t;

/**
 * @brief 1-minute report for CSV transmission over UART
 * @note  Contains Earth-frame wind statistics, updated each minute
 */
typedef struct {
    uint32_t timestamp_s;       /* Epoch seconds when report was generated */
    double latitude;            /* Latitude (degrees) */
    double longitude;           /* Longitude (degrees) */
    float u_mean;               /* U (East) wind component mean (m/s) */
    float u_std;                /* U (East) wind component stddev (m/s) */
    float v_mean;               /* V (North) wind component mean (m/s) */
    float v_std;                /* V (North) wind component stddev (m/s) */
    float w_mean;               /* W (Up) wind component mean (m/s) */
    float w_std;                /* W (Up) wind component stddev (m/s) */
    float wind_speed_mean;      /* Horizontal wind speed mean (m/s) */
    float wind_speed_std;       /* Horizontal wind speed stddev (m/s) */
    float wind_from_mean;       /* Wind FROM direction mean (degrees, 0-360, 0=from North) */
    float wind_from_std;        /* Wind FROM direction stddev (degrees) */
    float gust_mean;            /* Gust (max 3-sec mean) (m/s) */
    float gust_std;             /* Gust stddev (m/s) */
} CalcReport_t;

/* Public function prototypes ------------------------------------------------*/

void calc_init(void);
void calc_add_sample(const CalcSample_t *sample);
void calc_service(void);
CalcResults_t* calc_get_results(void);
uint8_t calc_results_ready(void);
CalcReport_t* calc_get_report_buffer(void);
uint16_t calc_get_report_count(void);
int16_t calc_get_report_head(void);
void calc_clear_reports(void);

#ifdef __cplusplus
}
#endif
#endif /* __CALCULATIONS_H__ */

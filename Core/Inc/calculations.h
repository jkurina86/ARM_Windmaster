/**
  ******************************************************************************
  * @file    calculations.h
  * @brief   Statistics calculations module header
  * @note    Incremental 1-minute statistics for wind data.
  *          Uses running sums for O(1) memory and non-blocking finalization.
  *          Computes mean, stddev, direction, and gust values.
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
} CalcSample_t;

/**
 * @brief Results structure containing calculated wind statistics
 * @note  Updated every 60 seconds when accumulation period completes
 */
typedef struct {
    /* Wind Direction (FROM) - degrees (0-360) */
    float wind_from_mean;
    float wind_from_stddev;

    /* Wind Speed (horizontal magnitude) - m/s */
    float wind_speed_mean;
    float wind_speed_stddev;

    /* U Wind component - m/s */
    float uwnd_mean;
    float uwnd_stddev;

    /* V Wind component - m/s */
    float vwnd_mean;
    float vwnd_stddev;

    /* W Wind component - m/s */
    float wwnd_mean;
    float wwnd_stddev;

    /* Gust (max 3-sec mean in 1 minute) - m/s */
    float gust_wnd_mean;
    float gust_wnd_stddev;

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
    float u_mean;               /* U (East) wind component mean (m/s) */
    float u_std;                /* U (East) wind component stddev (m/s) */
    float v_mean;               /* V (North) wind component mean (m/s) */
    float v_std;                /* V (North) wind component stddev (m/s) */
    float w_mean;               /* W (Up) wind component mean (m/s) */
    float w_std;                /* W (Up) wind component stddev (m/s) */
    float wspd_mean;            /* Horizontal wind speed mean (m/s) */
    float wspd_std;             /* Horizontal wind speed stddev (m/s) */
    float wdir_mean;            /* Wind direction mean (degrees, 0-360) */
    float wdir_std;             /* Wind direction stddev (degrees) */
    float gust_mean;            /* Gust (max 3-sec mean) (m/s) */
    float gust_std;             /* Gust stddev (m/s) */
} CalcReport_t;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief  Initialize the calculations module
 * @note   Clears accumulators and resets state. Call once at startup.
 */
void calc_init(void);

/**
 * @brief  Add a sample and update running statistics
 * @param  sample: Pointer to CalcSample_t with WindMaster data
 * @note   Called from recorder at 20Hz when records are paired.
 *         Incrementally updates all running sums.
 *         When period completes (1200 samples), snapshots accumulators
 *         and sets ready flag for calc_service() to finalize.
 */
void calc_add_sample(const CalcSample_t *sample);

/**
 * @brief  Service function to finalize ready calculations
 * @note   Call from main loop. Finalizes statistics when period completes.
 *         Non-blocking: only does fast arithmetic (no iteration).
 *         Uses state machine to yield between stages if needed.
 */
void calc_service(void);

/**
 * @brief  Get pointer to latest calculation results
 * @retval Pointer to CalcResults_t structure
 * @note   Check results->valid before using values.
 */
CalcResults_t* calc_get_results(void);

/**
 * @brief  Check if new results are available
 * @retval 1 if new results ready since last check, 0 otherwise
 * @note   Clears the "new results" flag when called.
 */
uint8_t calc_results_ready(void);

/**
 * @brief  Get pointer to report buffer in RAM2
 * @retval Pointer to CalcReport_t array of size CALC_REPORT_BUFFER_SIZE
 */
CalcReport_t* calc_get_report_buffer(void);

/**
 * @brief  Get current number of reports in buffer
 * @retval Number of valid reports (0 to CALC_REPORT_BUFFER_SIZE)
 */
uint16_t calc_get_report_count(void);

/**
 * @brief  Get index of most recent report in buffer
 * @retval Index of newest report, or -1 if buffer is empty
 */
int16_t calc_get_report_head(void);

#ifdef __cplusplus
}
#endif
#endif /* __CALCULATIONS_H__ */

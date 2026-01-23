/**
  ******************************************************************************
  * @file    calculations.c
  * @brief   Report calculations module
  * @note    Uses running sums updated per-sample. Non-blocking finalization.
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "calculations.h"
#include "systime.h"
#include <math.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define WM_SCALE  0.01f             /* Windmaster output scale (cm/s to m/s) */
#define DEG_TO_RAD 0.01745329252f   /* M_PI / 180 */
#define RAD_TO_DEG 57.29577951f     /* 180 / M_PI */

/* Lever arm: IMU to WM offset vector R [m] in platform frame */
#define LEVER_ARM_X  0.0f           /* X offset (lateral) */
#define LEVER_ARM_Y  0.0f           /* Y offset (forward/aft) */
#define LEVER_ARM_Z  0.8f           /* Z offset (IMU 0.8m below WM) */

/* Private types -------------------------------------------------------------*/

/**
 * @brief Running accumulators for moving averages
 * @note  Updated in calc_add_sample(), finalized in calc_service()
 */
typedef struct {
    uint16_t n;                 /* Sample count */

    /* Wind FROM direction accumulators (circular) */
    float wind_from_sin_sum;
    float wind_from_cos_sum;

    /* Wind speed accumulators */
    float wind_speed_sum;
    float wind_speed_sum_sq;

    /* U component accumulators */
    float u_sum;
    float u_sum_sq;

    /* V component accumulators */
    float v_sum;
    float v_sum_sq;

    /* W component accumulators */
    float w_sum;
    float w_sum_sq;

    /* Gust: 60-sample circular buffer for sliding window */
    float gust_ring[CALC_GUST_WINDOW];  /* Wind speeds for sliding window */
    uint8_t gust_idx;                   /* Current position in ring buffer */
    uint8_t gust_ring_full;             /* Ring buffer has been filled once */
    float gust_window_sum;              /* Sum of current 60-sample window */
    float gust_window_sum_sq;           /* Sum of squares for current window */
    float gust_max_mean;                /* Maximum 3-sec mean seen */
    float gust_max_sum;                 /* Sum at time of max (for stddev) */
    float gust_max_sum_sq;              /* Sum_sq at time of max (for stddev) */

    /* Position (latest sample) */
    double latitude;
    double longitude;
} CalcAccum_t;

/**
 * @brief Finalization state machine
 */
typedef enum {
    CALC_STATE_COLLECTING,          /* Accumulating samples */
    CALC_STATE_READY,               /* Period complete, ready to finalize */
    CALC_STATE_FINALIZE_SPEED,      /* Finalizing speed statistics */
    CALC_STATE_FINALIZE_DIRECTION,  /* Finalizing direction statistics */
    CALC_STATE_FINALIZE_GUST,       /* Finalizing gust statistics */
    CALC_STATE_DONE                 /* Results ready */
} CalcState_t;

/* Private variables ---------------------------------------------------------*/

/* Double-buffered accumulators */
static CalcAccum_t accum_active;    /* Being updated by calc_add_sample */
static CalcAccum_t accum_ready;     /* Awaiting finalization */

/* State machine */
static volatile CalcState_t state = CALC_STATE_COLLECTING;

/* Results */
static CalcResults_t results;
static uint8_t new_results_flag = 0;

/* Report buffer in RAM2 (circular buffer for 30 minutes of reports) */
__attribute__((section(".calc_reports")))
static CalcReport_t report_buffer[CALC_REPORT_BUFFER_SIZE];

static uint16_t report_count = 0;       /* Number of valid reports in buffer */
static uint16_t report_head = 0;        /* Index of next write position */

/* Private function prototypes -----------------------------------------------*/
static void store_report(void);
static void reset_accumulators(CalcAccum_t *a);
static void finalize_speed(void);
static void finalize_direction(void);
static void finalize_gust(void);
static void platform_to_earth(float u_p, float v_p, float w_p,
                              float roll, float pitch, float yaw,
                              float *u_e, float *v_e, float *w_e);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initialize the calculations module
 */
void calc_init(void) {
    reset_accumulators(&accum_active);
    memset(&accum_ready, 0, sizeof(CalcAccum_t));
    memset(&results, 0, sizeof(CalcResults_t));
    state = CALC_STATE_COLLECTING;
    new_results_flag = 0;
}

/**
 * @brief  Add a sample and update running statistics
 * @param  sample: Pointer to CalcSample_t with WindMaster and attitude data
 * @note   Wind is motion-corrected and transformed to Earth frame:
 *         U_earth = T(roll,pitch,yaw) × [U_obs + Omega * R]
 *         where Omega × R accounts for windmaster velocity due to platform rotation
 */
void calc_add_sample(const CalcSample_t *sample) {
    /* Convert windmaster output from cm/s to m/s */
    float u_obs = sample->u * WM_SCALE;
    float v_obs = sample->v * WM_SCALE;
    float w_obs = sample->w * WM_SCALE;

    /* Compute angular velocity correction: Omega * R
     * Cross product: [gy*Rz - gz*Ry, gz*Rx - gx*Rz, gx*Ry - gy*Rx]
     * With R = (0, 0, 0.8): correction = (0.8*gy, -0.8*gx, 0) */
    float omega_cross_r_x = sample->gyro_y * LEVER_ARM_Z - sample->gyro_z * LEVER_ARM_Y;
    float omega_cross_r_y = sample->gyro_z * LEVER_ARM_X - sample->gyro_x * LEVER_ARM_Z;
    float omega_cross_r_z = sample->gyro_x * LEVER_ARM_Y - sample->gyro_y * LEVER_ARM_X;

    /* Apply angular velocity correction: U_corrected = U_obs + Omega × R */
    float u_corrected = u_obs + omega_cross_r_x;
    float v_corrected = v_obs + omega_cross_r_y;
    float w_corrected = w_obs + omega_cross_r_z;

    /* Convert Euler angles from degrees to radians */
    float roll_rad = sample->roll * DEG_TO_RAD;
    float pitch_rad = sample->pitch * DEG_TO_RAD;
    float yaw_rad = sample->yaw * DEG_TO_RAD;

    /* Transform corrected wind from platform frame to Earth frame */
    float u_m, v_m, w_m;
    platform_to_earth(u_corrected, v_corrected, w_corrected,
                      roll_rad, pitch_rad, yaw_rad,
                      &u_m, &v_m, &w_m);

    /* Compute derived values from Earth-frame wind */
    float wind_speed = sqrtf(u_m * u_m + v_m * v_m);
    /* Wind FROM direction: negate components to get source direction */
    float wind_from_rad = atan2f(-u_m, -v_m);

    /* Update Wind FROM direction accumulators (circular) */
    accum_active.wind_from_sin_sum += sinf(wind_from_rad);
    accum_active.wind_from_cos_sum += cosf(wind_from_rad);
    /* Update Wind speed accumulators */
    accum_active.wind_speed_sum += wind_speed;
    accum_active.wind_speed_sum_sq += wind_speed * wind_speed;
    /* Update U/V/W accumulators */
    accum_active.u_sum += u_m;
    accum_active.u_sum_sq += u_m * u_m;
    accum_active.v_sum += v_m;
    accum_active.v_sum_sq += v_m * v_m;
    accum_active.w_sum += w_m;
    accum_active.w_sum_sq += w_m * w_m;

    /* Update Gust sliding window */
    /* Remove old value from sums, add new value */
    float old_windspeed = accum_active.gust_ring[accum_active.gust_idx];
    accum_active.gust_window_sum -= old_windspeed;
    accum_active.gust_window_sum_sq -= old_windspeed * old_windspeed;

    accum_active.gust_ring[accum_active.gust_idx] = wind_speed;
    accum_active.gust_window_sum += wind_speed ;
    accum_active.gust_window_sum_sq += wind_speed * wind_speed;

    accum_active.gust_idx++;
    if (accum_active.gust_idx >= CALC_GUST_WINDOW) {
        accum_active.gust_idx = 0;
        accum_active.gust_ring_full = 1;
    }

    /* Check for new maximum 3-sec mean (only once ring is full) */
    if (accum_active.gust_ring_full) {
        float window_mean = accum_active.gust_window_sum / CALC_GUST_WINDOW;
        if (window_mean > accum_active.gust_max_mean) {
            accum_active.gust_max_mean = window_mean;
            accum_active.gust_max_sum = accum_active.gust_window_sum;
            accum_active.gust_max_sum_sq = accum_active.gust_window_sum_sq;
        }
    }

    /* Store latest position */
    accum_active.latitude = sample->latitude;
    accum_active.longitude = sample->longitude;

    /* Increment sample count */
    accum_active.n++;

    /* Check if period is complete */
    if (accum_active.n >= CALC_PERIOD_SAMPLES) {
        /* Capture timestamp at end of period */
        results.timestamp_s = time_s_now();

        /* Copy active to ready buffer for finalization */
        memcpy(&accum_ready, &accum_active, sizeof(CalcAccum_t));

        /* Reset accumulators for next period */
        reset_accumulators(&accum_active);

        /* Signal ready for finalization */
        state = CALC_STATE_READY;
    }
}

/**
 * @brief  Service function to finalize ready calculations
 * @note   State machine allows yields between stages
 * 
 */
void calc_service(void) {
    switch (state) {
        case CALC_STATE_READY:
            state = CALC_STATE_FINALIZE_SPEED;
            /* fallthrough */

        case CALC_STATE_FINALIZE_SPEED:
            finalize_speed();
            state = CALC_STATE_FINALIZE_DIRECTION;
            break;

        case CALC_STATE_FINALIZE_DIRECTION:
            finalize_direction();
            state = CALC_STATE_FINALIZE_GUST;
            break;

        case CALC_STATE_FINALIZE_GUST:
            finalize_gust();

            /* Finalization complete - set metadata */
            results.sample_count = accum_ready.n;
            results.valid = 1;
            new_results_flag = 1;

            /* Store report to RAM2 circular buffer */
            store_report();

            state = CALC_STATE_DONE;
            break;

        case CALC_STATE_DONE:
            /* Stay in done state until next period triggers */
            state = CALC_STATE_COLLECTING;
            break;

        case CALC_STATE_COLLECTING:
        default:
            /* Nothing to do */
            break;
    }
}

/**
 * @brief  Get pointer to latest calculation results
 * @retval Pointer to CalcResults_t structure
 * @note   Check if results are valid before using values.
 */
CalcResults_t* calc_get_results(void) {
    return &results;
}

/**
 * @brief  Check if new results are available
 * @retval 1 if new results ready since last check, 0 otherwise
 * @note   Clears the "new results" flag when called.
 */
uint8_t calc_results_ready(void) {
    uint8_t ready = new_results_flag;
    new_results_flag = 0;
    return ready;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Reset accumulators to initial state
 * @param  a: Pointer to CalcAccum_t structure to reset
 * @retval None
 */
static void reset_accumulators(CalcAccum_t *a) {
    memset(a, 0, sizeof(CalcAccum_t));
}

/**
 * @brief  Finalize speed statistics (wind speed, U, V, W)
 * @note   Computes mean and stddev from running sums
 */
static void finalize_speed(void) {
    const float n = (float)accum_ready.n;
    if (n < 1.0f) return;

    /* Wind speed */
    results.wind_speed_mean = accum_ready.wind_speed_sum / n;
    float wind_speed_var = (accum_ready.wind_speed_sum_sq / n) - (results.wind_speed_mean * results.wind_speed_mean);
    if (wind_speed_var > 0) {
        results.wind_speed_stddev = sqrtf(wind_speed_var);
    } else {
        results.wind_speed_stddev = 0;
    }

    /* U component */
    results.u_mean = accum_ready.u_sum / n;
    float u_var = (accum_ready.u_sum_sq / n) - (results.u_mean * results.u_mean);
    if (u_var > 0) {
        results.u_stddev = sqrtf(u_var);
    } else {
        results.u_stddev = 0;
    }

    /* V component */
    results.v_mean = accum_ready.v_sum / n;
    float v_var = (accum_ready.v_sum_sq / n) - (results.v_mean * results.v_mean);
    if (v_var > 0) {
        results.v_stddev = sqrtf(v_var);
    } else {
        results.v_stddev = 0;
    }

    /* W component */
    results.w_mean = accum_ready.w_sum / n;
    float w_var = (accum_ready.w_sum_sq / n) - (results.w_mean * results.w_mean);
    if (w_var > 0) {
        results.w_stddev = sqrtf(w_var);
    } else {
        results.w_stddev = 0;
    }
}

/**
 * @brief  Finalize direction statistics (wind FROM direction)
 * @note   Uses resultant length method for circular stddev
 */
static void finalize_direction(void) {
    /* Cast to float for division */
    const float n = (float)accum_ready.n;
    if (n < 1.0f) return;

    /* Wind FROM direction */
    float wind_from_mean_rad = atan2f(accum_ready.wind_from_sin_sum / n, accum_ready.wind_from_cos_sum / n);
    results.wind_from_mean = wind_from_mean_rad * RAD_TO_DEG;
    if (results.wind_from_mean < 0) results.wind_from_mean += 360.0f;

    /* Wind FROM direction stddev */
    float wind_from_R = sqrtf(accum_ready.wind_from_sin_sum * accum_ready.wind_from_sin_sum +
                              accum_ready.wind_from_cos_sum * accum_ready.wind_from_cos_sum) / n;
    if (wind_from_R > 0.001f && wind_from_R < 1.0f) {
        results.wind_from_stddev = sqrtf(-2.0f * logf(wind_from_R)) * RAD_TO_DEG;
    } else if (wind_from_R >= 1.0f) {
        results.wind_from_stddev = 0;
    } else {
        results.wind_from_stddev = 180.0f;
    }
}

/**
 * @brief  Finalize gust statistics
 * @note   Computes mean and stddev of maximum 3-sec window
 */
static void finalize_gust(void) {     
    results.gust_mean = accum_ready.gust_max_mean;

    /* Compute stddev of the 3-sec window that had maximum mean */
    if (accum_ready.gust_ring_full) {
        float gust_window_mean = accum_ready.gust_max_sum / CALC_GUST_WINDOW;
        float gust_var = (accum_ready.gust_max_sum_sq / CALC_GUST_WINDOW) - (gust_window_mean * gust_window_mean);
        if (gust_var > 0) {
            results.gust_stddev = sqrtf(gust_var);
        } else {
            results.gust_stddev = 0;
        }
    } else {
        results.gust_stddev = 0;
    }
}

/**
 * @brief  Store current results to report buffer
 * @retval None
 */
static void store_report(void) {
    CalcReport_t *report = &report_buffer[report_head];

    report->timestamp_s = results.timestamp_s;
    report->latitude = accum_ready.latitude;
    report->longitude = accum_ready.longitude;
    report->u_mean = results.u_mean;
    report->u_std = results.u_stddev;
    report->v_mean = results.v_mean;
    report->v_std = results.v_stddev;
    report->w_mean = results.w_mean;
    report->w_std = results.w_stddev;
    report->wind_speed_mean = results.wind_speed_mean;
    report->wind_speed_std = results.wind_speed_stddev;
    report->wind_from_mean = results.wind_from_mean;
    report->wind_from_std = results.wind_from_stddev;
    report->gust_mean = results.gust_mean;
    report->gust_std = results.gust_stddev;

    /* Advance head (circular) */
    report_head++;
    if (report_head >= CALC_REPORT_BUFFER_SIZE) {
        report_head = 0;
    }

    /* Track count (saturates at buffer size) */
    if (report_count < CALC_REPORT_BUFFER_SIZE) {
        report_count++;
    }
}

/**
 * @brief  Get pointer to report buffer in RAM2
 * @retval Pointer to CalcReport_t array of size CALC_REPORT_BUFFER_SIZE
 */
CalcReport_t* calc_get_report_buffer(void) {
    return report_buffer;
}

/**
 * @brief  Get current number of reports in buffer
 * @retval Number of valid reports (0 to CALC_REPORT_BUFFER_SIZE)
 */
uint16_t calc_get_report_count(void) {
    return report_count;
}

/**
 * @brief  Get index of most recent report in buffer
 * @retval Index of most recent report, or -1 if buffer is empty
 */
int16_t calc_get_report_head(void) {
    if (report_count == 0) {
        return -1;
    }
    /* Head points to next write position, so most recent is head - 1 */
    if (report_head == 0) {
        return CALC_REPORT_BUFFER_SIZE - 1;
    } else {
        return report_head - 1;
    }
}

/**
 * @brief  Clear all reports from buffer
 * @note   Called after CSV transmission to Telus system
 */
void calc_clear_reports(void) {
    report_count = 0;
    report_head = 0;
}

/**
 * @brief  Transform wind vector from platform frame to Earth frame using Euler angles
 * @param  u_p, v_p, w_p: Wind components in platform frame (m/s)
 * @param  roll, pitch, yaw: Euler angles (radians)
 * @param  u_e, v_e, w_e: Output wind components in Earth frame (m/s)
 * @note   Uses ZYX rotation sequence: R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *         This rotates vectors from platform (body) frame to Earth (NED) frame.
 */
static void platform_to_earth(float u_p, float v_p, float w_p,
                              float roll, float pitch, float yaw,
                              float *u_e, float *v_e, float *w_e)
{
    /* Precompute trig functions */
    float cos_phi = cosf(roll);
    float sin_phi = sinf(roll);
    float cos_theta = cosf(pitch);
    float sin_theta = sinf(pitch);
    float cos_psi = cosf(yaw);
    float sin_psi = sinf(yaw);

    /* Rotation matrix T (platform to Earth):
     * Row 0: [cos_psi*cos_theta, -sin_psi*cos_phi + cos_psi*sin_theta*sin_phi, sin_psi*sin_phi + cos_psi*sin_theta*cos_phi]
     * Row 1: [sin_psi*cos_theta,  cos_psi*cos_phi + sin_psi*sin_theta*sin_phi, sin_psi*sin_theta*cos_phi - cos_psi*sin_phi]
     * Row 2: [-sin_theta,                   cos_theta*sin_phi,                                           cos_theta*cos_phi]
     */
    *u_e = (cos_psi * cos_theta) * u_p +
           (-sin_psi * cos_phi + cos_psi * sin_theta * sin_phi) * v_p +
           (sin_psi * sin_phi + cos_psi * sin_theta * cos_phi) * w_p;

    *v_e = (sin_psi * cos_theta) * u_p +
           (cos_psi * cos_phi + sin_psi * sin_theta * sin_phi) * v_p +
           (sin_psi * sin_theta * cos_phi - cos_psi * sin_phi) * w_p;

    *w_e = (-sin_theta) * u_p +
           (cos_theta * sin_phi) * v_p +
           (cos_theta * cos_phi) * w_p;
}

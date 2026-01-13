/**
  ******************************************************************************
  * @file    calculations.c
  * @brief   Statistics calculations module - incremental implementation
  * @note    Uses running sums updated per-sample for O(1) memory usage.
  *          Finalization is non-blocking (~20 arithmetic ops, no iteration).
  *          Computes wind statistics from WindMaster records at 20Hz.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "calculations.h"
#include "systime.h"
#include <math.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define MM_TO_M    0.001f
#define DEG_TO_RAD 0.01745329252f   /* M_PI / 180 */
#define RAD_TO_DEG 57.29577951f     /* 180 / M_PI */

/* Lever arm: IMU to anemometer offset vector R [m] in platform frame */
#define LEVER_ARM_X  0.0f           /* X offset (lateral) */
#define LEVER_ARM_Y  0.0f           /* Y offset (forward/aft) */
#define LEVER_ARM_Z  0.8f           /* Z offset (IMU 0.8m below anemometer) */

/* Private types -------------------------------------------------------------*/

/**
 * @brief Running accumulators for incremental statistics
 * @note  Updated in calc_add_sample(), finalized in calc_service()
 */
typedef struct {
    uint16_t n;                 /* Sample count */

    /* Wind direction accumulators (circular) */
    float wdir_sin_sum;
    float wdir_cos_sum;

    /* Wind speed accumulators */
    float wspd_sum;
    float wspd_sum_sq;

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
} CalcAccum_t;

/**
 * @brief Finalization state machine
 */
typedef enum {
    CALC_STATE_COLLECTING,      /* Accumulating samples */
    CALC_STATE_READY,           /* Period complete, ready to finalize */
    CALC_STATE_FINALIZE_LINEAR, /* Finalizing linear statistics */
    CALC_STATE_FINALIZE_CIRC,   /* Finalizing circular statistics */
    CALC_STATE_FINALIZE_GUST,   /* Finalizing gust statistics */
    CALC_STATE_DONE             /* Results ready */
} CalcState_t;

/* Private variables ---------------------------------------------------------*/

/* Active accumulators (being updated by calc_add_sample) */
static CalcAccum_t accum;

/* Snapshot of accumulators for finalization (copied when period completes) */
static CalcAccum_t accum_snapshot;

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
static void finalize_linear(void);
static void finalize_circular(void);
static void finalize_gust(void);

/**
 * @brief  Transform wind vector from platform frame to Earth frame using Euler angles
 * @param  u_p, v_p, w_p: Wind components in platform frame (m/s)
 * @param  roll, pitch, yaw: Euler angles (radians)
 * @param  u_e, v_e, w_e: Output wind components in Earth frame (m/s)
 * @note   Uses ZYX aerospace rotation sequence: R = Rz(yaw) * Ry(pitch) * Rx(roll)
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
     * Row 2: [-sin_theta,         cos_theta*sin_phi,                           cos_theta*cos_phi]
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

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initialize the calculations module
 */
void calc_init(void)
{
    reset_accumulators(&accum);
    memset(&accum_snapshot, 0, sizeof(CalcAccum_t));
    memset(&results, 0, sizeof(CalcResults_t));
    state = CALC_STATE_COLLECTING;
    new_results_flag = 0;
}

/**
 * @brief  Add a sample and update running statistics
 * @param  sample: Pointer to CalcSample_t with WindMaster and attitude data
 * @note   Wind is motion-corrected and transformed to Earth frame:
 *         U_earth = T(roll,pitch,yaw) × [U_obs + Omega × R]
 *         where Omega × R accounts for anemometer velocity due to platform rotation
 */
void calc_add_sample(const CalcSample_t *sample)
{
    /* Convert platform-frame wind from mm/s to m/s */
    float u_obs = sample->u * MM_TO_M;
    float v_obs = sample->v * MM_TO_M;
    float w_obs = sample->w * MM_TO_M;

    /* Compute angular velocity correction: Omega × R
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
    float wspd = sqrtf(u_m * u_m + v_m * v_m);
    float wdir_rad = atan2f(u_m, v_m);

    /* Update Wind direction accumulators (circular) */
    accum.wdir_sin_sum += sinf(wdir_rad);
    accum.wdir_cos_sum += cosf(wdir_rad);

    /* Update Wind speed accumulators */
    accum.wspd_sum += wspd;
    accum.wspd_sum_sq += wspd * wspd;

    /* Update U/V/W accumulators */
    accum.u_sum += u_m;
    accum.u_sum_sq += u_m * u_m;
    accum.v_sum += v_m;
    accum.v_sum_sq += v_m * v_m;
    accum.w_sum += w_m;
    accum.w_sum_sq += w_m * w_m;

    /* Update Gust sliding window */
    /* Remove old value from sums, add new value */
    float old_wspd = accum.gust_ring[accum.gust_idx];
    accum.gust_window_sum -= old_wspd;
    accum.gust_window_sum_sq -= old_wspd * old_wspd;

    accum.gust_ring[accum.gust_idx] = wspd;
    accum.gust_window_sum += wspd;
    accum.gust_window_sum_sq += wspd * wspd;

    accum.gust_idx++;
    if (accum.gust_idx >= CALC_GUST_WINDOW) {
        accum.gust_idx = 0;
        accum.gust_ring_full = 1;
    }

    /* Check for new maximum 3-sec mean (only once ring is full) */
    if (accum.gust_ring_full) {
        float window_mean = accum.gust_window_sum / CALC_GUST_WINDOW;
        if (window_mean > accum.gust_max_mean) {
            accum.gust_max_mean = window_mean;
            accum.gust_max_sum = accum.gust_window_sum;
            accum.gust_max_sum_sq = accum.gust_window_sum_sq;
        }
    }

    /* Increment sample count */
    accum.n++;

    /* Check if period is complete */
    if (accum.n >= CALC_PERIOD_SAMPLES) {
        /* Snapshot accumulators for finalization */
        memcpy(&accum_snapshot, &accum, sizeof(CalcAccum_t));

        /* Reset accumulators for next period */
        reset_accumulators(&accum);

        /* Signal ready for finalization */
        state = CALC_STATE_READY;
    }
}

/**
 * @brief  Service function to finalize ready calculations
 * @note   State machine allows yielding between stages
 */
void calc_service(void)
{
    switch (state) {
        case CALC_STATE_READY:
            /* Start finalization - transition to first stage */
            state = CALC_STATE_FINALIZE_LINEAR;
            /* Fall through to start immediately */
            /* fallthrough */

        case CALC_STATE_FINALIZE_LINEAR:
            finalize_linear();
            state = CALC_STATE_FINALIZE_CIRC;
            break;

        case CALC_STATE_FINALIZE_CIRC:
            finalize_circular();
            state = CALC_STATE_FINALIZE_GUST;
            break;

        case CALC_STATE_FINALIZE_GUST:
            finalize_gust();

            /* Finalization complete - set metadata */
            uint16_t dummy_ms;
            systime_snapshot(&results.timestamp_s, &dummy_ms);
            results.sample_count = accum_snapshot.n;
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
 */
CalcResults_t* calc_get_results(void)
{
    return &results;
}

/**
 * @brief  Check if new results are available
 */
uint8_t calc_results_ready(void)
{
    uint8_t ready = new_results_flag;
    new_results_flag = 0;
    return ready;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Reset accumulators to initial state
 */
static void reset_accumulators(CalcAccum_t *a)
{
    memset(a, 0, sizeof(CalcAccum_t));
}

/**
 * @brief  Finalize linear statistics (Wind speed, U, V, W)
 */
static void finalize_linear(void)
{
    const float n = (float)accum_snapshot.n;
    if (n < 1.0f) return;

    /* Wind speed */
    results.wind_speed_mean = accum_snapshot.wspd_sum / n;
    float wspd_var = (accum_snapshot.wspd_sum_sq / n) - (results.wind_speed_mean * results.wind_speed_mean);
    results.wind_speed_stddev = (wspd_var > 0) ? sqrtf(wspd_var) : 0;

    /* U component */
    results.uwnd_mean = accum_snapshot.u_sum / n;
    float u_var = (accum_snapshot.u_sum_sq / n) - (results.uwnd_mean * results.uwnd_mean);
    results.uwnd_stddev = (u_var > 0) ? sqrtf(u_var) : 0;

    /* V component */
    results.vwnd_mean = accum_snapshot.v_sum / n;
    float v_var = (accum_snapshot.v_sum_sq / n) - (results.vwnd_mean * results.vwnd_mean);
    results.vwnd_stddev = (v_var > 0) ? sqrtf(v_var) : 0;

    /* W component */
    results.wwnd_mean = accum_snapshot.w_sum / n;
    float w_var = (accum_snapshot.w_sum_sq / n) - (results.wwnd_mean * results.wwnd_mean);
    results.wwnd_stddev = (w_var > 0) ? sqrtf(w_var) : 0;
}

/**
 * @brief  Finalize circular statistics (Wind direction)
 * @note   Uses resultant length method for circular stddev
 */
static void finalize_circular(void)
{
    const float n = (float)accum_snapshot.n;
    if (n < 1.0f) return;

    /* Wind direction (FROM) */
    float wdir_mean_rad = atan2f(accum_snapshot.wdir_sin_sum / n, accum_snapshot.wdir_cos_sum / n);
    results.wind_from_mean = wdir_mean_rad * RAD_TO_DEG;
    if (results.wind_from_mean < 0) results.wind_from_mean += 360.0f;

    /* Wind direction circular stddev */
    float wdir_R = sqrtf(accum_snapshot.wdir_sin_sum * accum_snapshot.wdir_sin_sum +
                         accum_snapshot.wdir_cos_sum * accum_snapshot.wdir_cos_sum) / n;
    if (wdir_R > 0.001f && wdir_R < 1.0f) {
        results.wind_from_stddev = sqrtf(-2.0f * logf(wdir_R)) * RAD_TO_DEG;
    } else if (wdir_R >= 1.0f) {
        results.wind_from_stddev = 0;
    } else {
        results.wind_from_stddev = 180.0f;
    }
}

/**
 * @brief  Finalize gust statistics
 */
static void finalize_gust(void)
{
    results.gust_wnd_mean = accum_snapshot.gust_max_mean;

    /* Compute stddev of the 3-sec window that had maximum mean */
    if (accum_snapshot.gust_ring_full) {
        float gust_mean = accum_snapshot.gust_max_sum / CALC_GUST_WINDOW;
        float gust_var = (accum_snapshot.gust_max_sum_sq / CALC_GUST_WINDOW) -
                         (gust_mean * gust_mean);
        results.gust_wnd_stddev = (gust_var > 0) ? sqrtf(gust_var) : 0;
    } else {
        results.gust_wnd_stddev = 0;
    }
}

/**
 * @brief  Store current results to report buffer
 */
static void store_report(void)
{
    CalcReport_t *report = &report_buffer[report_head];

    report->timestamp_s = results.timestamp_s;
    report->u_mean = results.uwnd_mean;
    report->u_std = results.uwnd_stddev;
    report->v_mean = results.vwnd_mean;
    report->v_std = results.vwnd_stddev;
    report->w_mean = results.wwnd_mean;
    report->w_std = results.wwnd_stddev;
    report->wspd_mean = results.wind_speed_mean;
    report->wspd_std = results.wind_speed_stddev;
    report->wdir_mean = results.wind_from_mean;
    report->wdir_std = results.wind_from_stddev;
    report->gust_mean = results.gust_wnd_mean;
    report->gust_std = results.gust_wnd_stddev;

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
 */
CalcReport_t* calc_get_report_buffer(void)
{
    return report_buffer;
}

/**
 * @brief  Get current number of reports in buffer
 */
uint16_t calc_get_report_count(void)
{
    return report_count;
}

/**
 * @brief  Get index of most recent report in buffer
 */
int16_t calc_get_report_head(void)
{
    if (report_count == 0) {
        return -1;
    }
    /* Head points to next write position, so most recent is head - 1 */
    return (report_head == 0) ? (CALC_REPORT_BUFFER_SIZE - 1) : (report_head - 1);
}

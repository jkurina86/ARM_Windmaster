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
#define RAD_TO_DEG 57.29577951f     /* 180 / M_PI */

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

/* Private function prototypes -----------------------------------------------*/
static void reset_accumulators(CalcAccum_t *a);
static void finalize_linear(void);
static void finalize_circular(void);
static void finalize_gust(void);

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
 * @param  sample: Pointer to CalcSample_t with WindMaster data
 */
void calc_add_sample(const CalcSample_t *sample)
{
    /* Convert to meters and compute derived values */
    float u_m = sample->u * MM_TO_M;
    float v_m = sample->v * MM_TO_M;
    float w_m = sample->w * MM_TO_M;
    float wspd = sqrtf(u_m * u_m + v_m * v_m);
    float wdir_rad = atan2f((float)sample->u, (float)sample->v);

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

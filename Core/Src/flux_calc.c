/**
  ******************************************************************************
  * @file    flux_calc.c
  * @brief   Flux calculation functions for eddy covariance measurements
  * @note    Implements motion correction for moving platform flux measurements
  *          
  *          Motion-corrected wind velocity:
  *          U_true = T(phi,theta,psi)[U_obs + Omega × R] + V_hp + V_lp
  *          
  *          Where:
  *          - T(phi,theta,psi) Euler rotation matrix transforming platform to Earth frame
  *          - U_obs is observed wind in platform frame
  *          - Omega is angular velocity from VN300 gyro
  *          - R is lever arm from IMU to WM
  *          - V_hp is high-pass filtered platform velocity
  *          - V_lp is low-pass GPS velocity
  *          
  *          Reference: WHOI Direct Covariance Flux System (DCFS) methodology
  ******************************************************************************
  */

#include "flux_calc.h"
#include <math.h>
#include <string.h>
#include <arm_math.h>

/* Private Defines -----------------------------------------------------------*/
#define PI_F            3.14159265358979323846f
#define TWO_PI_F        6.28318530717958647693f
#define DEG_TO_RAD_F    (PI_F / 180.0f)
#define RAD_TO_DEG_F    (180.0f / PI_F)
#define EPSILON         1e-10f      /* Small value for division safety */

/* Private Function Prototypes -----------------------------------------------*/
static void init_complementary_filter(ComplementaryFilter_t *cf, float cutoff_hz);
static void init_velocity_hp_filter(VelocityHPFilter_t *vf, float cutoff_hz);
static void update_complementary_filter(ComplementaryFilter_t *cf,
                                         const Vec3f_t *gyro,
                                         const Vec3f_t *accel,
                                         float compass_heading);
static Vec3f_t compute_platform_velocity_hp(VelocityHPFilter_t *vf,
                                             const Vec3f_t *accel_earth,
                                             const EulerAngles_t *euler);
static void update_running_stats(FluxStats_t *stats, 
                                  const Vec3f_t *wind_stream,
                                  float temp);
static Vec3f_t apply_streamwise_rotation(const StreamwiseRotation_t *rot,
                                          const Vec3f_t *wind);

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================*/

/**
 * @brief Initialize the flux calculation module
 */
void flux_init(FluxCalc_t *fc, float filter_cutoff_hz, float rho_air)
{
    if (fc == NULL) return;
    
    /* Clear entire structure */
    memset(fc, 0, sizeof(FluxCalc_t));
    
    /* Set configuration */
    fc->filter_cutoff_hz = (filter_cutoff_hz > 0.0f) ? filter_cutoff_hz : FLUX_FILTER_CUTOFF_HZ;
    fc->rho_air = (rho_air > 0.0f) ? rho_air : FLUX_RHO_AIR;
    
    /* Set default lever arm (IMU to anemometer offset) */
    fc->R.x = FLUX_R_X;
    fc->R.y = FLUX_R_Y;
    fc->R.z = FLUX_R_Z;
    
    /* Initialize filters */
    init_complementary_filter(&fc->attitude_filter, fc->filter_cutoff_hz);
    init_velocity_hp_filter(&fc->vel_filter, fc->filter_cutoff_hz);
    
    /* Initialize streamwise rotation to identity */
    fc->stream_rot.sin_alpha = 0.0f;
    fc->stream_rot.cos_alpha = 1.0f;
    fc->stream_rot.sin_beta = 0.0f;
    fc->stream_rot.cos_beta = 1.0f;
    fc->stream_rot.valid = false;
    
    fc->initialized = true;
}

/**
 * @brief Reset flux calculation state and statistics
 */
void flux_reset(FluxCalc_t *fc)
{
    if (fc == NULL) return;
    
    float cutoff = fc->filter_cutoff_hz;
    float rho = fc->rho_air;
    Vec3f_t R = fc->R;
    
    flux_init(fc, cutoff, rho);
    fc->R = R;
}

/**
 * @brief Reset only the statistics accumulators
 */
void flux_reset_stats(FluxCalc_t *fc)
{
    if (fc == NULL) return;
    
    memset(&fc->stats, 0, sizeof(FluxStats_t));
    fc->stream_rot.valid = false;
}

/**
 * @brief Set the filter cutoff frequency
 */
void flux_set_filter_cutoff(FluxCalc_t *fc, float cutoff_hz)
{
    if (fc == NULL || cutoff_hz <= 0.0f) return;
    
    fc->filter_cutoff_hz = cutoff_hz;
    
    /* Recalculate filter coefficients */
    float tau = 1.0f / (TWO_PI_F * cutoff_hz);
    float alpha = FLUX_DT / (tau + FLUX_DT);
    
    fc->attitude_filter.alpha = alpha;
    fc->attitude_filter.beta = 1.0f - alpha;
    
    fc->vel_filter.alpha = alpha;
}

/**
 * @brief Set the IMU to anemometer offset vector R
 */
void flux_set_lever_arm(FluxCalc_t *fc, float r_x, float r_y, float r_z)
{
    if (fc == NULL) return;
    
    fc->R.x = r_x;
    fc->R.y = r_y;
    fc->R.z = r_z;
}

/**
 * @brief Process one sample of sensor data
 *        
 * This implements the full motion correction algorithm:
 * 1. Estimate attitude using complementary filter
 * 2. Build rotation matrix T(phi,theta,psi)
 * 3. Compute angular velocity correction: Omega x R
 * 4. Rotate corrected wind to Earth frame
 * 5. Add platform velocities
 * 6. Apply streamwise rotation
 * 7. Update statistics for flux calculation
 */
void flux_process_sample(FluxCalc_t *fc,
                         const Vec3f_t *wind_obs,
                         const Vec3f_t *gyro,
                         const Vec3f_t *accel,
                         float compass_heading,
                         const Vec3f_t *gps_vel,
                         float temp)
{
    if (fc == NULL || wind_obs == NULL || gyro == NULL || accel == NULL) return;
    if (!fc->initialized) return;
    
    /* Step 1: Update attitude estimate using complementary filter
     * Combines high-frequency gyro integration with low-frequency 
     * accelerometer-derived tilt angles */
    update_complementary_filter(&fc->attitude_filter, gyro, accel, compass_heading);
    
    /* Extract attitude estimate */
    fc->euler.phi = fc->attitude_filter.phi_hp + fc->attitude_filter.phi_lp;
    fc->euler.theta = fc->attitude_filter.theta_hp + fc->attitude_filter.theta_lp;
    fc->euler.psi = fc->attitude_filter.psi_hp + fc->attitude_filter.psi_lp;
    
    /* Step 2: Build rotation matrix T(phi,theta,psi) from platform to Earth frame
     * 
     * T = | cos(psi)cos(theta)   -sin(psi)cos(phi) + cos(psi)sin(theta)sin(phi)   sin(psi)sin(phi) + cos(psi)sin(theta)cos(phi) |
     *     | sin(psi)cos(theta)    cos(psi)cos(phi) + sin(psi)sin(theta)sin(phi)   sin(psi)sin(theta)cos(phi) - cos(psi)sin(phi) |
     *     |     -sin(theta)                    cos(theta)sin(phi)                        cos(theta)cos(phi)                     |
     */
    RotMatrix_t T = flux_euler_to_matrix(&fc->euler);
    
    /* Step 3: Compute angular velocity correction: Omega x R
     * This removes the velocity component at the anemometer due to platform rotation */
    Vec3f_t omega_cross_R = flux_cross_product(gyro, &fc->R);
    
    /* Step 4: Add angular correction to observed wind
     * U_corrected = U_obs + Omega x R */
    Vec3f_t wind_angular_corrected = flux_vec_add(wind_obs, &omega_cross_R);
    
    /* Step 5: Rotate to Earth frame
     * U_earth = T(phi,theta,psi) * U_corrected */
    Vec3f_t wind_earth = flux_rotate_vector(&T, &wind_angular_corrected);
    
    /* Step 6: Compute and add high-pass filtered platform velocity
     * This requires rotating acceleration to Earth frame first to remove gravity */
    Vec3f_t accel_earth = flux_rotate_vector(&T, accel);
    fc->platform_vel_hp = compute_platform_velocity_hp(&fc->vel_filter, &accel_earth, &fc->euler);
    
    /* Step 7: Add platform velocities
     * U_true = U_earth + V_hp + V_lp */
    fc->wind_corrected = flux_vec_add(&wind_earth, &fc->platform_vel_hp);
    
    /* Add GPS velocity (low-pass platform velocity) if provided */
    if (gps_vel != NULL) {
        fc->platform_vel_lp = *gps_vel;
        fc->wind_corrected = flux_vec_add(&fc->wind_corrected, gps_vel);
    }
    
    /* Step 8: Apply streamwise rotation if available
     * Rotates wind into mean streamwise direction to get (U + u', v', w') */
    if (fc->stream_rot.valid) {
        fc->wind_streamwise = apply_streamwise_rotation(&fc->stream_rot, &fc->wind_corrected);
    } else {
        fc->wind_streamwise = fc->wind_corrected;
    }
    
    /* Step 9: Update running statistics for flux calculations */
    update_running_stats(&fc->stats, &fc->wind_streamwise, temp);
}

/**
 * @brief Get current motion-corrected wind
 */
Vec3f_t flux_get_corrected_wind(const FluxCalc_t *fc)
{
    if (fc == NULL) {
        Vec3f_t zero = {0.0f, 0.0f, 0.0f};
        return zero;
    }
    return fc->wind_corrected;
}

/**
 * @brief Get current attitude estimate
 */
EulerAngles_t flux_get_attitude(const FluxCalc_t *fc)
{
    if (fc == NULL) {
        EulerAngles_t zero = {0.0f, 0.0f, 0.0f};
        return zero;
    }
    return fc->euler;
}

/**
 * @brief Compute flux results from accumulated statistics
 *        
 * Calculates:
 * - Momentum flux: tau_x = -rho_a * cov(w,u), tau_y = -rho_a * cov(w,v)
 * - Buoyancy flux: H_f = rho_a * C_p * cov(w,T)
 * - Friction velocity: u* = (tau_x^2 + tau_y^2)^(1/4) / sqrt(rho_a)
 */
FluxResults_t flux_compute_results(const FluxCalc_t *fc)
{
    FluxResults_t results;
    memset(&results, 0, sizeof(FluxResults_t));
    
    if (fc == NULL || fc->stats.n < 2) {
        results.valid = false;
        return results;
    }
    
    const FluxStats_t *s = &fc->stats;
    float n = (float)s->n;
    float n_inv = 1.0f / n;
    
    /* Compute means */
    float mean_u = (float)(s->sum_u * n_inv);
    float mean_v = (float)(s->sum_v * n_inv);
    float mean_w = (float)(s->sum_w * n_inv);
    float mean_T = (float)(s->sum_T * n_inv);
    
    /* Compute covariances using E[xy] - E[x]E[y] formula
     * Note: sum_wu = sum(w*u) so cov(w,u) = (sum_wu)/n - (mean_w)*(mean_u) */
    float cov_wu = (float)(s->sum_wu * n_inv) - mean_w * mean_u;
    float cov_wv = (float)(s->sum_wv * n_inv) - mean_w * mean_v;
    float cov_wT = (float)(s->sum_wT * n_inv) - mean_w * mean_T;
    
    /* Compute variances for diagnostics */
    float var_u = (float)(s->sum_uu * n_inv) - mean_u * mean_u;
    float var_v = (float)(s->sum_vv * n_inv) - mean_v * mean_v;
    float var_w = (float)(s->sum_ww * n_inv) - mean_w * mean_w;
    float var_T = (float)(s->sum_TT * n_inv) - mean_T * mean_T;
    
    /* Momentum flux: tau = -rho_a * cov(w, u/v)
     * The negative sign accounts for downward momentum transport */
    results.tau_x = -fc->rho_air * cov_wu;
    results.tau_y = -fc->rho_air * cov_wv;
    
    /* Magnitude of stress vector */
    float tau_sq = results.tau_x * results.tau_x + results.tau_y * results.tau_y;
    results.tau_mag = sqrtf(tau_sq);
    
    /* Buoyancy/sensible heat flux: H_f = rho_a * C_p * cov(w, T) */
    results.H_f = fc->rho_air * FLUX_CP_AIR * cov_wT;
    
    /* Friction velocity: u* = (|tau|/rho_a)^0.5 = (cov_wu^2 + cov_wv^2)^0.25 */
    float u_star_sq = sqrtf(cov_wu * cov_wu + cov_wv * cov_wv);
    results.u_star = sqrtf(u_star_sq);
    
    /* Mean streamwise wind speed */
    results.U_mean = sqrtf(mean_u * mean_u + mean_v * mean_v);
    
    /* Standard deviations */
    results.sigma_u = sqrtf(fabsf(var_u));
    results.sigma_v = sqrtf(fabsf(var_v));
    results.sigma_w = sqrtf(fabsf(var_w));
    results.sigma_T = sqrtf(fabsf(var_T));
    
    /* Mean flow tilt angle (deviation from horizontal) */
    float horiz_speed = sqrtf(mean_u * mean_u + mean_v * mean_v);
    if (horiz_speed > EPSILON) {
        results.tilt_mean = flux_rad_to_deg(atanf(mean_w / horiz_speed));
    }
    
    results.n_samples = s->n;
    results.valid = true;
    
    return results;
}

/**
 * @brief Update streamwise rotation based on current mean wind
 *        
 * Computes rotation angles to align coordinate system with mean wind:
 * - alpha: horizontal rotation to align x-axis with mean horizontal wind
 * - beta: vertical tilt to remove mean vertical component
 *
 * After rotation: U = mean(U) + u', V = v', W = w'
 */
void flux_update_streamwise_rotation(FluxCalc_t *fc)
{
    if (fc == NULL || fc->stats.n < 100) return;
    
    float n_inv = 1.0f / (float)fc->stats.n;
    float mean_u = (float)(fc->stats.sum_u * n_inv);
    float mean_v = (float)(fc->stats.sum_v * n_inv);
    float mean_w = (float)(fc->stats.sum_w * n_inv);
    
    /* Horizontal rotation angle: alpha = atan2(mean_v, mean_u) */
    float alpha = atan2f(mean_v, mean_u);
    fc->stream_rot.sin_alpha = sinf(alpha);
    fc->stream_rot.cos_alpha = cosf(alpha);
    
    /* Horizontal wind magnitude after first rotation */
    float U_horiz = sqrtf(mean_u * mean_u + mean_v * mean_v);
    
    /* Vertical tilt angle: beta = atan2(mean_w, U_horiz) */
    float beta = atan2f(mean_w, U_horiz);
    fc->stream_rot.sin_beta = sinf(beta);
    fc->stream_rot.cos_beta = cosf(beta);
    
    fc->stream_rot.valid = true;
}

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================*/

/**
 * @brief Build rotation matrix from Euler angles
 *        
 * Implements the standard aerospace ZYX rotation sequence:
 * R = Rz(psi) * Ry(theta) * Rx(phi)
 *
 * This rotates vectors from platform frame to Earth frame.
 */
RotMatrix_t flux_euler_to_matrix(const EulerAngles_t *euler)
{
    RotMatrix_t R;
    
    float cp = cosf(euler->phi);    /* cos(roll) */
    float sp = sinf(euler->phi);    /* sin(roll) */
    float ct = cosf(euler->theta);  /* cos(pitch) */
    float st = sinf(euler->theta);  /* sin(pitch) */
    float cy = cosf(euler->psi);    /* cos(yaw) */
    float sy = sinf(euler->psi);    /* sin(yaw) */
    
    /* Row 0 */
    R.m[0][0] = cy * ct;
    R.m[0][1] = -sy * cp + cy * st * sp;
    R.m[0][2] = sy * sp + cy * st * cp;
    
    /* Row 1 */
    R.m[1][0] = sy * ct;
    R.m[1][1] = cy * cp + sy * st * sp;
    R.m[1][2] = sy * st * cp - cy * sp;
    
    /* Row 2 */
    R.m[2][0] = -st;
    R.m[2][1] = ct * sp;
    R.m[2][2] = ct * cp;
    
    return R;
}

/**
 * @brief Apply rotation matrix to a vector
 */
Vec3f_t flux_rotate_vector(const RotMatrix_t *R, const Vec3f_t *v)
{
    Vec3f_t result;
    
    result.x = R->m[0][0] * v->x + R->m[0][1] * v->y + R->m[0][2] * v->z;
    result.y = R->m[1][0] * v->x + R->m[1][1] * v->y + R->m[1][2] * v->z;
    result.z = R->m[2][0] * v->x + R->m[2][1] * v->y + R->m[2][2] * v->z;
    
    return result;
}

/**
 * @brief Compute cross product of two vectors
 */
Vec3f_t flux_cross_product(const Vec3f_t *a, const Vec3f_t *b)
{
    Vec3f_t result;
    
    result.x = a->y * b->z - a->z * b->y;
    result.y = a->z * b->x - a->x * b->z;
    result.z = a->x * b->y - a->y * b->x;
    
    return result;
}

/**
 * @brief Add two vectors
 */
Vec3f_t flux_vec_add(const Vec3f_t *a, const Vec3f_t *b)
{
    Vec3f_t result;
    result.x = a->x + b->x;
    result.y = a->y + b->y;
    result.z = a->z + b->z;
    return result;
}

/**
 * @brief Subtract two vectors
 */
Vec3f_t flux_vec_sub(const Vec3f_t *a, const Vec3f_t *b)
{
    Vec3f_t result;
    result.x = a->x - b->x;
    result.y = a->y - b->y;
    result.z = a->z - b->z;
    return result;
}

/**
 * @brief Scale a vector by a scalar
 */
Vec3f_t flux_vec_scale(const Vec3f_t *v, float s)
{
    Vec3f_t result;
    result.x = v->x * s;
    result.y = v->y * s;
    result.z = v->z * s;
    return result;
}

/**
 * @brief Compute magnitude of a vector
 */
float flux_vec_magnitude(const Vec3f_t *v)
{
    float mag_sq = v->x * v->x + v->y * v->y + v->z * v->z;
    float mag;
    arm_sqrt_f32(mag_sq, &mag);
    return mag;
}

/**
 * @brief Normalize a vector to unit length
 */
Vec3f_t flux_vec_normalize(const Vec3f_t *v)
{
    float mag = flux_vec_magnitude(v);
    if (mag < EPSILON) {
        Vec3f_t zero = {0.0f, 0.0f, 0.0f};
        return zero;
    }
    return flux_vec_scale(v, 1.0f / mag);
}

/**
 * @brief Convert degrees to radians
 */
float flux_deg_to_rad(float deg)
{
    return deg * DEG_TO_RAD_F;
}

/**
 * @brief Convert radians to degrees
 */
float flux_rad_to_deg(float rad)
{
    return rad * RAD_TO_DEG_F;
}

/**
 * @brief Get number of samples in current averaging period
 */
uint32_t flux_get_sample_count(const FluxCalc_t *fc)
{
    if (fc == NULL) return 0;
    return fc->stats.n;
}

/**
 * @brief Check if sufficient samples for valid flux calculation
 */
bool flux_has_sufficient_samples(const FluxCalc_t *fc, uint32_t min_samples)
{
    if (fc == NULL) return false;
    return fc->stats.n >= min_samples;
}

/* ============================================================================
 * PRIVATE FUNCTIONS
 * ============================================================================*/

/**
 * @brief Initialize complementary filter for attitude estimation
 */
static void init_complementary_filter(ComplementaryFilter_t *cf, float cutoff_hz)
{
    memset(cf, 0, sizeof(ComplementaryFilter_t));
    
    /* Calculate filter coefficients from cutoff frequency
     * Low-pass: α = dt / (τ + dt), where τ = 1/(2πf_c)
     * High-pass: β = 1 - α = τ / (τ + dt) */
    float tau = 1.0f / (TWO_PI_F * cutoff_hz);
    cf->alpha = FLUX_DT / (tau + FLUX_DT);
    cf->beta = 1.0f - cf->alpha;
    
    cf->initialized = false;
}

/**
 * @brief Initialize high-pass filter for velocity estimation
 */
static void init_velocity_hp_filter(VelocityHPFilter_t *vf, float cutoff_hz)
{
    memset(vf, 0, sizeof(VelocityHPFilter_t));
    
    float tau = 1.0f / (TWO_PI_F * cutoff_hz);
    vf->alpha = FLUX_DT / (tau + FLUX_DT);
    
    vf->initialized = false;
}

/**
 * @brief Update complementary filter with new sensor readings
 *        
 * Complementary filter combines:
 * - High-frequency component: integrated angular rate (gyro)
 * - Low-frequency component: tilt from accelerometer + compass heading
 *
 * For pitch and roll:
 *   θ_hp = HP[∫θ̇ dt]  (high-pass filtered integrated gyro)
 *   θ_lp = LP[asin(-ax/g)]  (low-pass filtered accelerometer tilt)
 *   θ = θ_hp + θ_lp
 *
 * For yaw:
 *   psi_hp = HP[integral{psi̇' dt}]
 *   psi_lp = LP[compass]
 *   psi = psi_hp + psi_lp
 */
static void update_complementary_filter(ComplementaryFilter_t *cf,
                                         const Vec3f_t *gyro,
                                         const Vec3f_t *accel,
                                         float compass_heading)
{
    /* First sample: initialize with accelerometer-derived angles */
    if (!cf->initialized) {
        /* Initial pitch from x-accelerometer: theta = asin(-ax/g) */
        float ax_norm = -accel->x / FLUX_GRAVITY;
        ax_norm = fmaxf(-1.0f, fminf(1.0f, ax_norm));  /* Clamp to [-1, 1] */
        cf->theta_lp = asinf(ax_norm);
        cf->phi_lp = 0.0f;
        
        /* Calculate roll if pitch is not ±90° */
        if (fabsf(cf->theta_lp) < (PI_F / 2.0f - 0.1f)) {
            float ay_norm = accel->y / (FLUX_GRAVITY * cosf(cf->theta_lp));
            ay_norm = fmaxf(-1.0f, fminf(1.0f, ay_norm));
            cf->phi_lp = asinf(ay_norm);
        }
        
        cf->psi_lp = compass_heading;
        
        cf->phi_prev = cf->phi_lp;
        cf->theta_prev = cf->theta_lp;
        cf->psi_prev = cf->psi_lp;
        
        cf->phi_hp = 0.0f;
        cf->theta_hp = 0.0f;
        cf->psi_hp = 0.0f;
        
        cf->initialized = true;
        return;
    }
    
    /* Integrate angular rates to get high-frequency angle changes */
    float d_phi = gyro->x * FLUX_DT;
    float d_theta = gyro->y * FLUX_DT;
    float d_psi = gyro->z * FLUX_DT;
    
    /* Update high-pass filtered angles
     * HP filter: y_hp[n] = β * (y_hp[n-1] + dx)
     * This removes DC drift from integration while keeping high-frequency content */
    cf->phi_hp = cf->beta * (cf->phi_hp + d_phi);
    cf->theta_hp = cf->beta * (cf->theta_hp + d_theta);
    cf->psi_hp = cf->beta * (cf->psi_hp + d_psi);
    
    /* Compute low-frequency tilt angles from accelerometer
     * theta_accel = asin(-ax/g)
     * phi_accel = asin(ay/(g*cos(theta))) */
    float ax_norm = -accel->x / FLUX_GRAVITY;
    ax_norm = fmaxf(-1.0f, fminf(1.0f, ax_norm));
    float theta_accel = asinf(ax_norm);
    
    float phi_accel = 0.0f;
    float ct = cosf(theta_accel);
    if (fabsf(ct) > 0.1f) {
        float ay_norm = accel->y / (FLUX_GRAVITY * ct);
        ay_norm = fmaxf(-1.0f, fminf(1.0f, ay_norm));
        phi_accel = asinf(ay_norm);
    }
    
    /* Update low-pass filtered angles
     * LP filter: y_lp[n] = α * x[n] + (1-α) * y_lp[n-1] */
    cf->phi_lp = cf->alpha * phi_accel + (1.0f - cf->alpha) * cf->phi_lp;
    cf->theta_lp = cf->alpha * theta_accel + (1.0f - cf->alpha) * cf->theta_lp;
    cf->psi_lp = cf->alpha * compass_heading + (1.0f - cf->alpha) * cf->psi_lp;
    
    /* Handle yaw wraparound */
    if (compass_heading - cf->psi_lp > PI_F) {
        cf->psi_lp += TWO_PI_F;
    } else if (compass_heading - cf->psi_lp < -PI_F) {
        cf->psi_lp -= TWO_PI_F;
    }
}

/**
 * @brief Compute high-pass filtered platform velocity
 *        
 * 1. Remove gravity from accelerometer readings (using attitude estimate)
 * 2. Integrate to get velocity
 * 3. High-pass filter to remove low-frequency drift
 */
static Vec3f_t compute_platform_velocity_hp(VelocityHPFilter_t *vf,
                                             const Vec3f_t *accel_earth,
                                             const EulerAngles_t *euler)
{
    /* Remove gravity component from Earth-frame acceleration
     * In Earth frame, gravity is [0, 0, g] (pointing down) */
    Vec3f_t accel_no_grav;
    accel_no_grav.x = accel_earth->x;
    accel_no_grav.y = accel_earth->y;
    accel_no_grav.z = accel_earth->z - FLUX_GRAVITY;
    
    if (!vf->initialized) {
        vf->acc_prev = accel_no_grav;
        vf->vel_raw.x = 0.0f;
        vf->vel_raw.y = 0.0f;
        vf->vel_raw.z = 0.0f;
        vf->vel_hp = vf->vel_raw;
        vf->initialized = true;
        return vf->vel_hp;
    }
    
    /* Trapezoidal integration: v += (a + a_prev)/2 * dt */
    vf->vel_raw.x += 0.5f * (accel_no_grav.x + vf->acc_prev.x) * FLUX_DT;
    vf->vel_raw.y += 0.5f * (accel_no_grav.y + vf->acc_prev.y) * FLUX_DT;
    vf->vel_raw.z += 0.5f * (accel_no_grav.z + vf->acc_prev.z) * FLUX_DT;
    
    /* High-pass filter the velocity
     * HP: y[n] = (1-α) * (y[n-1] + x[n] - x_prev[n-1])
     * But since we're filtering integrated velocity:
     * y_hp = (1-α) * y_hp + (1-α) * dv
     * where dv is the change in raw velocity this step */
    float beta = 1.0f - vf->alpha;
    
    /* Alternative: simple HP on accumulated velocity */
    vf->vel_hp.x = beta * vf->vel_hp.x + vf->alpha * 0.0f + beta * 0.5f * (accel_no_grav.x + vf->acc_prev.x) * FLUX_DT;
    vf->vel_hp.y = beta * vf->vel_hp.y + vf->alpha * 0.0f + beta * 0.5f * (accel_no_grav.y + vf->acc_prev.y) * FLUX_DT;
    vf->vel_hp.z = beta * vf->vel_hp.z + vf->alpha * 0.0f + beta * 0.5f * (accel_no_grav.z + vf->acc_prev.z) * FLUX_DT;
    
    vf->acc_prev = accel_no_grav;
    
    return vf->vel_hp;
}

/**
 * @brief Update running statistics for flux calculation
 *        
 * Uses Welford's online algorithm concept but accumulates sums directly
 * for later covariance computation.
 */
static void update_running_stats(FluxStats_t *stats, 
                                  const Vec3f_t *wind_stream,
                                  float temp)
{
    float u = wind_stream->x;
    float v = wind_stream->y;
    float w = wind_stream->z;
    float T = temp;
    
    /* Update means using incremental formula */
    stats->n++;
    float n = (float)stats->n;
    float n_inv = 1.0f / n;
    
    /* Update running means */
    stats->mean_u += (u - stats->mean_u) * n_inv;
    stats->mean_v += (v - stats->mean_v) * n_inv;
    stats->mean_w += (w - stats->mean_w) * n_inv;
    stats->mean_T += (T - stats->mean_T) * n_inv;
    
    /* Accumulate sums for final covariance calculation */
    stats->sum_u += u;
    stats->sum_v += v;
    stats->sum_w += w;
    stats->sum_T += T;
    
    /* Accumulate products for covariances */
    stats->sum_wu += (double)w * (double)u;
    stats->sum_wv += (double)w * (double)v;
    stats->sum_wT += (double)w * (double)T;
    
    /* Accumulate squares for variances */
    stats->sum_uu += (double)u * (double)u;
    stats->sum_vv += (double)v * (double)v;
    stats->sum_ww += (double)w * (double)w;
    stats->sum_TT += (double)T * (double)T;
}

/**
 * @brief Apply double rotation to align wind with streamwise direction
 *        
 * First rotation: horizontal (yaw) to align mean V = 0
 * Second rotation: vertical (pitch) to align mean W = 0
 *
 * Result: U = Ū + u', V = v', W = w'
 */
static Vec3f_t apply_streamwise_rotation(const StreamwiseRotation_t *rot,
                                          const Vec3f_t *wind)
{
    Vec3f_t result;
    
    /* First rotation: horizontal (align with mean wind direction)
     * u1 = u*cos(α) + v*sin(α)
     * v1 = -u*sin(α) + v*cos(α)
     * w1 = w */
    float u1 = wind->x * rot->cos_alpha + wind->y * rot->sin_alpha;
    float v1 = -wind->x * rot->sin_alpha + wind->y * rot->cos_alpha;
    float w1 = wind->z;
    
    /* Second rotation: vertical (remove mean vertical component)
     * u2 = u1*cos(β) + w1*sin(β)
     * v2 = v1
     * w2 = -u1*sin(β) + w1*cos(β) */
    result.x = u1 * rot->cos_beta + w1 * rot->sin_beta;
    result.y = v1;
    result.z = -u1 * rot->sin_beta + w1 * rot->cos_beta;
    
    return result;
}
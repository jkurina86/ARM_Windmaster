/**
  ******************************************************************************
  * @file    flux_calc.h
  * @brief   Flux calculation functions for eddy covariance measurements
  * @note    Implements motion correction for moving platform flux measurements
  *          
  *          Key equations implemented:
  *          - Momentum flux: τ_x = -ρ_a * w'u', τ_y = -ρ_a * w'v'
  *          - Buoyancy flux: H_f = ρ_a * C_p * w'T'
  *          - Motion-corrected wind: U_true = T(φ,θ,ψ)[U_obs + Ω × R] + V_hp + V_lp
  *          
  *          Reference: WHOI Direct Covariance Flux System (DCFS) methodology
  ******************************************************************************
  */

#ifndef INC_FLUX_CALC_H_
#define INC_FLUX_CALC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported Constants --------------------------------------------------------*/

/* Physical constants */
#define FLUX_GRAVITY            9.80665f    /* Gravitational acceleration [m/s²] */
#define FLUX_RHO_AIR            1.225f      /* Air density at sea level [kg/m³] */
#define FLUX_CP_AIR             1005.0f     /* Specific heat capacity of air [J/(kg·K)] */

/* System geometry - IMU to anemometer offset vector R [m] */
#define FLUX_R_X                0.0f        /* X offset: IMU to anemometer [m] */
#define FLUX_R_Y                0.0f        /* Y offset: IMU to anemometer [m] */
#define FLUX_R_Z                0.8f        /* Z offset: IMU 0.8m below anemometer [m] */

/* Filter configuration */
#define FLUX_SAMPLE_RATE_HZ     20.0f       /* Sample rate [Hz] */
#define FLUX_DT                 (1.0f / FLUX_SAMPLE_RATE_HZ)  /* Sample period [s] */

/* Default complementary filter cutoff frequency [Hz] 
 * Platform and environment dependent - adjust based on variance spectra analysis */
#define FLUX_FILTER_CUTOFF_HZ   0.1f        /* Default cutoff frequency [Hz] */

/* Averaging window for flux calculations */
#define FLUX_AVERAGING_PERIOD_S 600.0f      /* 10 minute averaging period [s] */
#define FLUX_BUFFER_SIZE        ((uint32_t)(FLUX_AVERAGING_PERIOD_S * FLUX_SAMPLE_RATE_HZ))

/* Exported Types ------------------------------------------------------------*/

/**
 * @brief 3D vector structure for wind, velocity, angular rate, etc.
 */
typedef struct {
    float x;    /* X component (or U for wind) */
    float y;    /* Y component (or V for wind) */
    float z;    /* Z component (or W for wind) */
} Vec3f_t;

/**
 * @brief Euler angles structure
 */
typedef struct {
    float phi;      /* Roll angle [rad] - rotation about X-axis, + port up */
    float theta;    /* Pitch angle [rad] - rotation about Y-axis, + bow down */
    float psi;      /* Yaw/heading angle [rad] - rotation about Z-axis, + CCW */
} EulerAngles_t;

/**
 * @brief 3x3 rotation matrix structure (row-major)
 */
typedef struct {
    float m[3][3];  /* Matrix elements [row][col] */
} RotMatrix_t;

/**
 * @brief Complementary filter state for attitude estimation
 */
typedef struct {
    /* Integrated gyro angles (high-pass component) */
    float phi_hp;       /* High-pass roll [rad] */
    float theta_hp;     /* High-pass pitch [rad] */
    float psi_hp;       /* High-pass yaw [rad] */
    
    /* Low-pass filtered accelerometer-derived angles */
    float phi_lp;       /* Low-pass roll [rad] */
    float theta_lp;     /* Low-pass pitch [rad] */
    float psi_lp;       /* Low-pass yaw (from compass) [rad] */
    
    /* Filter coefficients (calculated from cutoff frequency) */
    float alpha;        /* Low-pass filter coefficient α = dt/(τ + dt) */
    float beta;         /* High-pass coefficient β = 1 - α */
    
    /* Previous values for filtering */
    float phi_prev;
    float theta_prev;
    float psi_prev;
    
    bool initialized;   /* True after first sample processed */
} ComplementaryFilter_t;

/**
 * @brief High-pass filter state for velocity integration
 */
typedef struct {
    Vec3f_t vel_hp;         /* High-pass filtered velocity [m/s] */
    Vec3f_t acc_prev;       /* Previous acceleration [m/s²] */
    Vec3f_t vel_raw;        /* Integrated velocity before HP filter */
    float alpha;            /* Filter coefficient */
    bool initialized;
} VelocityHPFilter_t;

/**
 * @brief Running statistics for flux calculations
 */
typedef struct {
    /* Accumulators for means */
    double sum_u;       /* Sum of U wind component */
    double sum_v;       /* Sum of V wind component */
    double sum_w;       /* Sum of W wind component */
    double sum_T;       /* Sum of virtual temperature */
    
    /* Accumulators for covariances */
    double sum_wu;      /* Sum of w'u' product */
    double sum_wv;      /* Sum of w'v' product */
    double sum_wT;      /* Sum of w'T' product */
    
    /* Accumulators for variances (optional diagnostics) */
    double sum_uu;      /* Sum of u'u' product */
    double sum_vv;      /* Sum of v'v' product */
    double sum_ww;      /* Sum of w'w' product */
    double sum_TT;      /* Sum of T'T' product */
    
    /* Sample count */
    uint32_t n;
    
    /* Running means (updated each sample) */
    float mean_u;
    float mean_v;
    float mean_w;
    float mean_T;
} FluxStats_t;

/**
 * @brief Streamwise rotation state
 */
typedef struct {
    float sin_alpha;    /* sin of horizontal rotation angle */
    float cos_alpha;    /* cos of horizontal rotation angle */
    float sin_beta;     /* sin of vertical tilt angle */
    float cos_beta;     /* cos of vertical tilt angle */
    bool valid;         /* True if rotation angles have been computed */
} StreamwiseRotation_t;

/**
 * @brief Complete flux calculation state
 */
typedef struct {
    /* Filter states */
    ComplementaryFilter_t attitude_filter;
    VelocityHPFilter_t vel_filter;
    
    /* Current estimates */
    EulerAngles_t euler;            /* Current attitude estimate */
    Vec3f_t wind_corrected;         /* Motion-corrected wind in Earth frame */
    Vec3f_t wind_streamwise;        /* Wind in streamwise coordinates */
    Vec3f_t platform_vel_hp;        /* High-pass platform velocity */
    Vec3f_t platform_vel_lp;        /* Low-pass platform velocity (GPS) */
    
    /* Streamwise rotation */
    StreamwiseRotation_t stream_rot;
    
    /* Statistics accumulators */
    FluxStats_t stats;
    
    /* Configuration */
    float filter_cutoff_hz;         /* Complementary filter cutoff [Hz] */
    float rho_air;                  /* Air density [kg/m³] */
    
    /* Position vector R (IMU to anemometer) */
    Vec3f_t R;
    
    /* Initialization flag */
    bool initialized;
} FluxCalc_t;

/**
 * @brief Flux calculation results
 */
typedef struct {
    /* Momentum fluxes [N/m² or Pa] */
    float tau_x;        /* X-component (u-direction) momentum flux */
    float tau_y;        /* Y-component (v-direction) momentum flux */
    float tau_mag;      /* Magnitude of momentum flux vector */
    
    /* Sensible heat flux [W/m²] */
    float H_f;          /* Buoyancy/sensible heat flux */
    
    /* Friction velocity [m/s] */
    float u_star;       /* u* = sqrt(tau/rho) */
    
    /* Mean wind speed [m/s] */
    float U_mean;       /* Mean streamwise wind speed */
    
    /* Turbulent statistics */
    float sigma_u;      /* Std dev of u' */
    float sigma_v;      /* Std dev of v' */
    float sigma_w;      /* Std dev of w' */
    float sigma_T;      /* Std dev of T' */
    
    /* Quality indicators */
    uint32_t n_samples; /* Number of samples in average */
    float tilt_mean;    /* Mean flow tilt angle [deg] */
    
    bool valid;         /* True if results are valid */
} FluxResults_t;

/* Exported Function Prototypes ----------------------------------------------*/

/**
 * @brief Initialize the flux calculation module
 * @param fc Pointer to FluxCalc_t state structure
 * @param filter_cutoff_hz Complementary filter cutoff frequency [Hz]
 * @param rho_air Air density [kg/m³], use 0 for default (1.225)
 */
void flux_init(FluxCalc_t *fc, float filter_cutoff_hz, float rho_air);

/**
 * @brief Reset flux calculation state and statistics
 * @param fc Pointer to FluxCalc_t state structure
 */
void flux_reset(FluxCalc_t *fc);

/**
 * @brief Reset only the statistics accumulators (start new averaging period)
 * @param fc Pointer to FluxCalc_t state structure
 */
void flux_reset_stats(FluxCalc_t *fc);

/**
 * @brief Set the filter cutoff frequency
 * @param fc Pointer to FluxCalc_t state structure
 * @param cutoff_hz New cutoff frequency [Hz]
 */
void flux_set_filter_cutoff(FluxCalc_t *fc, float cutoff_hz);

/**
 * @brief Set the IMU to anemometer offset vector R
 * @param fc Pointer to FluxCalc_t state structure
 * @param r_x X offset [m]
 * @param r_y Y offset [m]
 * @param r_z Z offset [m]
 */
void flux_set_lever_arm(FluxCalc_t *fc, float r_x, float r_y, float r_z);

/**
 * @brief Process one sample of sensor data
 * @param fc Pointer to FluxCalc_t state structure
 * @param wind_obs Observed wind vector from anemometer [m/s] in platform frame
 * @param gyro Angular velocity from IMU [rad/s] in platform frame
 * @param accel Acceleration from IMU [m/s²] in platform frame
 * @param compass_heading Compass heading (yaw) [rad], positive CCW from North
 * @param gps_vel GPS-derived velocity [m/s] in Earth frame (NED or ENU)
 * @param temp Virtual temperature [°C or K]
 */
void flux_process_sample(FluxCalc_t *fc,
                         const Vec3f_t *wind_obs,
                         const Vec3f_t *gyro,
                         const Vec3f_t *accel,
                         float compass_heading,
                         const Vec3f_t *gps_vel,
                         float temp);

/**
 * @brief Get current motion-corrected wind in Earth coordinates
 * @param fc Pointer to FluxCalc_t state structure
 * @return Motion-corrected wind vector [m/s]
 */
Vec3f_t flux_get_corrected_wind(const FluxCalc_t *fc);

/**
 * @brief Get current attitude estimate
 * @param fc Pointer to FluxCalc_t state structure
 * @return Current Euler angles [rad]
 */
EulerAngles_t flux_get_attitude(const FluxCalc_t *fc);

/**
 * @brief Compute flux results from accumulated statistics
 * @param fc Pointer to FluxCalc_t state structure
 * @return Flux calculation results
 */
FluxResults_t flux_compute_results(const FluxCalc_t *fc);

/**
 * @brief Update streamwise rotation based on current mean wind
 * @param fc Pointer to FluxCalc_t state structure
 * @note Call this periodically (e.g., every averaging period) to update rotation
 */
void flux_update_streamwise_rotation(FluxCalc_t *fc);

/* Utility Functions ---------------------------------------------------------*/

/**
 * @brief Build rotation matrix from Euler angles
 * @param euler Pointer to Euler angles structure
 * @return 3x3 rotation matrix (platform → Earth)
 */
RotMatrix_t flux_euler_to_matrix(const EulerAngles_t *euler);

/**
 * @brief Apply rotation matrix to a vector
 * @param R Pointer to rotation matrix
 * @param v Pointer to input vector
 * @return Rotated vector
 */
Vec3f_t flux_rotate_vector(const RotMatrix_t *R, const Vec3f_t *v);

/**
 * @brief Compute cross product of two vectors
 * @param a Pointer to first vector
 * @param b Pointer to second vector
 * @return Cross product a × b
 */
Vec3f_t flux_cross_product(const Vec3f_t *a, const Vec3f_t *b);

/**
 * @brief Add two vectors
 * @param a Pointer to first vector
 * @param b Pointer to second vector
 * @return Sum a + b
 */
Vec3f_t flux_vec_add(const Vec3f_t *a, const Vec3f_t *b);

/**
 * @brief Subtract two vectors
 * @param a Pointer to first vector
 * @param b Pointer to second vector
 * @return Difference a - b
 */
Vec3f_t flux_vec_sub(const Vec3f_t *a, const Vec3f_t *b);

/**
 * @brief Scale a vector by a scalar
 * @param v Pointer to vector
 * @param s Scalar value
 * @return Scaled vector s * v
 */
Vec3f_t flux_vec_scale(const Vec3f_t *v, float s);

/**
 * @brief Compute magnitude of a vector
 * @param v Pointer to vector
 * @return Magnitude |v|
 */
float flux_vec_magnitude(const Vec3f_t *v);

/**
 * @brief Normalize a vector to unit length
 * @param v Pointer to vector
 * @return Unit vector v/|v|, or zero vector if |v| ≈ 0
 */
Vec3f_t flux_vec_normalize(const Vec3f_t *v);

/**
 * @brief Convert degrees to radians
 * @param deg Angle in degrees
 * @return Angle in radians
 */
float flux_deg_to_rad(float deg);

/**
 * @brief Convert radians to degrees
 * @param rad Angle in radians
 * @return Angle in degrees
 */
float flux_rad_to_deg(float rad);

/**
 * @brief Get number of samples in current averaging period
 * @param fc Pointer to FluxCalc_t state structure
 * @return Number of samples accumulated
 */
uint32_t flux_get_sample_count(const FluxCalc_t *fc);

/**
 * @brief Check if sufficient samples for valid flux calculation
 * @param fc Pointer to FluxCalc_t state structure
 * @param min_samples Minimum required samples
 * @return True if sample count >= min_samples
 */
bool flux_has_sufficient_samples(const FluxCalc_t *fc, uint32_t min_samples);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLUX_CALC_H_ */
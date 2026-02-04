// Runtime-tunable radio parameters
//
// Provides live parameter tuning via ESB radio to eliminate the
// build-flash-wait-test cycle during PID tuning.
//
// Thread safety: ARM Cortex-M4 guarantees atomic 32-bit aligned reads/writes.
// Actors read current values each control loop iteration (no caching).
// Ground station writes are immediately visible to all actors.

#ifndef TUNABLE_PARAMS_H
#define TUNABLE_PARAMS_H

#include "hive_runtime.h"

// Parameter count (must match enum)
#define TUNABLE_PARAM_COUNT 45

// Parameter identifiers
// Grouped by subsystem for easier navigation
typedef enum {
    // Rate PID (0-6)
    PARAM_RATE_KP = 0,
    PARAM_RATE_KI = 1,
    PARAM_RATE_KD = 2,
    PARAM_RATE_IMAX = 3,
    PARAM_RATE_OMAX_ROLL = 4,
    PARAM_RATE_OMAX_PITCH = 5,
    PARAM_RATE_OMAX_YAW = 6,

    // Attitude PID (7-11)
    PARAM_ATT_KP = 7,
    PARAM_ATT_KI = 8,
    PARAM_ATT_KD = 9,
    PARAM_ATT_IMAX = 10,
    PARAM_ATT_OMAX = 11,

    // Altitude PID (12-18)
    PARAM_ALT_KP = 12,
    PARAM_ALT_KI = 13,
    PARAM_ALT_KD = 14,
    PARAM_ALT_IMAX = 15,
    PARAM_ALT_OMAX = 16,
    PARAM_HOVER_THRUST = 17,
    PARAM_VVEL_DAMPING = 18,

    // Emergency limits (19-20)
    PARAM_EMERGENCY_TILT_LIMIT = 19,
    PARAM_EMERGENCY_ALT_MAX = 20,

    // Landing (21-22)
    PARAM_LANDING_DESCENT_RATE = 21,
    PARAM_LANDING_VELOCITY_GAIN = 22,

    // Position control (23-25)
    PARAM_POS_KP = 23,
    PARAM_POS_KD = 24,
    PARAM_MAX_TILT_ANGLE = 25,

    // Complementary filter (26-30)
    PARAM_CF_ALPHA = 26,
    PARAM_CF_MAG_ALPHA = 27,
    PARAM_CF_USE_MAG = 28,
    PARAM_CF_ACCEL_THRESH_LO = 29,
    PARAM_CF_ACCEL_THRESH_HI = 30,

    // Waypoint navigation (31-35)
    PARAM_WP_TOLERANCE_XY = 31,
    PARAM_WP_TOLERANCE_Z = 32,
    PARAM_WP_TOLERANCE_YAW = 33,
    PARAM_WP_TOLERANCE_VEL = 34,
    PARAM_WP_HOVER_TIME_S = 35,

    // Flight manager (36)
    PARAM_THRUST_RAMP_MS = 36,

    // Altitude Kalman filter (37-43)
    PARAM_KF_Q_ALTITUDE = 37,
    PARAM_KF_Q_VELOCITY = 38,
    PARAM_KF_Q_BIAS = 39,
    PARAM_KF_R_ALTITUDE = 40,
    PARAM_KF_P0_ALTITUDE = 41,
    PARAM_KF_P0_VELOCITY = 42,
    PARAM_KF_P0_BIAS = 43,

    // Horizontal velocity filter (44)
    PARAM_HVEL_FILTER_ALPHA = 44,
} tunable_param_id_t;

// All tunable parameters in a single struct
// Organized to match the enum for easy iteration
typedef struct {
    // Rate PID
    float rate_kp;
    float rate_ki;
    float rate_kd;
    float rate_imax;
    float rate_omax_roll;
    float rate_omax_pitch;
    float rate_omax_yaw;

    // Attitude PID
    float att_kp;
    float att_ki;
    float att_kd;
    float att_imax;
    float att_omax;

    // Altitude PID
    float alt_kp;
    float alt_ki;
    float alt_kd;
    float alt_imax;
    float alt_omax;
    float hover_thrust;
    float vvel_damping;

    // Emergency limits
    float emergency_tilt_limit;
    float emergency_alt_max;

    // Landing
    float landing_descent_rate;
    float landing_velocity_gain;

    // Position control
    float pos_kp;
    float pos_kd;
    float max_tilt_angle;

    // Complementary filter
    float cf_alpha;
    float cf_mag_alpha;
    float cf_use_mag; // 0.0 = false, 1.0 = true
    float cf_accel_thresh_lo;
    float cf_accel_thresh_hi;

    // Waypoint navigation
    float wp_tolerance_xy;
    float wp_tolerance_z;
    float wp_tolerance_yaw;
    float wp_tolerance_vel;
    float wp_hover_time_s;

    // Flight manager
    float thrust_ramp_ms;

    // Altitude Kalman filter
    float kf_q_altitude;
    float kf_q_velocity;
    float kf_q_bias;
    float kf_r_altitude;
    float kf_p0_altitude;
    float kf_p0_velocity;
    float kf_p0_bias;

    // Horizontal velocity filter
    float hvel_filter_alpha;
} tunable_params_t;

// Initialize parameters with platform defaults from HAL_* defines
void tunable_params_init(tunable_params_t *params);

// Set a parameter by ID with validation
// Returns HIVE_OK on success, HIVE_ERR_INVALID if ID or value out of range
hive_status_t tunable_params_set(tunable_params_t *params,
                                 tunable_param_id_t id, float value);

// Get a parameter by ID
// Returns 0.0f if ID is invalid
float tunable_params_get(const tunable_params_t *params, tunable_param_id_t id);

// Get parameter name string for debugging/ground station
// Returns "unknown" if ID is invalid
const char *tunable_params_name(tunable_param_id_t id);

#endif // TUNABLE_PARAMS_H

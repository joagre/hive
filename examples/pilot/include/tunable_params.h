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
#define TUNABLE_PARAM_COUNT 60

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

    // Altitude PID (12-17)
    PARAM_ALT_KP = 12,
    PARAM_ALT_KI = 13,
    PARAM_ALT_KD = 14,
    PARAM_ALT_IMAX = 15,
    PARAM_ALT_OMAX = 16,
    PARAM_VVEL_DAMPING = 17,

    // Emergency limits (18-19)
    PARAM_EMERGENCY_TILT_LIMIT = 18,
    PARAM_EMERGENCY_ALT_MAX = 19,

    // Landing (20-21)
    PARAM_LANDING_DESCENT_RATE = 20,
    PARAM_LANDING_VELOCITY_GAIN = 21,

    // Position control (22-24)
    PARAM_POS_KP = 22,
    PARAM_POS_KD = 23,
    PARAM_MAX_TILT_ANGLE = 24,

    // Complementary filter (25-29)
    PARAM_CF_ALPHA = 25,
    PARAM_CF_MAG_ALPHA = 26,
    PARAM_CF_USE_MAG = 27,
    PARAM_CF_ACCEL_THRESH_LO = 28,
    PARAM_CF_ACCEL_THRESH_HI = 29,

    // Waypoint navigation (30-34)
    PARAM_WP_TOLERANCE_XY = 30,
    PARAM_WP_TOLERANCE_Z = 31,
    PARAM_WP_TOLERANCE_YAW = 32,
    PARAM_WP_TOLERANCE_VEL = 33,
    PARAM_WP_HOVER_TIME_S = 34,

    // Altitude Kalman filter (35-41)
    PARAM_KF_Q_ALTITUDE = 35,
    PARAM_KF_Q_VELOCITY = 36,
    PARAM_KF_Q_BIAS = 37,
    PARAM_KF_R_ALTITUDE = 38,
    PARAM_KF_P0_ALTITUDE = 39,
    PARAM_KF_P0_VELOCITY = 40,
    PARAM_KF_P0_BIAS = 41,

    // Horizontal velocity filter (42) - kept for backwards compat
    PARAM_HVEL_FILTER_ALPHA = 42,

    // Flight manager lifecycle (43-44)
    PARAM_ARMED_COUNTDOWN_S = 43,
    PARAM_AUTO_GO_DELAY_S = 44,

    // Yaw rate PID - separate from roll/pitch (45-47)
    // Yaw needs higher gains: no aerodynamic restoring force,
    // constant motor torque imbalance requires integral action.
    PARAM_RATE_YAW_KP = 45,
    PARAM_RATE_YAW_KI = 46,
    PARAM_RATE_YAW_KD = 47,

    // Horizontal Kalman filter (48-54)
    PARAM_HKF_Q_POSITION = 48,
    PARAM_HKF_Q_VELOCITY = 49,
    PARAM_HKF_Q_BIAS = 50,
    PARAM_HKF_R_VELOCITY = 51,
    PARAM_HKF_P0_POSITION = 52,
    PARAM_HKF_P0_VELOCITY = 53,
    PARAM_HKF_P0_BIAS = 54,

    // Liftoff and climb (55-57)
    PARAM_LIFTOFF_CLIMB_RATE = 55,
    PARAM_LIFTOFF_RAMP_RATE = 56,
    PARAM_LIFTOFF_THRUST_CORRECTION = 57,

    // Innovation gating (58-59)
    PARAM_KF_MAX_INNOVATION = 58,
    PARAM_HKF_MAX_INNOVATION = 59,
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

    // Flight manager lifecycle
    float armed_countdown_s;
    float auto_go_delay_s;

    // Yaw rate PID (separate from roll/pitch)
    float rate_yaw_kp;
    float rate_yaw_ki;
    float rate_yaw_kd;

    // Horizontal Kalman filter
    float hkf_q_position;
    float hkf_q_velocity;
    float hkf_q_bias;
    float hkf_r_velocity;
    float hkf_p0_position;
    float hkf_p0_velocity;
    float hkf_p0_bias;

    // Liftoff and climb
    float liftoff_climb_rate;
    float liftoff_ramp_rate;
    float liftoff_thrust_correction;

    // Innovation gating
    float kf_max_innovation;
    float hkf_max_innovation;
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

// Runtime-tunable radio parameters implementation
//
// All validation ranges chosen to be safe for flight while allowing
// reasonable tuning. Values outside these ranges indicate ground station
// bugs or radio corruption.

#include "tunable_params.h"
#include "config.h"
#include "hal_config.h"
#include "hive_log.h"

// Parameter metadata for validation
typedef struct {
    const char *name;
    float min;
    float max;
} param_meta_t;

// Validation ranges - conservative for safety
static const param_meta_t param_meta[TUNABLE_PARAM_COUNT] = {
    // Rate PID (0-6)
    [PARAM_RATE_KP] = {.name = "rate_kp", .min = 0.0f, .max = 0.5f},
    [PARAM_RATE_KI] = {.name = "rate_ki", .min = 0.0f, .max = 0.1f},
    [PARAM_RATE_KD] = {.name = "rate_kd", .min = 0.0f, .max = 0.05f},
    [PARAM_RATE_IMAX] = {.name = "rate_imax", .min = 0.0f, .max = 1.0f},
    [PARAM_RATE_OMAX_ROLL] = {.name = "rate_omax_roll",
                              .min = 0.0f,
                              .max = 0.5f},
    [PARAM_RATE_OMAX_PITCH] = {.name = "rate_omax_pitch",
                               .min = 0.0f,
                               .max = 0.5f},
    [PARAM_RATE_OMAX_YAW] = {.name = "rate_omax_yaw", .min = 0.0f, .max = 0.5f},

    // Attitude PID (7-11)
    [PARAM_ATT_KP] = {.name = "att_kp", .min = 0.0f, .max = 10.0f},
    [PARAM_ATT_KI] = {.name = "att_ki", .min = 0.0f, .max = 1.0f},
    [PARAM_ATT_KD] = {.name = "att_kd", .min = 0.0f, .max = 1.0f},
    [PARAM_ATT_IMAX] = {.name = "att_imax", .min = 0.0f, .max = 2.0f},
    [PARAM_ATT_OMAX] = {.name = "att_omax", .min = 0.0f, .max = 5.0f},

    // Altitude PID (12-17)
    [PARAM_ALT_KP] = {.name = "alt_kp", .min = 0.0f, .max = 1.0f},
    [PARAM_ALT_KI] = {.name = "alt_ki", .min = 0.0f, .max = 0.5f},
    [PARAM_ALT_KD] = {.name = "alt_kd", .min = 0.0f, .max = 0.5f},
    [PARAM_ALT_IMAX] = {.name = "alt_imax", .min = 0.0f, .max = 1.0f},
    [PARAM_ALT_OMAX] = {.name = "alt_omax", .min = 0.0f, .max = 0.5f},
    [PARAM_VVEL_DAMPING] = {.name = "vvel_damping", .min = 0.0f, .max = 1.0f},

    // Emergency limits (18-19)
    [PARAM_EMERGENCY_TILT_LIMIT] = {.name = "emergency_tilt_limit",
                                    .min = 0.17f,
                                    .max = 1.57f}, // 10-90 deg
    [PARAM_EMERGENCY_ALT_MAX] = {.name = "emergency_alt_max",
                                 .min = 0.5f,
                                 .max = 10.0f},

    // Landing (20-21)
    [PARAM_LANDING_DESCENT_RATE] = {.name = "landing_descent_rate",
                                    .min = -1.0f,
                                    .max = 0.0f}, // negative = down
    [PARAM_LANDING_VELOCITY_GAIN] = {.name = "landing_velocity_gain",
                                     .min = 0.0f,
                                     .max = 2.0f},

    // Position control (22-24)
    [PARAM_POS_KP] = {.name = "pos_kp", .min = 0.0f, .max = 1.0f},
    [PARAM_POS_KD] = {.name = "pos_kd", .min = 0.0f, .max = 1.0f},
    [PARAM_MAX_TILT_ANGLE] = {.name = "max_tilt_angle",
                              .min = 0.0f,
                              .max = 0.78f}, // 0-45 deg

    // Complementary filter (25-29)
    [PARAM_CF_ALPHA] = {.name = "cf_alpha", .min = 0.5f, .max = 1.0f},
    [PARAM_CF_MAG_ALPHA] = {.name = "cf_mag_alpha", .min = 0.5f, .max = 1.0f},
    [PARAM_CF_USE_MAG] = {.name = "cf_use_mag",
                          .min = 0.0f,
                          .max = 1.0f}, // boolean as float
    [PARAM_CF_ACCEL_THRESH_LO] = {.name = "cf_accel_thresh_lo",
                                  .min = 0.5f,
                                  .max = 1.0f},
    [PARAM_CF_ACCEL_THRESH_HI] = {.name = "cf_accel_thresh_hi",
                                  .min = 1.0f,
                                  .max = 1.5f},

    // Waypoint navigation (30-34)
    [PARAM_WP_TOLERANCE_XY] = {.name = "wp_tolerance_xy",
                               .min = 0.01f,
                               .max = 1.0f},
    [PARAM_WP_TOLERANCE_Z] = {.name = "wp_tolerance_z",
                              .min = 0.01f,
                              .max = 0.5f},
    [PARAM_WP_TOLERANCE_YAW] = {.name = "wp_tolerance_yaw",
                                .min = 0.01f,
                                .max = 0.5f},
    [PARAM_WP_TOLERANCE_VEL] = {.name = "wp_tolerance_vel",
                                .min = 0.01f,
                                .max = 0.5f},
    [PARAM_WP_HOVER_TIME_S] = {.name = "wp_hover_time_s",
                               .min = 0.0f,
                               .max = 60.0f},

    // Altitude Kalman filter (35-41)
    // Q (process noise) - how much we trust the model vs measurements
    [PARAM_KF_Q_ALTITUDE] = {.name = "kf_q_altitude",
                             .min = 0.00001f,
                             .max = 0.01f},
    [PARAM_KF_Q_VELOCITY] = {.name = "kf_q_velocity",
                             .min = 0.001f,
                             .max = 10.0f},
    [PARAM_KF_Q_BIAS] = {.name = "kf_q_bias", .min = 0.000001f, .max = 0.01f},
    // R (measurement noise) - sensor noise variance
    [PARAM_KF_R_ALTITUDE] = {.name = "kf_r_altitude",
                             .min = 0.0001f,
                             .max = 0.1f},
    // P0 (initial covariance) - initial uncertainty
    [PARAM_KF_P0_ALTITUDE] = {.name = "kf_p0_altitude",
                              .min = 0.01f,
                              .max = 10.0f},
    [PARAM_KF_P0_VELOCITY] = {.name = "kf_p0_velocity",
                              .min = 0.01f,
                              .max = 10.0f},
    [PARAM_KF_P0_BIAS] = {.name = "kf_p0_bias", .min = 0.001f, .max = 1.0f},

    // Horizontal velocity filter (42)
    [PARAM_HVEL_FILTER_ALPHA] = {.name = "hvel_filter_alpha",
                                 .min = 0.5f,
                                 .max = 0.999f},

    // Flight manager lifecycle (43-44)
    [PARAM_ARMED_COUNTDOWN_S] = {.name = "armed_countdown_s",
                                 .min = 5.0f,
                                 .max = 300.0f},
    [PARAM_AUTO_GO_DELAY_S] = {.name = "auto_go_delay_s",
                               .min = 0.0f,
                               .max = 60.0f},

    // Yaw rate PID (45-47)
    [PARAM_RATE_YAW_KP] = {.name = "rate_yaw_kp", .min = 0.0f, .max = 0.5f},
    [PARAM_RATE_YAW_KI] = {.name = "rate_yaw_ki", .min = 0.0f, .max = 0.2f},
    [PARAM_RATE_YAW_KD] = {.name = "rate_yaw_kd", .min = 0.0f, .max = 0.05f},
};

void tunable_params_init(tunable_params_t *params) {
    // Rate PID - from HAL
    params->rate_kp = HAL_RATE_PID_KP;
    params->rate_ki = HAL_RATE_PID_KI;
    params->rate_kd = HAL_RATE_PID_KD;
    params->rate_imax = HAL_RATE_PID_IMAX;
    params->rate_omax_roll = HAL_RATE_PID_OMAX_ROLL;
    params->rate_omax_pitch = HAL_RATE_PID_OMAX_PITCH;
    params->rate_omax_yaw = HAL_RATE_PID_OMAX_YAW;

    // Attitude PID - from HAL
    params->att_kp = HAL_ATTITUDE_PID_KP;
    params->att_ki = HAL_ATTITUDE_PID_KI;
    params->att_kd = HAL_ATTITUDE_PID_KD;
    params->att_imax = HAL_ATTITUDE_PID_IMAX;
    params->att_omax = HAL_ATTITUDE_PID_OMAX;

    // Altitude PID - from HAL
    params->alt_kp = HAL_ALT_PID_KP;
    params->alt_ki = HAL_ALT_PID_KI;
    params->alt_kd = HAL_ALT_PID_KD;
    params->alt_imax = HAL_ALT_PID_IMAX;
    params->alt_omax = HAL_ALT_PID_OMAX;
    params->vvel_damping = HAL_VVEL_DAMPING_GAIN;

    // Emergency limits - from config.h
    params->emergency_tilt_limit = EMERGENCY_TILT_LIMIT;
    params->emergency_alt_max = EMERGENCY_ALTITUDE_MAX;

    // Landing - local defaults (matching altitude_actor.c)
    params->landing_descent_rate = -0.15f;
    params->landing_velocity_gain = 0.5f;

    // Position control - from HAL
    params->pos_kp = HAL_POS_KP;
    params->pos_kd = HAL_POS_KD;
    params->max_tilt_angle = HAL_MAX_TILT_ANGLE;

    // Complementary filter - from CF_CONFIG_DEFAULT
    params->cf_alpha = 0.995f;
    params->cf_mag_alpha = 0.95f;
    params->cf_use_mag = 1.0f; // true
    params->cf_accel_thresh_lo = 0.8f;
    params->cf_accel_thresh_hi = 1.2f;

    // Waypoint navigation - from config.h
    params->wp_tolerance_xy = WAYPOINT_TOLERANCE_XY;
    params->wp_tolerance_z = WAYPOINT_TOLERANCE_Z;
    params->wp_tolerance_yaw = WAYPOINT_TOLERANCE_YAW;
    params->wp_tolerance_vel = WAYPOINT_TOLERANCE_VEL;
    // Default hover time from flight profile (2 seconds)
    params->wp_hover_time_s = 2.0f;

    // Altitude Kalman filter - from config.h
    params->kf_q_altitude = ALT_KF_Q_ALTITUDE;
    params->kf_q_velocity = ALT_KF_Q_VELOCITY;
    params->kf_q_bias = ALT_KF_Q_BIAS;
    params->kf_r_altitude = ALT_KF_R_ALTITUDE;
    params->kf_p0_altitude = ALT_KF_P0_ALTITUDE;
    params->kf_p0_velocity = ALT_KF_P0_VELOCITY;
    params->kf_p0_bias = ALT_KF_P0_BIAS;

    // Horizontal velocity filter - from config.h
    params->hvel_filter_alpha = HVEL_FILTER_ALPHA;

    // Yaw rate PID - from HAL (separate from roll/pitch)
    params->rate_yaw_kp = HAL_RATE_YAW_PID_KP;
    params->rate_yaw_ki = HAL_RATE_YAW_PID_KI;
    params->rate_yaw_kd = HAL_RATE_YAW_PID_KD;

    // Flight manager lifecycle
#ifdef SIMULATED_TIME
    params->armed_countdown_s = 5.0f; // Short countdown for testing
    params->auto_go_delay_s = 2.0f;   // Auto-GO after 2s in simulation
#else
    params->armed_countdown_s = 10.0f; // 10s countdown (tunable via radio)
    params->auto_go_delay_s = 0.0f;    // Require manual GO on hardware
#endif
}

hive_status_t tunable_params_set(tunable_params_t *params,
                                 tunable_param_id_t id, float value) {
    if (id >= TUNABLE_PARAM_COUNT) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "param id out of range");
    }

    const param_meta_t *meta = &param_meta[id];

    // Validate range
    if (value < meta->min || value > meta->max) {
        HIVE_LOG_WARN("[PARAMS] %s=%.4f out of range [%.4f, %.4f]", meta->name,
                      value, meta->min, meta->max);
        return HIVE_ERROR(HIVE_ERR_INVALID, "value out of range");
    }

    // Set value using pointer arithmetic
    // tunable_params_t fields are laid out to match enum order
    float *base = (float *)params;
    base[id] = value;

    HIVE_LOG_INFO("[PARAMS] Set %s=%.4f", meta->name, value);
    return HIVE_SUCCESS;
}

float tunable_params_get(const tunable_params_t *params,
                         tunable_param_id_t id) {
    if (id >= TUNABLE_PARAM_COUNT) {
        return 0.0f;
    }

    const float *base = (const float *)params;
    return base[id];
}

const char *tunable_params_name(tunable_param_id_t id) {
    if (id >= TUNABLE_PARAM_COUNT) {
        return "unknown";
    }
    return param_meta[id].name;
}

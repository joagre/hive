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
// Format: { name, min, max }
static const param_meta_t param_meta[TUNABLE_PARAM_COUNT] = {
    // Rate PID (0-6)
    [PARAM_RATE_KP] = {"rate_kp", 0.0f, 0.5f},
    [PARAM_RATE_KI] = {"rate_ki", 0.0f, 0.1f},
    [PARAM_RATE_KD] = {"rate_kd", 0.0f, 0.05f},
    [PARAM_RATE_IMAX] = {"rate_imax", 0.0f, 1.0f},
    [PARAM_RATE_OMAX_ROLL] = {"rate_omax_roll", 0.0f, 0.5f},
    [PARAM_RATE_OMAX_PITCH] = {"rate_omax_pitch", 0.0f, 0.5f},
    [PARAM_RATE_OMAX_YAW] = {"rate_omax_yaw", 0.0f, 0.5f},

    // Attitude PID (7-11)
    [PARAM_ATT_KP] = {"att_kp", 0.0f, 10.0f},
    [PARAM_ATT_KI] = {"att_ki", 0.0f, 1.0f},
    [PARAM_ATT_KD] = {"att_kd", 0.0f, 1.0f},
    [PARAM_ATT_IMAX] = {"att_imax", 0.0f, 2.0f},
    [PARAM_ATT_OMAX] = {"att_omax", 0.0f, 5.0f},

    // Altitude PID (12-18)
    [PARAM_ALT_KP] = {"alt_kp", 0.0f, 1.0f},
    [PARAM_ALT_KI] = {"alt_ki", 0.0f, 0.5f},
    [PARAM_ALT_KD] = {"alt_kd", 0.0f, 0.5f},
    [PARAM_ALT_IMAX] = {"alt_imax", 0.0f, 1.0f},
    [PARAM_ALT_OMAX] = {"alt_omax", 0.0f, 0.5f},
    [PARAM_HOVER_THRUST] = {"hover_thrust", 0.1f, 0.9f},
    [PARAM_VVEL_DAMPING] = {"vvel_damping", 0.0f, 1.0f},

    // Emergency limits (19-20)
    [PARAM_EMERGENCY_TILT_LIMIT] = {"emergency_tilt_limit", 0.17f,
                                    1.57f}, // 10-90 deg
    [PARAM_EMERGENCY_ALT_MAX] = {"emergency_alt_max", 0.5f, 10.0f},

    // Landing (21-22)
    [PARAM_LANDING_DESCENT_RATE] = {"landing_descent_rate", -1.0f,
                                    0.0f}, // negative = down
    [PARAM_LANDING_VELOCITY_GAIN] = {"landing_velocity_gain", 0.0f, 2.0f},

    // Position control (23-25)
    [PARAM_POS_KP] = {"pos_kp", 0.0f, 1.0f},
    [PARAM_POS_KD] = {"pos_kd", 0.0f, 1.0f},
    [PARAM_MAX_TILT_ANGLE] = {"max_tilt_angle", 0.0f, 0.78f}, // 0-45 deg

    // Complementary filter (26-30)
    [PARAM_CF_ALPHA] = {"cf_alpha", 0.5f, 1.0f},
    [PARAM_CF_MAG_ALPHA] = {"cf_mag_alpha", 0.5f, 1.0f},
    [PARAM_CF_USE_MAG] = {"cf_use_mag", 0.0f, 1.0f}, // boolean as float
    [PARAM_CF_ACCEL_THRESH_LO] = {"cf_accel_thresh_lo", 0.5f, 1.0f},
    [PARAM_CF_ACCEL_THRESH_HI] = {"cf_accel_thresh_hi", 1.0f, 1.5f},

    // Waypoint navigation (31-35)
    [PARAM_WP_TOLERANCE_XY] = {"wp_tolerance_xy", 0.01f, 1.0f},
    [PARAM_WP_TOLERANCE_Z] = {"wp_tolerance_z", 0.01f, 0.5f},
    [PARAM_WP_TOLERANCE_YAW] = {"wp_tolerance_yaw", 0.01f, 0.5f},
    [PARAM_WP_TOLERANCE_VEL] = {"wp_tolerance_vel", 0.01f, 0.5f},
    [PARAM_WP_HOVER_TIME_S] = {"wp_hover_time_s", 0.0f, 60.0f},

    // Flight manager (36)
    [PARAM_THRUST_RAMP_MS] = {"thrust_ramp_ms", 0.0f, 5000.0f},

    // Altitude Kalman filter (37-43)
    // Q (process noise) - how much we trust the model vs measurements
    [PARAM_KF_Q_ALTITUDE] = {"kf_q_altitude", 0.00001f, 0.01f},
    [PARAM_KF_Q_VELOCITY] = {"kf_q_velocity", 0.001f, 10.0f},
    [PARAM_KF_Q_BIAS] = {"kf_q_bias", 0.000001f, 0.01f},
    // R (measurement noise) - sensor noise variance
    [PARAM_KF_R_ALTITUDE] = {"kf_r_altitude", 0.0001f, 0.1f},
    // P0 (initial covariance) - initial uncertainty
    [PARAM_KF_P0_ALTITUDE] = {"kf_p0_altitude", 0.01f, 10.0f},
    [PARAM_KF_P0_VELOCITY] = {"kf_p0_velocity", 0.01f, 10.0f},
    [PARAM_KF_P0_BIAS] = {"kf_p0_bias", 0.001f, 1.0f},

    // Horizontal velocity filter (44)
    [PARAM_HVEL_FILTER_ALPHA] = {"hvel_filter_alpha", 0.5f, 0.999f},
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
    params->hover_thrust = HAL_HOVER_THRUST;
    params->vvel_damping = HAL_VVEL_DAMPING_GAIN;

    // Emergency limits - from config.h
    params->emergency_tilt_limit = EMERGENCY_TILT_LIMIT;
    params->emergency_alt_max = EMERGENCY_ALTITUDE_MAX;

    // Landing - local defaults (matching altitude_actor.c)
    params->landing_descent_rate = -0.15f;
    params->landing_velocity_gain = 0.5f;

    // Position control - from config.h
    params->pos_kp = POS_KP;
    params->pos_kd = POS_KD;
    params->max_tilt_angle = MAX_TILT_ANGLE;

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

    // Flight manager
    params->thrust_ramp_ms = 500.0f; // 0.5 seconds

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

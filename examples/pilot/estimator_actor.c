// Estimator actor - Attitude estimation and state computation
//
// Subscribes to sensor bus, runs complementary filter for attitude,
// uses Kalman filter for altitude/velocity estimation, publishes state.

#include "estimator_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "tunable_params.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "fusion/complementary_filter.h"
#include "fusion/altitude_kf.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_log.h"
#include "hive_timer.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define GRAVITY 9.81f

// Sanity limits for sensor validation
// These are generous limits - anything outside indicates sensor failure
#define MAX_POSITION_M 100.0f  // 100m from origin (generous for indoor/outdoor)
#define MAX_ALTITUDE_M 50.0f   // 50m max altitude
#define MAX_VELOCITY_MPS 20.0f // 20 m/s max velocity
#define MAX_GYRO_RADPS 35.0f   // ~2000 deg/s max angular rate
#define MAX_ACCEL_MPS2 80.0f   // ~8g max acceleration

// Check if float is valid (not NaN or Inf)
static inline bool is_valid_float(float f) {
    return isfinite(f);
}

// Validate sensor data, return false if any critical value is bad
// Logs which sensor failed for post-mortem analysis
static bool validate_sensors(const sensor_data_t *s) {
    // Check gyro
    for (int i = 0; i < 3; i++) {
        if (!is_valid_float(s->gyro[i])) {
            HIVE_LOG_ERROR("[EST] Sensor fail: gyro[%d]=NaN/Inf", i);
            return false;
        }
        if (fabsf(s->gyro[i]) > MAX_GYRO_RADPS) {
            HIVE_LOG_ERROR("[EST] Sensor fail: gyro[%d]=%.1f > %.1f", i,
                           s->gyro[i], MAX_GYRO_RADPS);
            return false;
        }
    }
    // Check accelerometer
    for (int i = 0; i < 3; i++) {
        if (!is_valid_float(s->accel[i])) {
            HIVE_LOG_ERROR("[EST] Sensor fail: accel[%d]=NaN/Inf", i);
            return false;
        }
        if (fabsf(s->accel[i]) > MAX_ACCEL_MPS2) {
            HIVE_LOG_ERROR("[EST] Sensor fail: accel[%d]=%.1f > %.1f", i,
                           s->accel[i], MAX_ACCEL_MPS2);
            return false;
        }
    }
    // GPS validation (only if marked valid)
    if (s->gps_valid) {
        if (!is_valid_float(s->gps_x) || !is_valid_float(s->gps_y) ||
            !is_valid_float(s->gps_z)) {
            HIVE_LOG_ERROR("[EST] Sensor fail: GPS pos=NaN/Inf");
            return false;
        }
        if (fabsf(s->gps_x) > MAX_POSITION_M ||
            fabsf(s->gps_y) > MAX_POSITION_M) {
            HIVE_LOG_ERROR("[EST] Sensor fail: GPS xy=(%.1f,%.1f) > %.1f",
                           s->gps_x, s->gps_y, MAX_POSITION_M);
            return false;
        }
        if (s->gps_z < -1.0f || s->gps_z > MAX_ALTITUDE_M) {
            HIVE_LOG_ERROR("[EST] Sensor fail: GPS z=%.2f out of [-1,%.0f]",
                           s->gps_z, MAX_ALTITUDE_M);
            return false;
        }
    }
    // Barometer validation (only if marked valid)
    if (s->baro_valid) {
        if (!is_valid_float(s->baro_altitude)) {
            HIVE_LOG_ERROR("[EST] Sensor fail: baro=NaN/Inf");
            return false;
        }
        if (s->baro_altitude < -1.0f || s->baro_altitude > MAX_ALTITUDE_M) {
            HIVE_LOG_ERROR("[EST] Sensor fail: baro=%.2f out of [-1,%.0f]",
                           s->baro_altitude, MAX_ALTITUDE_M);
            return false;
        }
    }
    // Velocity validation (only if marked valid)
    if (s->velocity_valid) {
        if (!is_valid_float(s->velocity_x) || !is_valid_float(s->velocity_y)) {
            HIVE_LOG_ERROR("[EST] Sensor fail: velocity=NaN/Inf");
            return false;
        }
        if (fabsf(s->velocity_x) > MAX_VELOCITY_MPS ||
            fabsf(s->velocity_y) > MAX_VELOCITY_MPS) {
            HIVE_LOG_ERROR("[EST] Sensor fail: vel=(%.1f,%.1f) > %.1f",
                           s->velocity_x, s->velocity_y, MAX_VELOCITY_MPS);
            return false;
        }
    }
    return true;
}

// Actor state - initialized by estimator_actor_init
typedef struct {
    hive_bus_id_t sensor_bus;
    hive_bus_id_t state_bus;
    tunable_params_t *params;
    hive_actor_id_t flight_manager;
} estimator_state_t;

void *estimator_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static estimator_state_t state;
    state.sensor_bus = buses->sensor_bus;
    state.state_bus = buses->state_bus;
    state.params = buses->params;
    state.flight_manager = HIVE_ACTOR_ID_INVALID;
    return &state;
}

void estimator_actor(void *args, const hive_spawn_info_t *siblings,
                     size_t sibling_count) {
    estimator_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[EST] Failed to find flight_manager sibling");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    hive_status_t status = hive_bus_subscribe(state->sensor_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[EST] bus subscribe failed: %s", HIVE_ERR_STR(status));
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Initialize complementary filter for attitude using tunable params
    tunable_params_t *p = state->params;
    cf_state_t filter;
    cf_config_t cf_config = {
        .alpha = p->cf_alpha,
        .mag_alpha = p->cf_mag_alpha,
        .use_mag = (p->cf_use_mag > 0.5f),
        .accel_threshold_lo = p->cf_accel_thresh_lo,
        .accel_threshold_hi = p->cf_accel_thresh_hi,
    };
    cf_init(&filter, &cf_config);

    // Initialize altitude Kalman filter using tunable params
    altitude_kf_state_t alt_kf;
    altitude_kf_config_t kf_config = {.q_altitude = p->kf_q_altitude,
                                      .q_velocity = p->kf_q_velocity,
                                      .q_bias = p->kf_q_bias,
                                      .r_altitude = p->kf_r_altitude,
                                      .p0_altitude = p->kf_p0_altitude,
                                      .p0_velocity = p->kf_p0_velocity,
                                      .p0_bias = p->kf_p0_bias};
    altitude_kf_init(&alt_kf, &kf_config);

    // Horizontal velocity (still using differentiation + LPF for now)
    float prev_x = 0.0f;
    float prev_y = 0.0f;
    float x_velocity = 0.0f;
    float y_velocity = 0.0f;
    bool first_sample = true;

    // Last known valid position (held when GPS becomes invalid)
    float last_valid_x = 0.0f;
    float last_valid_y = 0.0f;

    // Rangefinder mode (like Bitcraze's "surfaceFollowingMode")
    // Once rangefinder gives valid reading, we stay in this mode and NEVER
    // fall back to baro (which is unreliable due to prop wash)
    bool rangefinder_mode = false;
    float last_valid_rangefinder_alt = 0.0f;

    // Track sensor state transitions for logging
    bool prev_gps_valid = false;
    bool prev_velocity_valid = false;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    // Set up hive_select() sources: sensor bus + RESET notification
    enum { SEL_SENSOR, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_SENSOR] = {HIVE_SEL_BUS, .bus = state->sensor_bus},
        [SEL_RESET] = {HIVE_SEL_IPC, .ipc = {state->flight_manager,
                                             HIVE_MSG_NOTIFY, NOTIFY_RESET}},
    };

    while (1) {
        sensor_data_t sensors;
        state_estimate_t est;

        // Wait for sensor data OR RESET notification
        hive_select_result_t result;
        status = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[EST] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_RESET) {
            // Reinitialize filters
            HIVE_LOG_INFO("[EST] RESET - reinitializing filters");
            cf_init(&filter, &cf_config);
            altitude_kf_init(&alt_kf, &kf_config);
            prev_x = 0.0f;
            prev_y = 0.0f;
            x_velocity = 0.0f;
            y_velocity = 0.0f;
            first_sample = true;
            last_valid_x = 0.0f;
            last_valid_y = 0.0f;
            rangefinder_mode = false;
            last_valid_rangefinder_alt = 0.0f;
            prev_gps_valid = false;
            prev_velocity_valid = false;
            prev_time = hive_get_time();
            continue;
        }

        // SEL_SENSOR: Copy sensor data from select result
        if (result.bus.len != sizeof(sensors)) {
            HIVE_LOG_ERROR("[EST] Sensor bus corrupted: size=%zu expected=%zu",
                           result.bus.len, sizeof(sensors));
            continue;
        }
        memcpy(&sensors, result.bus.data, sizeof(sensors));

        // Validate sensor data - reject garbage readings
        // (validate_sensors logs which sensor failed)
        if (!validate_sensors(&sensors)) {
            continue;
        }

        // Measure actual dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Guard against bad dt
        if (dt <= 0.0f || dt > 1.0f) {
            HIVE_LOG_WARN("[EST] bad dt=%.6f, skipping cycle", dt);
            continue;
        }

        // Update complementary filter config from tunable params (live tuning)
        filter.config.alpha = p->cf_alpha;
        filter.config.mag_alpha = p->cf_mag_alpha;
        filter.config.use_mag = (p->cf_use_mag > 0.5f);
        filter.config.accel_threshold_lo = p->cf_accel_thresh_lo;
        filter.config.accel_threshold_hi = p->cf_accel_thresh_hi;

        // Update altitude Kalman filter config from tunable params (live tuning)
        // Note: P0 changes only affect future resets, Q/R affect every cycle
        alt_kf.config.q_altitude = p->kf_q_altitude;
        alt_kf.config.q_velocity = p->kf_q_velocity;
        alt_kf.config.q_bias = p->kf_q_bias;
        alt_kf.config.r_altitude = p->kf_r_altitude;

        // Run complementary filter for attitude estimation
        cf_update(&filter, &sensors, dt);

        // Get attitude from filter
        cf_get_attitude(&filter, &est.roll, &est.pitch, &est.yaw);

        // Angular rates directly from gyro
        est.roll_rate = sensors.gyro[0];
        est.pitch_rate = sensors.gyro[1];
        est.yaw_rate = sensors.gyro[2];

        // -----------------------------------------------------------------
        // Position from sensors
        // -----------------------------------------------------------------
        float measured_altitude = 0.0f;

        // Log GPS state transitions
        if (sensors.gps_valid != prev_gps_valid) {
            if (sensors.gps_valid) {
                HIVE_LOG_INFO("[EST] GPS valid: pos=(%.2f, %.2f, %.2f)",
                              sensors.gps_x, sensors.gps_y, sensors.gps_z);
            } else {
                HIVE_LOG_WARN("[EST] GPS lost - holding position (%.2f, %.2f)",
                              last_valid_x, last_valid_y);
            }
            prev_gps_valid = sensors.gps_valid;
        }

        if (sensors.gps_valid) {
            est.x = sensors.gps_x;
            est.y = sensors.gps_y;
            measured_altitude = sensors.gps_z;
            // Update last known valid position
            last_valid_x = est.x;
            last_valid_y = est.y;
        } else {
            // Hold last known position when GPS becomes invalid
            // (e.g., flow deck out of range). This prevents position
            // controller from generating aggressive corrections.
            est.x = last_valid_x;
            est.y = last_valid_y;
        }

        // Altitude source: Rangefinder only (like Bitcraze's "surfaceFollowingMode")
        // Once rangefinder works, we stay in rangefinder mode and NEVER use baro.
        // Baro is unreliable at low altitude due to prop wash from motors.
        if (sensors.gps_z >= 0.01f && sensors.gps_z <= 1.3f) {
            // Good rangefinder reading - use it
            measured_altitude = sensors.gps_z;
            last_valid_rangefinder_alt = sensors.gps_z;
            rangefinder_mode = true;
        } else if (rangefinder_mode) {
            // Rangefinder invalid but we've used it before - hold last value.
            // The Kalman filter will coast on accelerometer integration.
            measured_altitude = last_valid_rangefinder_alt;
        } else if (sensors.baro_valid) {
            // Never had rangefinder - use baro (pre-flight on ground)
            measured_altitude = sensors.baro_altitude;
        }

        // -----------------------------------------------------------------
        // Altitude and vertical velocity estimation (Kalman filter)
        // -----------------------------------------------------------------
        // Compute vertical acceleration in world frame
        // Body frame accel_z points down when level, world frame z points up
        // Need to rotate body accel to world frame and extract vertical component
        float cos_roll = cosf(est.roll);
        float sin_roll = sinf(est.roll);
        float cos_pitch = cosf(est.pitch);
        float sin_pitch = sinf(est.pitch);

        // Simplified rotation: vertical component of body-frame accel
        // accel_world_z = -ax*sin(pitch) + ay*sin(roll)*cos(pitch) + az*cos(roll)*cos(pitch)
        // Note: sensors.accel[2] includes gravity (~9.81 when level)
        float accel_world_z = -sensors.accel[0] * sin_pitch +
                              sensors.accel[1] * sin_roll * cos_pitch +
                              sensors.accel[2] * cos_roll * cos_pitch;

        // Remove gravity to get true vertical acceleration
        // When level and stationary, accel_world_z is ~9.81, so subtracting gives ~0
        float accel_z = accel_world_z - GRAVITY;

        // Run Kalman filter
        if (!alt_kf.initialized) {
            if (!isfinite(measured_altitude)) {
                HIVE_LOG_ERROR(
                    "[EST] NaN altitude at KF init - sensor failure!");
                measured_altitude = 0.0f;
            }
            altitude_kf_reset(&alt_kf, measured_altitude);
        }
        altitude_kf_update(&alt_kf, accel_z, measured_altitude, dt);

        // Get estimates from KF
        altitude_kf_get_state(&alt_kf, &est.altitude, &est.vertical_velocity,
                              NULL);

        // -----------------------------------------------------------------
        // Horizontal velocity
        // -----------------------------------------------------------------
        // Log velocity source transitions
        if (sensors.velocity_valid != prev_velocity_valid) {
            if (sensors.velocity_valid) {
                HIVE_LOG_INFO("[EST] Velocity source: optical flow");
            } else {
                HIVE_LOG_WARN(
                    "[EST] Velocity source: differentiation (flow lost)");
            }
            prev_velocity_valid = sensors.velocity_valid;
        }

        if (sensors.velocity_valid) {
            // Use direct velocity from HAL (e.g., optical flow)
            // This is higher quality than differentiated position
            x_velocity = sensors.velocity_x;
            y_velocity = sensors.velocity_y;
            first_sample = false;
        } else if (first_sample) {
            // No velocity yet, initialize to zero
            x_velocity = 0.0f;
            y_velocity = 0.0f;
            first_sample = false;
        } else if (dt > 0.0f) {
            // Fallback: differentiate position + LPF
            float raw_vx = (est.x - prev_x) / dt;
            float raw_vy = (est.y - prev_y) / dt;
            x_velocity = LPF(x_velocity, raw_vx, p->hvel_filter_alpha);
            y_velocity = LPF(y_velocity, raw_vy, p->hvel_filter_alpha);
        }
        prev_x = est.x;
        prev_y = est.y;
        est.x_velocity = x_velocity;
        est.y_velocity = y_velocity;

        status = hive_bus_publish(state->state_bus, &est, sizeof(est));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[EST] bus publish failed: %s", HIVE_ERR_STR(status));
        }
    }
}

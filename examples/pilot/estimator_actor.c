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
#include "fusion/horizontal_kf.h"
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
    // Check accel/gyro validity flags (pre-flight finding #5).
    // Skipping one 4ms cycle on I2C failure is better than running on stale data.
    if (!s->accel_valid || !s->gyro_valid) {
        HIVE_LOG_WARN("[EST] Sensor fail: accel_valid=%d gyro_valid=%d",
                      s->accel_valid, s->gyro_valid);
        return false;
    }
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
    // Barometer: skip validation entirely. Baro is unreliable (prop wash,
    // temperature drift) and we use the rangefinder for altitude. Bad baro
    // must never reject the entire sensor reading and starve the pipeline.
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

    // Horizontal Kalman filters (one per axis)
    horizontal_kf_state_t hkf_x;
    horizontal_kf_state_t hkf_y;
    horizontal_kf_config_t hkf_config = {.q_position = p->hkf_q_position,
                                         .q_velocity = p->hkf_q_velocity,
                                         .q_bias = p->hkf_q_bias,
                                         .r_velocity = p->hkf_r_velocity,
                                         .p0_position = p->hkf_p0_position,
                                         .p0_velocity = p->hkf_p0_velocity,
                                         .p0_bias = p->hkf_p0_bias};
    horizontal_kf_init(&hkf_x, &hkf_config);
    horizontal_kf_init(&hkf_y, &hkf_config);

    // Rangefinder mode (like Bitcraze's "surfaceFollowingMode")
    // Once rangefinder gives valid reading, we stay in this mode and NEVER
    // fall back to baro (which is unreliable due to prop wash)
    bool rangefinder_mode = false;
    float last_valid_rangefinder_alt = 0.0f;
    uint64_t last_rangefinder_time = 0; // For ground-level drift prevention

    // Track previous altitude measurement to detect new data.
    // The rangefinder updates at ~40Hz while the control loop runs at 250Hz.
    // Only run KF correction when a genuinely new measurement arrives,
    // otherwise the covariance shrinks too fast (same data treated as
    // independent observations).
    bool have_altitude_measurement = false;

    // Track sensor state transitions for logging
    bool prev_gps_valid = false;
    bool prev_velocity_valid = false;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    // Set up hive_select() sources: sensor bus + RESET notification
    enum { SEL_SENSOR, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_SENSOR] = {.type = HIVE_SEL_BUS, .bus = state->sensor_bus},
        [SEL_RESET] = {.type = HIVE_SEL_IPC,
                       .ipc = {.sender = state->flight_manager,
                               .class = HIVE_MSG_REQUEST,
                               .id = NOTIFY_RESET}},
    };

    while (true) {
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
            // Reinitialize filters but preserve rangefinder metadata.
            // The KF state resets (altitude, velocity, covariance) but we
            // keep rangefinder_mode and last reading so the ground-level
            // drift prevention stays active between consecutive flights.
            // Without this, the KF drifts negative for ~5s after RESET
            // (no rangefinder corrections) causing blind max-thrust flight.
            HIVE_LOG_INFO("[EST] RESET - reinitializing filters");
            cf_init(&filter, &cf_config);
            altitude_kf_init(&alt_kf, &kf_config);
            horizontal_kf_init(&hkf_x, &hkf_config);
            horizontal_kf_init(&hkf_y, &hkf_config);
            // rangefinder_mode, last_valid_rangefinder_alt, and
            // last_rangefinder_time intentionally preserved across RESET
            have_altitude_measurement = false;
            prev_gps_valid = false;
            prev_velocity_valid = false;
            prev_time = hive_get_time();
            hive_status_t rs = hive_ipc_reply(&result.ipc, NULL, 0);
            if (HIVE_FAILED(rs)) {
                HIVE_LOG_ERROR("[EST] RESET reply failed: %s",
                               HIVE_ERR_STR(rs));
            }
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

        // Update horizontal KF config from tunable params (live tuning)
        hkf_x.config.q_position = p->hkf_q_position;
        hkf_x.config.q_velocity = p->hkf_q_velocity;
        hkf_x.config.q_bias = p->hkf_q_bias;
        hkf_x.config.r_velocity = p->hkf_r_velocity;
        hkf_y.config.q_position = p->hkf_q_position;
        hkf_y.config.q_velocity = p->hkf_q_velocity;
        hkf_y.config.q_bias = p->hkf_q_bias;
        hkf_y.config.r_velocity = p->hkf_r_velocity;

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
                HIVE_LOG_TRACE("[EST] GPS valid: pos=(%.2f, %.2f, %.2f)",
                               sensors.gps_x, sensors.gps_y, sensors.gps_z);
            } else {
                HIVE_LOG_TRACE("[EST] GPS lost");
            }
            prev_gps_valid = sensors.gps_valid;
        }

        // Altitude measurement from rangefinder (via gps_z)
        if (sensors.gps_valid) {
            measured_altitude = sensors.gps_z;
        }

        // Altitude source: Rangefinder only (like Bitcraze's "surfaceFollowingMode")
        // Once rangefinder works, we stay in rangefinder mode and NEVER use baro.
        // Baro is unreliable at low altitude due to prop wash from motors.
        bool fresh_rangefinder = false;
        if (sensors.gps_z >= RANGEFINDER_MIN_M &&
            sensors.gps_z <= RANGEFINDER_MAX_M) {
            // Good rangefinder reading - use it
            measured_altitude = sensors.gps_z;
            last_valid_rangefinder_alt = sensors.gps_z;
            last_rangefinder_time = now;
            rangefinder_mode = true;
            fresh_rangefinder = true;
        } else if (rangefinder_mode) {
            // Rangefinder invalid but we've used it before - hold last value.
            // The Kalman filter will coast on accelerometer integration.
            measured_altitude = last_valid_rangefinder_alt;

            // Ground-level drift prevention: VL53L1x reads 0 below its
            // ~40mm minimum range (drone sits at ~20-30mm on ground).
            // Without correction the KF drifts on accel bias (observed
            // 3.5 m/s drift to -21m in 6 seconds post-landing).
            // If last valid reading was near ground and rangefinder has
            // been absent >500ms, apply ground-level correction.
            if (last_valid_rangefinder_alt < RANGEFINDER_GROUND_ALT_M &&
                last_rangefinder_time > 0 &&
                (now - last_rangefinder_time) > RANGEFINDER_GROUND_TIMEOUT_US) {
                fresh_rangefinder = true;
            }
        }
        // No baro fallback - rangefinder is our only altitude source.
        // Before first rangefinder reading, Kalman filter coasts at 0.

        // -----------------------------------------------------------------
        // World-frame acceleration (full rotation including yaw)
        // -----------------------------------------------------------------
        // Body-to-world rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
        // Used for both vertical (altitude KF) and horizontal (XY KF) axes.
        float cos_roll = cosf(est.roll);
        float sin_roll = sinf(est.roll);
        float cos_pitch = cosf(est.pitch);
        float sin_pitch = sinf(est.pitch);
        float cos_yaw = cosf(est.yaw);
        float sin_yaw = sinf(est.yaw);

        float ax = sensors.accel[0];
        float ay = sensors.accel[1];
        float az = sensors.accel[2];

        // Full rotation matrix rows for world-frame acceleration:
        // R[0] = [cos_yaw*cos_pitch,
        //         cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll,
        //         cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll]
        // R[1] = [sin_yaw*cos_pitch,
        //         sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll,
        //         sin_yaw*sin_pitch*cos_roll - cos_yaw*sin_roll]
        // R[2] = [-sin_pitch, cos_pitch*sin_roll, cos_pitch*cos_roll]
        float accel_world_x =
            ax * cos_yaw * cos_pitch +
            ay * (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) +
            az * (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll);

        float accel_world_y =
            ax * sin_yaw * cos_pitch +
            ay * (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) +
            az * (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll);

        float accel_world_z = -ax * sin_pitch + ay * cos_pitch * sin_roll +
                              az * cos_pitch * cos_roll;

        // Remove gravity to get true acceleration
        float accel_z = accel_world_z - GRAVITY;

        // Run Kalman filter
        // Predict every cycle (accelerometer integration).
        // Correct only when a fresh rangefinder sample arrives (~40Hz).
        // The VL53L1x runs at 40Hz while the control loop runs at 250Hz.
        // We detect fresh data by checking if gps_z was in valid range
        // (sensors report gps_z=0 when no new ToF sample is ready).
        if (!alt_kf.initialized) {
            if (!isfinite(measured_altitude)) {
                HIVE_LOG_ERROR(
                    "[EST] NaN altitude at KF init - sensor failure!");
                measured_altitude = 0.0f;
            }
            altitude_kf_reset(&alt_kf, measured_altitude);
            have_altitude_measurement = true;
        }
        altitude_kf_predict(&alt_kf, accel_z, dt);
        if (have_altitude_measurement && fresh_rangefinder) {
            // Innovation gating: reject rangefinder outliers that would
            // corrupt the Kalman filter state and velocity estimate.
            // The VL53L1x can give spurious readings from multipath or
            // SNR issues at longer ranges.
            float innovation = measured_altitude - alt_kf.altitude;
            if (fabsf(innovation) > KF_MAX_INNOVATION) {
                HIVE_LOG_WARN("[EST] Rangefinder outlier: meas=%.3f pred=%.3f "
                              "innov=%.3f",
                              measured_altitude, alt_kf.altitude, innovation);
            } else {
                altitude_kf_correct(&alt_kf, measured_altitude);
            }
        }

        // Get estimates from KF
        altitude_kf_get_state(&alt_kf, &est.altitude, &est.vertical_velocity,
                              NULL);

        // -----------------------------------------------------------------
        // Horizontal position and velocity (Kalman filter)
        // -----------------------------------------------------------------
        // Log velocity source transitions
        if (sensors.velocity_valid != prev_velocity_valid) {
            if (sensors.velocity_valid) {
                HIVE_LOG_TRACE("[EST] Velocity source: optical flow");
            } else {
                HIVE_LOG_TRACE("[EST] Velocity lost - KF coasting on accel");
            }
            prev_velocity_valid = sensors.velocity_valid;
        }

        // Initialize horizontal KFs on first valid velocity
        if (!hkf_x.initialized && sensors.velocity_valid) {
            horizontal_kf_reset(&hkf_x);
            horizontal_kf_reset(&hkf_y);
        }

        // Predict every cycle using world-frame horizontal acceleration
        horizontal_kf_predict(&hkf_x, accel_world_x, dt);
        horizontal_kf_predict(&hkf_y, accel_world_y, dt);

        // Correct when optical flow velocity is available
        if (sensors.velocity_valid) {
            // Innovation gating: reject outlier flow readings
            float innov_x = sensors.velocity_x - hkf_x.velocity;
            float innov_y = sensors.velocity_y - hkf_y.velocity;
            if (fabsf(innov_x) <= HKF_MAX_INNOVATION) {
                horizontal_kf_correct(&hkf_x, sensors.velocity_x);
            } else {
                HIVE_LOG_WARN("[EST] Flow X outlier: meas=%.3f pred=%.3f",
                              sensors.velocity_x, hkf_x.velocity);
            }
            if (fabsf(innov_y) <= HKF_MAX_INNOVATION) {
                horizontal_kf_correct(&hkf_y, sensors.velocity_y);
            } else {
                HIVE_LOG_WARN("[EST] Flow Y outlier: meas=%.3f pred=%.3f",
                              sensors.velocity_y, hkf_y.velocity);
            }
        }

        // Get filtered state from horizontal KFs
        horizontal_kf_get_state(&hkf_x, &est.x, &est.x_velocity, NULL);
        horizontal_kf_get_state(&hkf_y, &est.y, &est.y_velocity, NULL);

        status = hive_bus_publish(state->state_bus, &est, sizeof(est));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[EST] bus publish failed: %s", HIVE_ERR_STR(status));
        }
    }
}

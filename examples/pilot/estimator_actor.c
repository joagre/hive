// Estimator actor - Attitude estimation and state computation
//
// Subscribes to sensor bus, runs complementary filter for attitude,
// uses Kalman filter for altitude/velocity estimation, publishes state.

#include "estimator_actor.h"
#include "pilot_buses.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "fusion/complementary_filter.h"
#if USE_ALTITUDE_KF
#include "fusion/altitude_kf.h"
#endif
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include "hive_timer.h"
#include <stdbool.h>
#include <math.h>

#define GRAVITY 9.81f

// Actor state - initialized by estimator_actor_init
typedef struct {
    bus_id sensor_bus;
    bus_id state_bus;
} estimator_state;

void *estimator_actor_init(void *init_args) {
    const pilot_buses *buses = init_args;
    static estimator_state state;
    state.sensor_bus = buses->sensor_bus;
    state.state_bus = buses->state_bus;
    return &state;
}

void estimator_actor(void *args, const hive_spawn_info *siblings,
                     size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;

    estimator_state *state = args;

    hive_status status = hive_bus_subscribe(state->sensor_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[EST] bus subscribe failed: %s", HIVE_ERR_STR(status));
        return;
    }

    // Initialize complementary filter for attitude
    cf_state_t filter;
    cf_config_t cf_config = CF_CONFIG_DEFAULT;
    cf_config.use_mag = true; // Use magnetometer for yaw if available
    cf_init(&filter, &cf_config);

#if USE_ALTITUDE_KF
    // Initialize altitude Kalman filter
    altitude_kf_state_t alt_kf;
    altitude_kf_config_t kf_config = {.q_altitude = ALT_KF_Q_ALTITUDE,
                                      .q_velocity = ALT_KF_Q_VELOCITY,
                                      .q_bias = ALT_KF_Q_BIAS,
                                      .r_altitude = ALT_KF_R_ALTITUDE,
                                      .p0_altitude = ALT_KF_P0_ALTITUDE,
                                      .p0_velocity = ALT_KF_P0_VELOCITY,
                                      .p0_bias = ALT_KF_P0_BIAS};
    altitude_kf_init(&alt_kf, &kf_config);
    HIVE_LOG_INFO("[EST] Using altitude Kalman filter");
#else
    // State for velocity estimation (differentiate position)
    float prev_altitude = 0.0f;
    float vertical_velocity = 0.0f;
    HIVE_LOG_INFO("[EST] Using LPF velocity estimation");
#endif

    // Horizontal velocity (still using differentiation + LPF for now)
    float prev_x = 0.0f;
    float prev_y = 0.0f;
    float x_velocity = 0.0f;
    float y_velocity = 0.0f;
    bool first_sample = true;

    // Last known valid position (held when GPS becomes invalid)
    float last_valid_x = 0.0f;
    float last_valid_y = 0.0f;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    while (1) {
        sensor_data_t sensors;
        state_estimate_t est;
        size_t len;

        // Block until sensor data available
        status = hive_bus_read_wait(state->sensor_bus, &sensors,
                                    sizeof(sensors), &len, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[EST] bus read_wait failed: %s",
                           HIVE_ERR_STR(status));
            return;
        }

        // Measure actual dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

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
            // Altitude from barometer (HAL computes from calibrated reference)
            if (sensors.baro_valid) {
                measured_altitude = sensors.baro_altitude;
            }
        }

        // -----------------------------------------------------------------
        // Altitude and vertical velocity estimation
        // -----------------------------------------------------------------
#if USE_ALTITUDE_KF
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
            altitude_kf_reset(&alt_kf, measured_altitude);
        }
        altitude_kf_update(&alt_kf, accel_z, measured_altitude, dt);

        // Get estimates from KF
        altitude_kf_get_state(&alt_kf, &est.altitude, &est.vertical_velocity,
                              NULL);
#else
        // Simple differentiation + LPF fallback
        est.altitude = measured_altitude;

        if (first_sample) {
            vertical_velocity = 0.0f;
        } else if (dt > 0.0f) {
            float raw_vvel = (est.altitude - prev_altitude) / dt;
            vertical_velocity =
                LPF(vertical_velocity, raw_vvel, VVEL_FILTER_ALPHA);
        }
        prev_altitude = est.altitude;
        est.vertical_velocity = vertical_velocity;
#endif

        // -----------------------------------------------------------------
        // Horizontal velocity
        // -----------------------------------------------------------------
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
            x_velocity = LPF(x_velocity, raw_vx, HVEL_FILTER_ALPHA);
            y_velocity = LPF(y_velocity, raw_vy, HVEL_FILTER_ALPHA);
        }
        prev_x = est.x;
        prev_y = est.y;
        est.x_velocity = x_velocity;
        est.y_velocity = y_velocity;

        if (HIVE_FAILED(
                hive_bus_publish(state->state_bus, &est, sizeof(est)))) {
            HIVE_LOG_WARN("[EST] bus publish failed");
        }
    }
}

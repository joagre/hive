// Estimator actor - Attitude estimation and state computation
//
// Subscribes to sensor bus, runs complementary filter for attitude,
// computes velocities, publishes state estimate.

#include "estimator_actor.h"
#include "pilot_buses.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "fusion/complementary_filter.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include "hive_timer.h"
#include <stdbool.h>
#include <math.h>

// Barometric altitude conversion (simplified, sea-level reference)
// altitude = 44330 * (1 - (pressure/1013.25)^0.19029)
static float pressure_to_altitude(float pressure_hpa, float ref_pressure) {
    if (pressure_hpa <= 0.0f || ref_pressure <= 0.0f) {
        return 0.0f;
    }
    // Use reference pressure for relative altitude
    return 44330.0f * (1.0f - powf(pressure_hpa / ref_pressure, 0.19029f));
}

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

    // Initialize complementary filter
    cf_state_t filter;
    cf_config_t config = CF_CONFIG_DEFAULT;
    config.use_mag = true; // Use magnetometer for yaw if available
    cf_init(&filter, &config);

    // State for velocity estimation (differentiate GPS position)
    float prev_x = 0.0f;
    float prev_y = 0.0f;
    float prev_altitude = 0.0f;
    float x_velocity = 0.0f;
    float y_velocity = 0.0f;
    float vertical_velocity = 0.0f;
    bool first_sample = true;

    // Barometer reference (set from first reading)
    float baro_ref_pressure = 0.0f;

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

        // Position from GPS (if available)
        if (sensors.gps_valid) {
            est.x = sensors.gps_x;
            est.y = sensors.gps_y;
            est.altitude = sensors.gps_z;
        } else {
            est.x = 0.0f;
            est.y = 0.0f;
            // Altitude from barometer
            if (sensors.baro_valid) {
                if (baro_ref_pressure == 0.0f) {
                    baro_ref_pressure = sensors.pressure_hpa;
                }
                est.altitude = pressure_to_altitude(sensors.pressure_hpa,
                                                    baro_ref_pressure);
            } else {
                est.altitude = 0.0f;
            }
        }

        // Compute velocities by differentiating position
        if (first_sample) {
            x_velocity = 0.0f;
            y_velocity = 0.0f;
            vertical_velocity = 0.0f;
            first_sample = false;
        } else if (dt > 0.0f) {
            float raw_vx = (est.x - prev_x) / dt;
            float raw_vy = (est.y - prev_y) / dt;
            float raw_vvel = (est.altitude - prev_altitude) / dt;
            x_velocity = LPF(x_velocity, raw_vx, HVEL_FILTER_ALPHA);
            y_velocity = LPF(y_velocity, raw_vy, HVEL_FILTER_ALPHA);
            vertical_velocity =
                LPF(vertical_velocity, raw_vvel, VVEL_FILTER_ALPHA);
        }
        prev_x = est.x;
        prev_y = est.y;
        prev_altitude = est.altitude;
        est.x_velocity = x_velocity;
        est.y_velocity = y_velocity;
        est.vertical_velocity = vertical_velocity;

        if (HIVE_FAILED(
                hive_bus_publish(state->state_bus, &est, sizeof(est)))) {
            HIVE_LOG_WARN("[EST] bus publish failed");
        }
    }
}

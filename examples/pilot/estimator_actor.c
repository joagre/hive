// Estimator actor - Attitude estimation and state computation
//
// Subscribes to sensor bus, runs complementary filter for attitude,
// computes velocities, publishes state estimate.

#include "estimator_actor.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "fusion/complementary_filter.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include <assert.h>
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

static bus_id s_sensor_bus;
static bus_id s_state_bus;

void estimator_actor_init(bus_id sensor_bus, bus_id state_bus) {
    s_sensor_bus = sensor_bus;
    s_state_bus = state_bus;
}

void estimator_actor(void *arg) {
    (void)arg;

    hive_status status = hive_bus_subscribe(s_sensor_bus);
    assert(HIVE_SUCCEEDED(status));

    // Initialize complementary filter
    cf_state_t filter;
    cf_config_t config = CF_CONFIG_DEFAULT;
    config.use_mag = true;  // Use magnetometer for yaw if available
    cf_init(&filter, &config);

    // State for velocity estimation (differentiate GPS position)
    float prev_x = 0.0f, prev_y = 0.0f, prev_altitude = 0.0f;
    float x_velocity = 0.0f, y_velocity = 0.0f, vertical_velocity = 0.0f;
    bool first_sample = true;

    // Barometer reference (set from first reading)
    float baro_ref_pressure = 0.0f;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    while (1) {
        sensor_data_t sensors;
        size_t len;

        // Block until sensor data available
        hive_bus_read_wait(s_sensor_bus, &sensors, sizeof(sensors), &len, -1);

        // Measure actual dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Run complementary filter for attitude estimation
        cf_update(&filter, &sensors, dt);

        state_estimate_t state;

        // Get attitude from filter
        cf_get_attitude(&filter, &state.roll, &state.pitch, &state.yaw);

        // Angular rates directly from gyro
        state.roll_rate = sensors.gyro[0];
        state.pitch_rate = sensors.gyro[1];
        state.yaw_rate = sensors.gyro[2];

        // Position from GPS (if available)
        if (sensors.gps_valid) {
            state.x = sensors.gps_x;
            state.y = sensors.gps_y;
            state.altitude = sensors.gps_z;
        } else {
            state.x = 0.0f;
            state.y = 0.0f;
            // Altitude from barometer
            if (sensors.baro_valid) {
                if (baro_ref_pressure == 0.0f) {
                    baro_ref_pressure = sensors.pressure_hpa;
                }
                state.altitude = pressure_to_altitude(sensors.pressure_hpa, baro_ref_pressure);
            } else {
                state.altitude = 0.0f;
            }
        }

        // Compute velocities by differentiating position
        if (first_sample) {
            x_velocity = 0.0f;
            y_velocity = 0.0f;
            vertical_velocity = 0.0f;
            first_sample = false;
        } else if (dt > 0.0f) {
            float raw_vx = (state.x - prev_x) / dt;
            float raw_vy = (state.y - prev_y) / dt;
            float raw_vvel = (state.altitude - prev_altitude) / dt;
            x_velocity = LPF(x_velocity, raw_vx, HVEL_FILTER_ALPHA);
            y_velocity = LPF(y_velocity, raw_vy, HVEL_FILTER_ALPHA);
            vertical_velocity = LPF(vertical_velocity, raw_vvel, VVEL_FILTER_ALPHA);
        }
        prev_x = state.x;
        prev_y = state.y;
        prev_altitude = state.altitude;
        state.x_velocity = x_velocity;
        state.y_velocity = y_velocity;
        state.vertical_velocity = vertical_velocity;

        hive_bus_publish(s_state_bus, &state, sizeof(state));
    }
}

// Estimator actor - Sensor fusion and state estimation
//
// For Webots: Mostly pass-through since inertial_unit provides clean attitude.
//             Computes velocities by differentiating GPS position.
//
// For real hardware: Would implement complementary filter or Kalman filter.

#include "estimator_actor.h"
#include "hive_runtime.h"
#include "types.h"
#include "config.h"
#include <assert.h>
#include <stdbool.h>

static bus_id s_imu_bus;
static bus_id s_state_bus;

void estimator_actor_init(bus_id imu_bus, bus_id state_bus) {
    s_imu_bus = imu_bus;
    s_state_bus = state_bus;
}

void estimator_actor(void *arg) {
    (void)arg;

    assert(!HIVE_FAILED(hive_bus_subscribe(s_imu_bus)));

    // State for velocity estimation (differentiate GPS position)
    float prev_x = 0.0f, prev_y = 0.0f, prev_altitude = 0.0f;
    float x_velocity = 0.0f, y_velocity = 0.0f, vertical_velocity = 0.0f;
    bool first_sample = true;

    while (1) {
        imu_data_t imu;

        if (BUS_READ(s_imu_bus, &imu)) {
            state_estimate_t state;

            // Pass through attitude (Webots inertial_unit is already fused)
            state.roll = imu.roll;
            state.pitch = imu.pitch;
            state.yaw = imu.yaw;

            // Map gyro axes to roll/pitch/yaw rates (body frame → semantic names)
            state.roll_rate = imu.gyro_x;
            state.pitch_rate = imu.gyro_y;
            state.yaw_rate = imu.gyro_z;

            // Pass through position
            state.x = imu.x;
            state.y = imu.y;
            state.altitude = imu.altitude;

            // Compute velocities by differentiating position
            if (first_sample) {
                x_velocity = 0.0f;
                y_velocity = 0.0f;
                vertical_velocity = 0.0f;
                first_sample = false;
            } else {
                float raw_vx = (imu.x - prev_x) / TIME_STEP_S;
                float raw_vy = (imu.y - prev_y) / TIME_STEP_S;
                float raw_vvel = (imu.altitude - prev_altitude) / TIME_STEP_S;
                x_velocity = LPF(x_velocity, raw_vx, HVEL_FILTER_ALPHA);
                y_velocity = LPF(y_velocity, raw_vy, HVEL_FILTER_ALPHA);
                vertical_velocity = LPF(vertical_velocity, raw_vvel, VVEL_FILTER_ALPHA);
            }
            prev_x = imu.x;
            prev_y = imu.y;
            prev_altitude = imu.altitude;
            state.x_velocity = x_velocity;
            state.y_velocity = y_velocity;
            state.vertical_velocity = vertical_velocity;

            hive_bus_publish(s_state_bus, &state, sizeof(state));
        }

        hive_yield();
    }
}

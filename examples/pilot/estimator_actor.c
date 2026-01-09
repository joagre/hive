// Estimator actor - Sensor fusion and state estimation
//
// For Webots: Mostly pass-through since inertial_unit provides clean attitude.
//             Computes vertical velocity by differentiating GPS altitude.
//
// For real hardware: Would implement complementary filter or Kalman filter.

#include "estimator_actor.h"
#include "hive_runtime.h"
#include "types.h"
#include "config.h"
#include <stdbool.h>

static bus_id s_imu_bus;
static bus_id s_state_bus;

void estimator_actor_init(bus_id imu_bus, bus_id state_bus) {
    s_imu_bus = imu_bus;
    s_state_bus = state_bus;
}

void estimator_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_imu_bus);

    // State for vertical velocity estimation
    float prev_altitude = 0.0f;
    float vertical_velocity = 0.0f;
    bool first_sample = true;

    while (1) {
        imu_data_t imu;
        size_t len;

        if (hive_bus_read(s_imu_bus, &imu, sizeof(imu), &len).code == HIVE_OK) {
            state_estimate_t state;

            // Pass through attitude (Webots inertial_unit is already fused)
            state.roll = imu.roll;
            state.pitch = imu.pitch;
            state.yaw = imu.yaw;

            // Map gyro axes to roll/pitch/yaw rates (body frame → semantic names)
            state.roll_rate = imu.gyro_x;
            state.pitch_rate = imu.gyro_y;
            state.yaw_rate = imu.gyro_z;

            // Pass through altitude
            state.altitude = imu.altitude;

            // Compute vertical velocity by differentiating altitude
            if (first_sample) {
                vertical_velocity = 0.0f;
                first_sample = false;
            } else {
                float raw_vvel = (imu.altitude - prev_altitude) / TIME_STEP_S;
                vertical_velocity = LPF(vertical_velocity, raw_vvel, VVEL_FILTER_ALPHA);
            }
            prev_altitude = imu.altitude;
            state.vertical_velocity = vertical_velocity;

            hive_bus_publish(s_state_bus, &state, sizeof(state));
        }

        hive_yield();
    }
}

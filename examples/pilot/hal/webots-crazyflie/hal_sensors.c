// Webots Crazyflie HAL - Sensors
//
// Read sensor data from Webots simulation with noise injection.

#include "../hal.h"
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <math.h>

#include "hal_internal.h"

#define GRAVITY 9.81f

void hal_read_sensors(sensor_data_t *sensors) {
    const double *gyro = wb_gyro_get_values(g_gyro);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(g_imu);
    const double *gps = wb_gps_get_values(g_gps);

    // Synthesize accelerometer from gravity + known attitude
    // (Webots Crazyflie PROTO has no accelerometer device)
    float roll = (float)rpy[0];
    float pitch = (float)rpy[1];
    sensors->accel[0] = -GRAVITY * sinf(pitch);
    sensors->accel[1] = GRAVITY * sinf(roll) * cosf(pitch);
    sensors->accel[2] = GRAVITY * cosf(roll) * cosf(pitch);

    // Add accelerometer noise
    sensors->accel[0] += ACCEL_NOISE_STDDEV * randf_gaussian();
    sensors->accel[1] += ACCEL_NOISE_STDDEV * randf_gaussian();
    sensors->accel[2] += ACCEL_NOISE_STDDEV * randf_gaussian();

    // Gyroscope (body frame, rad/s)
    sensors->gyro[0] = (float)gyro[0];
    sensors->gyro[1] = (float)gyro[1];
    sensors->gyro[2] = (float)gyro[2];

    // Add gyro noise and bias drift
    g_gyro_bias[0] += GYRO_BIAS_DRIFT * randf_gaussian();
    g_gyro_bias[1] += GYRO_BIAS_DRIFT * randf_gaussian();
    g_gyro_bias[2] += GYRO_BIAS_DRIFT * randf_gaussian();

    sensors->gyro[0] += GYRO_NOISE_STDDEV * randf_gaussian() + g_gyro_bias[0];
    sensors->gyro[1] += GYRO_NOISE_STDDEV * randf_gaussian() + g_gyro_bias[1];
    sensors->gyro[2] += GYRO_NOISE_STDDEV * randf_gaussian() + g_gyro_bias[2];

    // No magnetometer in Webots Crazyflie PROTO
    sensors->mag[0] = 0.0f;
    sensors->mag[1] = 0.0f;
    sensors->mag[2] = 0.0f;
    sensors->mag_valid = false;

    // No barometer - use GPS altitude instead
    sensors->pressure_hpa = 0.0f;
    sensors->baro_altitude = 0.0f;
    sensors->baro_temp_c = 0.0f;
    sensors->baro_valid = false;

    // GPS position with noise (simulates integrated flow deck)
    float current_gps[3] = {(float)gps[0], (float)gps[1], (float)gps[2]};
    float current_height = current_gps[2];

    // Check if height is within flow deck operational range
    bool height_valid = (current_height >= FLOW_MIN_HEIGHT) &&
                        (current_height <= FLOW_MAX_HEIGHT);

    if (height_valid) {
        sensors->gps_x =
            current_gps[0] + GPS_XY_NOISE_STDDEV * randf_gaussian();
        sensors->gps_y =
            current_gps[1] + GPS_XY_NOISE_STDDEV * randf_gaussian();
        sensors->gps_z =
            current_height + ALTITUDE_NOISE_STDDEV * randf_gaussian();
        sensors->gps_valid = true;

        // Compute velocity from GPS
        float dt = TIME_STEP_MS / 1000.0f;
        if (g_prev_gps_valid && dt > 0.0f) {
            sensors->velocity_x = (current_gps[0] - g_prev_gps[0]) / dt;
            sensors->velocity_y = (current_gps[1] - g_prev_gps[1]) / dt;
            sensors->velocity_valid = true;
        } else {
            sensors->velocity_x = 0.0f;
            sensors->velocity_y = 0.0f;
            sensors->velocity_valid = false;
        }
    } else {
        sensors->gps_x = 0.0f;
        sensors->gps_y = 0.0f;
        sensors->gps_z = current_height;
        sensors->gps_valid = false;
        sensors->velocity_x = 0.0f;
        sensors->velocity_y = 0.0f;
        sensors->velocity_valid = false;
    }

    // Store for next iteration
    g_prev_gps[0] = current_gps[0];
    g_prev_gps[1] = current_gps[1];
    g_prev_gps[2] = current_gps[2];
    g_prev_gps_valid = height_valid;
}

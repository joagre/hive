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
    //
    // Pitch axis (accel[0]) sign convention:
    // The Crazyflie HAL negates the BMI088 X-axis accel so the complementary
    // filter produces positive pitch = nose-UP (aerospace convention).
    // The Webots PROTO pitch axis has opposite polarity, so we use
    // +g*sin(pitch) here (vs -g*sin(pitch) in the naive projection)
    // to match the Crazyflie convention.
    float roll = (float)rpy[0];
    float pitch = (float)rpy[1];
    sensors->accel[0] = GRAVITY * sinf(pitch);
    sensors->accel[1] = GRAVITY * sinf(roll) * cosf(pitch);
    sensors->accel[2] = GRAVITY * cosf(roll) * cosf(pitch);
    sensors->accel_valid = true;

    // Add accelerometer noise
    sensors->accel[0] += ACCEL_NOISE_STDDEV * randf_gaussian();
    sensors->accel[1] += ACCEL_NOISE_STDDEV * randf_gaussian();
    sensors->accel[2] += ACCEL_NOISE_STDDEV * randf_gaussian();

    // Gyroscope (body frame, rad/s)
    //
    // gyro[1] negated: Webots PROTO pitch axis has opposite polarity from
    // the Crazyflie BMI088. Matches the Crazyflie HAL's gyro Y negation.
    //
    // gyro[2] NOT negated: the yaw axis difference is an actuator issue
    // (PROTO propellers spin opposite from hardware), not a sensor issue.
    // Compensated in hal_motors.c by negating cmd->yaw in the mixer.
    sensors->gyro[0] = (float)gyro[0];
    sensors->gyro[1] = -(float)gyro[1];
    sensors->gyro[2] = (float)gyro[2];
    sensors->gyro_valid = true;

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

    // Raw GPS position (clean) - sensor_actor differentiates for velocity.
    // Noise-free so differentiation doesn't amplify noise
    // (sigma_v = sigma_pos * sqrt(2) / dt would be ~0.7 m/s at 250 Hz).
    sensors->raw_gps_x = (float)gps[0];
    sensors->raw_gps_y = (float)gps[1];
    sensors->raw_gps_z = (float)gps[2];
    sensors->raw_gps_valid = true;

    // Noisy rangefinder (simulates VL53L1x measurement noise).
    // Altitude noise applied here, not in raw_gps_z, so velocity
    // differentiation stays clean while altitude KF sees realistic noise.
    sensors->range_height =
        (float)gps[2] + ALTITUDE_NOISE_STDDEV * randf_gaussian();
    sensors->range_valid = true;

    // No optical flow sensor in Webots
    sensors->flow_dpixel_x = 0;
    sensors->flow_dpixel_y = 0;
    sensors->flow_valid = false;

    // Processed fields - sensor_actor fills these from raw data above
    sensors->gps_x = 0.0f;
    sensors->gps_y = 0.0f;
    sensors->gps_z = 0.0f;
    sensors->gps_valid = false;
    sensors->velocity_x = 0.0f;
    sensors->velocity_y = 0.0f;
    sensors->velocity_valid = false;
}

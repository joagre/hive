// Webots Crazyflie HAL Implementation
//
// Hardware abstraction for Webots simulation of Bitcraze Crazyflie.
// Provides raw sensor data for the portable complementary filter.

#include "../hal.h"
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GRAVITY 9.81f

// ----------------------------------------------------------------------------
// Sensor Noise Simulation
// ----------------------------------------------------------------------------
// Realistic noise levels to test the complementary filter.
// Automatically disabled for FIRST_TEST profile (clean, predictable behavior).
// Override with -DSENSOR_NOISE=0 or -DSENSOR_NOISE=1 if needed.

#include "config.h"     // For FLIGHT_PROFILE
#include "math_utils.h" // For CLAMPF

// Noise level: 0=off, 1=low (10%), 2=full (100%)
// Default: full noise for all profiles (realistic simulation)
// Override with -DSENSOR_NOISE=0 or -DSENSOR_NOISE=1 if needed
#ifndef SENSOR_NOISE
#define SENSOR_NOISE 2 // Full noise - realistic simulation
#endif

#if SENSOR_NOISE == 0
#define ACCEL_NOISE_STDDEV 0.0f
#define GYRO_NOISE_STDDEV 0.0f
#elif SENSOR_NOISE == 1
#define ACCEL_NOISE_STDDEV 0.01f  // Low - m/s^2 (~0.1% of gravity)
#define GYRO_NOISE_STDDEV 0.0005f // Low - rad/s (~0.03 deg/s)
#else
// Realistic MEMS IMU noise (MPU6050/BMI088 class)
// Accel: ~150 ug/sqrt(Hz) at 125Hz BW -> 0.016 m/s^2
// Gyro:  ~0.007 deg/s/sqrt(Hz) at 125Hz BW -> 0.0014 rad/s
#define ACCEL_NOISE_STDDEV 0.02f // Realistic - m/s^2 (~0.2% of gravity)
#define GYRO_NOISE_STDDEV 0.001f // Realistic - rad/s (~0.06 deg/s)
#endif
#define GYRO_BIAS_DRIFT 0.0f // rad/s per step (0 = disabled)
#define GPS_NOISE_STDDEV \
    0.0f // meters (disabled - causes velocity noise issues)

// Xorshift32 PRNG state (seeded at init for variety)
static uint32_t g_rng_state = 1; // Will be seeded in hal_init()

// Accumulated gyro bias (simulates sensor drift)
static float g_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// Xorshift32 PRNG - fast, deterministic, good distribution
static uint32_t xorshift32(void) {
    uint32_t x = g_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng_state = x;
    return x;
}

// Uniform random float in [0, 1)
static float randf_uniform(void) {
    return (float)(xorshift32() & 0x7FFFFF) / (float)0x800000;
}

// Gaussian random using Box-Muller transform
static float randf_gaussian(void) {
    float u1 = randf_uniform();
    float u2 = randf_uniform();
    // Avoid log(0)
    if (u1 < 1e-10f)
        u1 = 1e-10f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

// ----------------------------------------------------------------------------
// Hardware handles
// ----------------------------------------------------------------------------

static WbDeviceTag g_motors[NUM_MOTORS];
static WbDeviceTag g_gyro;
static WbDeviceTag g_imu;
static WbDeviceTag g_gps;

// Motor direction signs (Crazyflie motor rotation directions)
static const float MOTOR_SIGNS[NUM_MOTORS] = {-1.0f, 1.0f, -1.0f, 1.0f};

// ----------------------------------------------------------------------------
// Platform Lifecycle
// ----------------------------------------------------------------------------

int hal_init(void) {
    wb_robot_init();

    // Seed PRNG from wall clock for variety in noise patterns each run.
    // This makes drift direction random, demonstrating it's noise-induced
    // random walk rather than systematic bias. Drift without position control
    // is physically expected.
    g_rng_state = (uint32_t)time(NULL) ^ 0xDEADBEEF;
    if (g_rng_state == 0)
        g_rng_state = 12345; // xorshift can't have zero state

    // Initialize motors
    const char *motor_names[NUM_MOTORS] = {"m1_motor", "m2_motor", "m3_motor",
                                           "m4_motor"};

    for (int i = 0; i < NUM_MOTORS; i++) {
        g_motors[i] = wb_robot_get_device(motor_names[i]);
        if (g_motors[i] == 0) {
            return -1;
        }
        wb_motor_set_position(g_motors[i], INFINITY);
        wb_motor_set_velocity(g_motors[i], 0.0);
    }

    // Initialize sensors
    g_gyro = wb_robot_get_device("gyro");
    g_imu = wb_robot_get_device("inertial_unit");
    g_gps = wb_robot_get_device("gps");

    if (g_gyro == 0 || g_imu == 0 || g_gps == 0) {
        return -1;
    }

    wb_gyro_enable(g_gyro, TIME_STEP_MS);
    wb_inertial_unit_enable(g_imu, TIME_STEP_MS);
    wb_gps_enable(g_gps, TIME_STEP_MS);

    return 0;
}

void hal_cleanup(void) {
    wb_robot_cleanup();
}

bool hal_self_test(void) {
    // Webots sensors always work
    return true;
}

void hal_calibrate(void) {
    // No-op: Webots sensors don't need calibration
}

void hal_arm(void) {
    // No-op: Webots motors are always ready
}

void hal_disarm(void) {
    // Stop all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        wb_motor_set_velocity(g_motors[i], 0.0);
    }
}

// ----------------------------------------------------------------------------
// Sensor Interface
// ----------------------------------------------------------------------------

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
    // Bias drifts slowly over time (random walk)
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

    // GPS with noise (includes altitude)
    sensors->gps_x = (float)gps[0] + GPS_NOISE_STDDEV * randf_gaussian();
    sensors->gps_y = (float)gps[1] + GPS_NOISE_STDDEV * randf_gaussian();
    sensors->gps_z = (float)gps[2] + GPS_NOISE_STDDEV * randf_gaussian();
    sensors->gps_valid = true;
}

// ----------------------------------------------------------------------------
// Motor Interface
// ----------------------------------------------------------------------------

// Crazyflie X-configuration mixer
//
// Motor layout:
//         Front
//       M2    M3
//         \  /
//          \/
//          /\.
//         /  \.
//       M1    M4
//         Rear
//
// Motor rotation: M1(CCW), M2(CW), M3(CCW), M4(CW)

void hal_write_torque(const torque_cmd_t *cmd) {
    // Platform-specific adjustment: negate pitch for Crazyflie coordinate frame
    float pitch = -cmd->pitch;

    // Apply mixer: convert torque to individual motor commands
    float motors[NUM_MOTORS];
    motors[0] = cmd->thrust - cmd->roll + pitch + cmd->yaw; // M1 (rear-left)
    motors[1] = cmd->thrust - cmd->roll - pitch - cmd->yaw; // M2 (front-left)
    motors[2] = cmd->thrust + cmd->roll - pitch + cmd->yaw; // M3 (front-right)
    motors[3] = cmd->thrust + cmd->roll + pitch - cmd->yaw; // M4 (rear-right)

    // Clamp and output to Webots motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        float clamped = CLAMPF(motors[i], 0.0f, 1.0f);
        wb_motor_set_velocity(g_motors[i],
                              MOTOR_SIGNS[i] * clamped * MOTOR_MAX_VELOCITY);
    }
}

// ----------------------------------------------------------------------------
// Simulated Time Interface
// ----------------------------------------------------------------------------

bool hal_step(void) {
    return wb_robot_step(TIME_STEP_MS) != -1;
}

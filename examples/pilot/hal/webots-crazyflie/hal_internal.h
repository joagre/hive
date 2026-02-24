// Webots Crazyflie HAL - Internal Shared State
//
// Shared hardware handles and state for HAL implementation files.

#ifndef HAL_INTERNAL_H
#define HAL_INTERNAL_H

#include <webots/types.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "math_utils.h"

// ----------------------------------------------------------------------------
// Hardware Handles
// ----------------------------------------------------------------------------

extern WbDeviceTag g_motors[NUM_MOTORS];
extern WbDeviceTag g_gyro;
extern WbDeviceTag g_imu;
extern WbDeviceTag g_gps;

// ----------------------------------------------------------------------------
// Simulation State
// ----------------------------------------------------------------------------

extern uint32_t g_rng_state;
extern float g_gyro_bias[3];
extern float g_motor_state[NUM_MOTORS];

// ----------------------------------------------------------------------------
// Sensor Noise Configuration
// ----------------------------------------------------------------------------

#ifndef SENSOR_NOISE
#define SENSOR_NOISE 3 // Flow deck realistic
#endif

#if SENSOR_NOISE == 0
#define ACCEL_NOISE_STDDEV 0.0f
#define GYRO_NOISE_STDDEV 0.0f
#define ALTITUDE_NOISE_STDDEV 0.0f
#define GYRO_BIAS_DRIFT 0.0f
#define GPS_XY_NOISE_STDDEV 0.0f
#elif SENSOR_NOISE == 1
#define ACCEL_NOISE_STDDEV 0.02f
#define GYRO_NOISE_STDDEV 0.001f
#define ALTITUDE_NOISE_STDDEV 0.01f
#define GYRO_BIAS_DRIFT 0.00001f
#define GPS_XY_NOISE_STDDEV 0.005f
#elif SENSOR_NOISE == 2
#define ACCEL_NOISE_STDDEV 0.05f
#define GYRO_NOISE_STDDEV 0.003f
#define ALTITUDE_NOISE_STDDEV 0.01f
#define GYRO_BIAS_DRIFT 0.00001f
#define GPS_XY_NOISE_STDDEV 0.01f
#else // SENSOR_NOISE == 3
#define ACCEL_NOISE_STDDEV 0.02f
#define GYRO_NOISE_STDDEV 0.001f
#define ALTITUDE_NOISE_STDDEV 0.01f
#define GYRO_BIAS_DRIFT 0.0f
#define GPS_XY_NOISE_STDDEV 0.002f
#endif

// ----------------------------------------------------------------------------
// Motor Configuration
// ----------------------------------------------------------------------------

#ifndef MOTOR_TIME_CONSTANT_MS
#define MOTOR_TIME_CONSTANT_MS 20.0f
#endif

// Motor direction signs
static const float MOTOR_SIGNS[NUM_MOTORS] = {-1.0f, 1.0f, -1.0f, 1.0f};

// ----------------------------------------------------------------------------
// PRNG Utilities
// ----------------------------------------------------------------------------

static inline uint32_t xorshift32(void) {
    uint32_t x = g_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng_state = x;
    return x;
}

static inline float randf_uniform(void) {
    return (float)(xorshift32() & 0x7FFFFF) / (float)0x800000;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>

static inline float randf_gaussian(void) {
    float u1 = randf_uniform();
    float u2 = randf_uniform();
    if (u1 < 1e-10f)
        u1 = 1e-10f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

#endif // HAL_INTERNAL_H

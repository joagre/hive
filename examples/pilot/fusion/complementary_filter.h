// Complementary filter for attitude estimation
//
// Portable sensor fusion algorithm that fuses accelerometer and gyroscope
// data to estimate roll and pitch. Optionally fuses magnetometer for yaw.
//
// The complementary filter combines:
// - Gyroscope: Fast response, but drifts over time
// - Accelerometer: Slow/noisy, but no drift (gravity reference)
// - Magnetometer: Heading reference (with tilt compensation)
//
// Formula: angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
//
// Typical alpha = 0.98 (98% gyro, 2% accelerometer correction)

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <stdbool.h>

// Forward declaration (sensor_data_t defined in types.h)
struct sensor_data;

// Filter configuration
typedef struct {
    float alpha;              // Complementary filter coefficient (0.0-1.0)
                              // Higher = more gyro trust, less accel correction
    float mag_alpha;          // Magnetometer filter coefficient for yaw
    bool use_mag;             // Enable magnetometer fusion for yaw
    float accel_threshold_lo; // Reject accel if magnitude below this (g)
    float accel_threshold_hi; // Reject accel if magnitude above this (g)
    float accel_bias_ki;      // Integral gain for accel bias estimation (0=off)
    float accel_bias_max;     // Maximum bias magnitude per axis (m/s^2)
} cf_config_t;

// Default configuration
// alpha = 0.995: High gyro trust, slow accel correction (reduces noise
// sensitivity) Accel thresholds: Only trust accelerometer near 1g (not during
// maneuvers)
#define CF_CONFIG_DEFAULT                                       \
    {                                                           \
        .alpha = 0.995f, .mag_alpha = 0.95f, .use_mag = false,  \
        .accel_threshold_lo = 0.8f, .accel_threshold_hi = 1.2f, \
        .accel_bias_ki = 0.5f, .accel_bias_max = 1.0f           \
    }

// Filter state
typedef struct {
    float roll;          // Current roll estimate (radians)
    float pitch;         // Current pitch estimate (radians)
    float yaw;           // Current yaw estimate (radians)
    float gyro_bias[3];  // Optional gyro bias (subtracted from readings)
    float accel_bias[3]; // Estimated accelerometer bias (m/s^2, body frame)
    cf_config_t config;  // Filter configuration
    bool initialized;    // True after first update
} cf_state_t;

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize the complementary filter.
// config: Filter configuration (NULL = use defaults)
void cf_init(cf_state_t *state, const cf_config_t *config);

// Reset filter to zero attitude.
void cf_reset(cf_state_t *state);

// Update filter with new sensor data.
// sensors: Raw sensor readings (accel, gyro, optionally mag)
// dt: Time since last update (seconds)
void cf_update(cf_state_t *state, const struct sensor_data *sensors, float dt);

// Get current attitude estimate.
void cf_get_attitude(const cf_state_t *state, float *roll, float *pitch,
                     float *yaw);

// Set gyro bias (subtracted from gyro readings before integration).
// bias: [x, y, z] bias in rad/s
void cf_set_gyro_bias(cf_state_t *state, const float bias[3]);

// Get estimated accelerometer bias (body frame, m/s^2).
void cf_get_accel_bias(const cf_state_t *state, float bias[3]);

// Utility: Calculate roll from accelerometer (radians).
// Only valid when stationary or in steady flight.
float cf_accel_roll(const float accel[3]);

// Utility: Calculate pitch from accelerometer (radians).
// Only valid when stationary or in steady flight.
float cf_accel_pitch(const float accel[3]);

// Utility: Check if accelerometer reading is valid for attitude correction.
// Returns true if magnitude is within threshold (near 1g).
bool cf_accel_valid(const float accel[3], float threshold_lo,
                    float threshold_hi);

#endif // COMPLEMENTARY_FILTER_H

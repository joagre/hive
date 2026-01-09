// Complementary filter for attitude estimation
//
// Fuses accelerometer and gyroscope data to estimate roll and pitch.
// Optionally fuses magnetometer for yaw estimation.
//
// The complementary filter combines:
// - Gyroscope: Fast response, but drifts over time
// - Accelerometer: Slow/noisy, but no drift (gravity reference)
// - Magnetometer: Heading reference (with tilt compensation)
//
// Formula: angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
//
// Typical alpha = 0.98 (98% gyro, 2% accelerometer correction)

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>
#include <stdbool.h>

// Attitude state (Euler angles in radians)
typedef struct {
    float roll;   // Rotation around X axis (positive = right wing down)
    float pitch;  // Rotation around Y axis (positive = nose up)
    float yaw;    // Rotation around Z axis (positive = clockwise from above)
} attitude_t;

// Angular rates (rad/s)
typedef struct {
    float roll_rate;   // d(roll)/dt
    float pitch_rate;  // d(pitch)/dt
    float yaw_rate;    // d(yaw)/dt
} attitude_rates_t;

// Filter configuration
typedef struct {
    float alpha;              // Complementary filter coefficient (0.0-1.0)
                              // Higher = more gyro trust, less accel correction
    float mag_alpha;          // Magnetometer filter coefficient for yaw
    bool use_mag;             // Enable magnetometer fusion for yaw
    float accel_threshold_lo; // Reject accel if magnitude below this (g)
    float accel_threshold_hi; // Reject accel if magnitude above this (g)
} attitude_config_t;

// Default configuration
// alpha = 0.98: Good balance of responsiveness and drift correction
// Accel thresholds: Only trust accelerometer near 1g (not during maneuvers)
#define ATTITUDE_CONFIG_DEFAULT { \
    .alpha = 0.98f, \
    .mag_alpha = 0.95f, \
    .use_mag = false, \
    .accel_threshold_lo = 0.8f, \
    .accel_threshold_hi = 1.2f \
}

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize the attitude filter.
// Optionally provide initial attitude (NULL = start at zero).
void attitude_init(const attitude_config_t *config, const attitude_t *initial);

// Reset filter to specified attitude (or zero if NULL).
void attitude_reset(const attitude_t *initial);

// Update filter with new sensor data.
// accel: Accelerometer readings (m/s²), used for roll/pitch reference
// gyro: Gyroscope readings (rad/s), used for angle integration
// dt: Time since last update (seconds)
void attitude_update(const float accel[3], const float gyro[3], float dt);

// Update filter with magnetometer data for yaw.
// mag: Magnetometer readings (any unit, only direction matters)
// Call this after attitude_update() if using magnetometer.
void attitude_update_mag(const float mag[3]);

// Get current attitude estimate (Euler angles in radians).
void attitude_get(attitude_t *att);

// Get current angular rates (rad/s).
// These are the gyro readings transformed to body frame.
void attitude_get_rates(attitude_rates_t *rates);

// Get roll angle (radians).
float attitude_get_roll(void);

// Get pitch angle (radians).
float attitude_get_pitch(void);

// Get yaw angle (radians).
float attitude_get_yaw(void);

// Calculate roll from accelerometer (radians).
// Only valid when stationary or in steady flight.
float attitude_accel_roll(const float accel[3]);

// Calculate pitch from accelerometer (radians).
// Only valid when stationary or in steady flight.
float attitude_accel_pitch(const float accel[3]);

// Check if accelerometer reading is valid for attitude correction.
// Returns true if magnitude is within threshold (near 1g).
bool attitude_accel_valid(const float accel[3]);

#endif // ATTITUDE_H

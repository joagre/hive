// Altitude Kalman Filter for vertical state estimation
//
// Estimates altitude, vertical velocity, and accelerometer bias using a
// linear Kalman filter. Fuses accelerometer (prediction) with rangefinder
// or barometer (correction).
//
// State vector: x = [altitude, vertical_velocity, accel_bias]^T
//
// Model:
//   altitude[k] = altitude[k-1] + vz[k-1]*dt + 0.5*(az - bias)*dt^2
//   vz[k] = vz[k-1] + (az - bias)*dt
//   bias[k] = bias[k-1]  (random walk)
//
// Measurement:
//   z = altitude + noise
//
// Tunable parameters (Q and R matrices):
//   - q_altitude: Process noise for altitude (m^2)
//   - q_velocity: Process noise for velocity (m^2/s^2)
//   - q_bias: Process noise for accel bias - controls bias drift rate (m^2/s^4)
//   - r_altitude: Measurement noise for rangefinder/baro (m^2)

#ifndef ALTITUDE_KF_H
#define ALTITUDE_KF_H

#include <stdbool.h>

// Filter configuration (tunable parameters)
typedef struct {
    // Process noise covariance (Q matrix diagonal)
    float q_altitude; // Position process noise (m^2)
    float q_velocity; // Velocity process noise (m^2/s^2)
    float q_bias;     // Accel bias random walk (m^2/s^4)

    // Measurement noise covariance (R)
    float r_altitude; // Rangefinder/baro noise (m^2)

    // Initial state uncertainty (P0 diagonal)
    float p0_altitude; // Initial altitude uncertainty (m^2)
    float p0_velocity; // Initial velocity uncertainty (m^2/s^2)
    float p0_bias;     // Initial bias uncertainty (m^2/s^4)
} altitude_kf_config_t;

// Default configuration - conservative starting point
// Tune these based on actual sensor characteristics and flight tests
#define ALTITUDE_KF_CONFIG_DEFAULT                                     \
    {                                                                  \
        .q_altitude = 0.01f, .q_velocity = 0.1f, .q_bias = 0.001f,     \
        .r_altitude = 0.05f, .p0_altitude = 1.0f, .p0_velocity = 1.0f, \
        .p0_bias = 0.1f                                                \
    }

// Filter state
typedef struct {
    // State estimate: x = [altitude, velocity, bias]
    float altitude;   // Estimated altitude (m)
    float velocity;   // Estimated vertical velocity (m/s), positive = up
    float accel_bias; // Estimated accelerometer bias (m/s^2)

    // Error covariance matrix P (3x3, symmetric, stored as 6 elements)
    // P = [p00 p01 p02]
    //     [p01 p11 p12]
    //     [p02 p12 p22]
    float p00, p01, p02; // Row 0
    float p11, p12;      // Row 1 (p01 = p10)
    float p22;           // Row 2 (p02 = p20, p12 = p21)

    // Configuration
    altitude_kf_config_t config;

    // Filter status
    bool initialized;
} altitude_kf_state_t;

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize the altitude Kalman filter.
// config: Filter configuration (NULL = use defaults)
void altitude_kf_init(altitude_kf_state_t *state,
                      const altitude_kf_config_t *config);

// Reset filter to initial state.
// initial_altitude: Starting altitude estimate (m)
void altitude_kf_reset(altitude_kf_state_t *state, float initial_altitude);

// Prediction step: propagate state using accelerometer.
// accel_z: Vertical acceleration in world frame (m/s^2), positive = up
//          Should already have gravity removed: accel_z = accel_body_z - g
// dt: Time step (seconds)
void altitude_kf_predict(altitude_kf_state_t *state, float accel_z, float dt);

// Correction step: update state using altitude measurement.
// measured_altitude: Altitude from rangefinder or barometer (m)
void altitude_kf_correct(altitude_kf_state_t *state, float measured_altitude);

// Combined predict + correct for convenience.
// accel_z: Vertical acceleration in world frame, gravity removed (m/s^2)
// measured_altitude: Altitude measurement (m)
// dt: Time step (seconds)
void altitude_kf_update(altitude_kf_state_t *state, float accel_z,
                        float measured_altitude, float dt);

// Get current state estimates.
void altitude_kf_get_state(const altitude_kf_state_t *state, float *altitude,
                           float *velocity, float *accel_bias);

// Get estimation uncertainty (square root of P diagonal).
void altitude_kf_get_uncertainty(const altitude_kf_state_t *state,
                                 float *altitude_std, float *velocity_std,
                                 float *bias_std);

#endif // ALTITUDE_KF_H

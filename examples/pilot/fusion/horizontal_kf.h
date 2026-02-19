// Horizontal Kalman Filter for XY velocity/position estimation
//
// Estimates position, velocity, and accelerometer bias for one horizontal
// axis using a linear Kalman filter. Fuses accelerometer (prediction) with
// optical flow velocity (correction). One instance per axis (X and Y).
//
// State vector: x = [position, velocity, accel_bias]^T
//
// Model (identical to altitude KF):
//   position[k] = position[k-1] + v[k-1]*dt + 0.5*(a - bias)*dt^2
//   v[k] = v[k-1] + (a - bias)*dt
//   bias[k] = bias[k-1]  (random walk)
//
// Measurement (differs from altitude KF - velocity, not position):
//   z = velocity + noise
//   H = [0, 1, 0]

#ifndef HORIZONTAL_KF_H
#define HORIZONTAL_KF_H

#include <stdbool.h>

// Filter configuration (tunable parameters)
typedef struct {
    // Process noise covariance (Q matrix diagonal)
    float q_position; // Position process noise (m^2)
    float q_velocity; // Velocity process noise (m^2/s^2)
    float q_bias;     // Accel bias random walk (m^2/s^4)

    // Measurement noise covariance (R)
    float r_velocity; // Optical flow velocity noise (m/s)^2

    // Initial state uncertainty (P0 diagonal)
    float p0_position; // Initial position uncertainty (m^2)
    float p0_velocity; // Initial velocity uncertainty (m^2/s^2)
    float p0_bias;     // Initial bias uncertainty (m^2/s^4)
} horizontal_kf_config_t;

// Filter state
typedef struct {
    // State estimate: x = [position, velocity, bias]
    float position;   // Estimated position (m)
    float velocity;   // Estimated velocity (m/s)
    float accel_bias; // Estimated accelerometer bias (m/s^2)

    // Error covariance matrix P (3x3, symmetric, stored as 6 elements)
    // P = [p00 p01 p02]
    //     [p01 p11 p12]
    //     [p02 p12 p22]
    float p00, p01, p02; // Row 0
    float p11, p12;      // Row 1 (p01 = p10)
    float p22;           // Row 2 (p02 = p20, p12 = p21)

    // Configuration
    horizontal_kf_config_t config;

    // Filter status
    bool initialized;
} horizontal_kf_state_t;

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize the horizontal Kalman filter.
// config: Filter configuration (must not be NULL)
void horizontal_kf_init(horizontal_kf_state_t *state,
                        const horizontal_kf_config_t *config);

// Reset filter to initial state (position = 0, velocity = 0).
void horizontal_kf_reset(horizontal_kf_state_t *state);

// Prediction step: propagate state using world-frame acceleration.
// accel: Horizontal acceleration in world frame (m/s^2) for this axis
// dt: Time step (seconds)
void horizontal_kf_predict(horizontal_kf_state_t *state, float accel, float dt);

// Correction step: update state using velocity measurement.
// measured_velocity: Velocity from optical flow (m/s)
void horizontal_kf_correct(horizontal_kf_state_t *state,
                           float measured_velocity);

// Get current state estimates.
// Any output pointer may be NULL to skip.
void horizontal_kf_get_state(const horizontal_kf_state_t *state,
                             float *position, float *velocity,
                             float *accel_bias);

// Get estimation uncertainty (square root of P diagonal).
// Any output pointer may be NULL to skip.
void horizontal_kf_get_uncertainty(const horizontal_kf_state_t *state,
                                   float *position_std, float *velocity_std,
                                   float *bias_std);

#endif // HORIZONTAL_KF_H

// Horizontal Kalman Filter implementation
//
// 3-state linear Kalman filter for horizontal position/velocity estimation.
// Prediction is identical to altitude_kf (same F matrix). Correction uses
// H = [0, 1, 0] (velocity measurement instead of position).
//
// See horizontal_kf.h for detailed documentation.

#include "horizontal_kf.h"
#include <math.h>
#include <stddef.h>

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------

void horizontal_kf_init(horizontal_kf_state_t *state,
                        const horizontal_kf_config_t *config) {
    state->config = *config;

    // Initialize state to zero
    state->position = 0.0f;
    state->velocity = 0.0f;
    state->accel_bias = 0.0f;

    // Initialize covariance to configured initial uncertainty
    state->p00 = state->config.p0_position;
    state->p01 = 0.0f;
    state->p02 = 0.0f;
    state->p11 = state->config.p0_velocity;
    state->p12 = 0.0f;
    state->p22 = state->config.p0_bias;

    state->initialized = false;
}

void horizontal_kf_reset(horizontal_kf_state_t *state) {
    state->position = 0.0f;
    state->velocity = 0.0f;
    state->accel_bias = 0.0f;

    // Reset covariance
    state->p00 = state->config.p0_position;
    state->p01 = 0.0f;
    state->p02 = 0.0f;
    state->p11 = state->config.p0_velocity;
    state->p12 = 0.0f;
    state->p22 = state->config.p0_bias;

    state->initialized = true;
}

// ----------------------------------------------------------------------------
// Prediction Step
// ----------------------------------------------------------------------------
//
// Identical to altitude_kf_predict - same state transition model.
//
// F = [1, dt, -0.5*dt^2]     B = [0.5*dt^2]     u = accel
//     [0,  1,     -dt  ]         [   dt   ]
//     [0,  0,      1   ]         [   0    ]
//
// Covariance prediction: P_pred = F*P*F^T + Q

void horizontal_kf_predict(horizontal_kf_state_t *state, float accel,
                           float dt) {
    // Precompute terms
    float dt2 = dt * dt;
    float half_dt2 = 0.5f * dt2;

    // Corrected acceleration (remove estimated bias)
    float accel_corrected = accel - state->accel_bias;

    // State prediction: x_pred = F*x + B*u
    float pos_pred =
        state->position + state->velocity * dt + half_dt2 * accel_corrected;
    float vel_pred = state->velocity + accel_corrected * dt;
    float bias_pred = state->accel_bias;

    // Covariance prediction: P_pred = F*P*F^T + Q
    // P = [a, b, c]    where a=p00, b=p01, c=p02
    //     [b, d, e]          d=p11, e=p12
    //     [c, e, f]          f=p22
    float a = state->p00;
    float b = state->p01;
    float c = state->p02;
    float d = state->p11;
    float e = state->p12;
    float f = state->p22;

    // Compute F*P (3x3 matrix)
    // Row 0: [1, dt, -half_dt2] * P
    float fp00 = a + dt * b - half_dt2 * c;
    float fp01 = b + dt * d - half_dt2 * e;
    float fp02 = c + dt * e - half_dt2 * f;

    // Row 1: [0, 1, -dt] * P
    float fp11 = d - dt * e;
    float fp12 = e - dt * f;

    // Row 2: [0, 0, 1] * P
    float fp22 = f;

    // Compute (F*P)*F^T
    // F^T = [1,  0, 0]
    //       [dt, 1, 0]
    //       [-half_dt2, -dt, 1]
    float p00_new = fp00 + fp01 * dt - fp02 * half_dt2;
    float p01_new = fp01 - fp02 * dt;
    float p02_new = fp02;
    float p11_new = fp11 - fp12 * dt;
    float p12_new = fp12;
    float p22_new = fp22;

    // Add process noise Q (diagonal)
    p00_new += state->config.q_position;
    p11_new += state->config.q_velocity;
    p22_new += state->config.q_bias;

    // Update state
    state->position = pos_pred;
    state->velocity = vel_pred;
    state->accel_bias = bias_pred;

    // Update covariance
    state->p00 = p00_new;
    state->p01 = p01_new;
    state->p02 = p02_new;
    state->p11 = p11_new;
    state->p12 = p12_new;
    state->p22 = p22_new;
}

// ----------------------------------------------------------------------------
// Correction Step
// ----------------------------------------------------------------------------
//
// Measurement model: z = H*x + v, where H = [0, 1, 0]
// (We measure velocity, not position)
//
// Innovation: y = z - H*x = z - velocity
// Innovation covariance: S = H*P*H^T + R = P[1][1] + R
// Kalman gain: K = P*H^T / S = [P[0][1], P[1][1], P[1][2]]^T / S
// State update: x = x + K*y
// Covariance update: P = (I - K*H)*P

void horizontal_kf_correct(horizontal_kf_state_t *state,
                           float measured_velocity) {
    // Innovation (measurement residual)
    float y = measured_velocity - state->velocity;

    // Innovation covariance: S = P[1][1] + R
    float s = state->p11 + state->config.r_velocity;

    // Avoid division by zero
    if (s < 1e-9f) {
        s = 1e-9f;
    }

    // Kalman gain: K = P*H^T / S = [P01, P11, P12]^T / S
    float k0 = state->p01 / s;
    float k1 = state->p11 / s;
    float k2 = state->p12 / s;

    // State update: x = x + K*y
    state->position += k0 * y;
    state->velocity += k1 * y;
    state->accel_bias += k2 * y;

    // Covariance update: P = (I - K*H)*P
    // K*H = [0, k0, 0]
    //       [0, k1, 0]
    //       [0, k2, 0]
    //
    // (I - K*H) = [1, -k0, 0]
    //             [0, 1-k1, 0]
    //             [0, -k2, 1]
    //
    // P_new = (I - K*H) * P

    float p00 = state->p00;
    float p01 = state->p01;
    float p02 = state->p02;
    float p11 = state->p11;
    float p12 = state->p12;
    float p22 = state->p22;

    // Row 0: [1, -k0, 0] * P
    state->p00 = p00 - k0 * p01;
    state->p01 = p01 - k0 * p11;
    state->p02 = p02 - k0 * p12;

    // Row 1: [0, 1-k1, 0] * P
    state->p11 = (1.0f - k1) * p11;
    state->p12 = (1.0f - k1) * p12;

    // Row 2: [0, -k2, 1] * P
    state->p22 = -k2 * p12 + p22;

    state->initialized = true;
}

// ----------------------------------------------------------------------------
// State Access
// ----------------------------------------------------------------------------

void horizontal_kf_get_state(const horizontal_kf_state_t *state,
                             float *position, float *velocity,
                             float *accel_bias) {
    if (position != NULL) {
        *position = state->position;
    }
    if (velocity != NULL) {
        *velocity = state->velocity;
    }
    if (accel_bias != NULL) {
        *accel_bias = state->accel_bias;
    }
}

void horizontal_kf_get_uncertainty(const horizontal_kf_state_t *state,
                                   float *position_std, float *velocity_std,
                                   float *bias_std) {
    if (position_std != NULL) {
        *position_std = sqrtf(state->p00);
    }
    if (velocity_std != NULL) {
        *velocity_std = sqrtf(state->p11);
    }
    if (bias_std != NULL) {
        *bias_std = sqrtf(state->p22);
    }
}

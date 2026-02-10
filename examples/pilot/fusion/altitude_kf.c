// Altitude Kalman Filter implementation
//
// 3-state linear Kalman filter for altitude estimation.
// See altitude_kf.h for detailed documentation.

#include "altitude_kf.h"
#include <math.h>
#include <stddef.h>

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------

void altitude_kf_init(altitude_kf_state_t *state,
                      const altitude_kf_config_t *config) {
    // Use provided config or defaults
    if (config != NULL) {
        state->config = *config;
    } else {
        state->config = (altitude_kf_config_t)ALTITUDE_KF_CONFIG_DEFAULT;
    }

    // Initialize state to zero
    state->altitude = 0.0f;
    state->velocity = 0.0f;
    state->accel_bias = 0.0f;

    // Initialize covariance to configured initial uncertainty
    state->p00 = state->config.p0_altitude;
    state->p01 = 0.0f;
    state->p02 = 0.0f;
    state->p11 = state->config.p0_velocity;
    state->p12 = 0.0f;
    state->p22 = state->config.p0_bias;

    state->initialized = false;
}

void altitude_kf_reset(altitude_kf_state_t *state, float initial_altitude) {
    state->altitude = initial_altitude;
    state->velocity = 0.0f;
    state->accel_bias = 0.0f;

    // Reset covariance
    state->p00 = state->config.p0_altitude;
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
// State transition model:
//   altitude[k] = altitude[k-1] + velocity[k-1]*dt + 0.5*(accel - bias)*dt^2
//   velocity[k] = velocity[k-1] + (accel - bias)*dt
//   bias[k] = bias[k-1]
//
// In matrix form: x_pred = F*x + B*u
//
// F = [1, dt, -0.5*dt^2]     B = [0.5*dt^2]     u = accel_z
//     [0,  1,     -dt  ]         [   dt   ]
//     [0,  0,      1   ]         [   0    ]
//
// Covariance prediction: P_pred = F*P*F^T + Q

void altitude_kf_predict(altitude_kf_state_t *state, float accel_z, float dt) {
    // Precompute terms
    float dt2 = dt * dt;
    float half_dt2 = 0.5f * dt2;

    // Corrected acceleration (remove estimated bias)
    float accel_corrected = accel_z - state->accel_bias;

    // -------------------------------------------------------------------------
    // State prediction: x_pred = F*x + B*u
    // -------------------------------------------------------------------------
    float alt_pred =
        state->altitude + state->velocity * dt + half_dt2 * accel_corrected;
    float vel_pred = state->velocity + accel_corrected * dt;
    float bias_pred = state->accel_bias; // Bias assumed constant

    // -------------------------------------------------------------------------
    // Covariance prediction: P_pred = F*P*F^T + Q
    // -------------------------------------------------------------------------
    // F = [1, dt, -0.5*dt^2]
    //     [0,  1,     -dt  ]
    //     [0,  0,      1   ]
    //
    // This is done by computing F*P first, then (F*P)*F^T
    //
    // Let's denote the old covariance as:
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

    // Compute (F*P)*F^T = FP * F^T
    // F^T = [1,  0, 0]
    //       [dt, 1, 0]
    //       [-half_dt2, -dt, 1]

    // P_pred[0][0] = fp00*1 + fp01*dt + fp02*(-half_dt2)
    float p00_new = fp00 + fp01 * dt - fp02 * half_dt2;

    // P_pred[0][1] = fp00*0 + fp01*1 + fp02*(-dt)
    float p01_new = fp01 - fp02 * dt;

    // P_pred[0][2] = fp00*0 + fp01*0 + fp02*1
    float p02_new = fp02;

    // P_pred[1][1] = fp10*0 + fp11*1 + fp12*(-dt)
    float p11_new = fp11 - fp12 * dt;

    // P_pred[1][2] = fp10*0 + fp11*0 + fp12*1
    float p12_new = fp12;

    // P_pred[2][2] = fp20*0 + fp21*0 + fp22*1
    float p22_new = fp22;

    // Add process noise Q (diagonal)
    p00_new += state->config.q_altitude;
    p11_new += state->config.q_velocity;
    p22_new += state->config.q_bias;

    // Update state
    state->altitude = alt_pred;
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
// Measurement model: z = H*x + v, where H = [1, 0, 0]
//
// Innovation: y = z - H*x = z - altitude
// Innovation covariance: S = H*P*H^T + R = P[0][0] + R
// Kalman gain: K = P*H^T / S = [P[0][0], P[0][1], P[0][2]]^T / S
// State update: x = x + K*y
// Covariance update: P = (I - K*H)*P

void altitude_kf_correct(altitude_kf_state_t *state, float measured_altitude) {
    // Innovation (measurement residual)
    float y = measured_altitude - state->altitude;

    // Innovation covariance: S = P[0][0] + R
    float s = state->p00 + state->config.r_altitude;

    // Avoid division by zero
    if (s < 1e-9f) {
        s = 1e-9f;
    }

    // Kalman gain: K = P*H^T / S = [P00, P01, P02]^T / S
    float k0 = state->p00 / s;
    float k1 = state->p01 / s;
    float k2 = state->p02 / s;

    // State update: x = x + K*y
    state->altitude += k0 * y;
    state->velocity += k1 * y;
    state->accel_bias += k2 * y;

    // Covariance update: P = (I - K*H)*P
    // K*H = [k0, 0, 0]
    //       [k1, 0, 0]
    //       [k2, 0, 0]
    //
    // (I - K*H) = [1-k0, 0, 0]
    //             [-k1,  1, 0]
    //             [-k2,  0, 1]
    //
    // P_new = (I - K*H) * P

    float p00 = state->p00;
    float p01 = state->p01;
    float p02 = state->p02;
    float p11 = state->p11;
    float p12 = state->p12;
    float p22 = state->p22;

    // Row 0: [1-k0, 0, 0] * P
    state->p00 = (1.0f - k0) * p00;
    state->p01 = (1.0f - k0) * p01;
    state->p02 = (1.0f - k0) * p02;

    // Row 1: [-k1, 1, 0] * P
    state->p11 = -k1 * p01 + p11;
    state->p12 = -k1 * p02 + p12;

    // Row 2: [-k2, 0, 1] * P
    state->p22 = -k2 * p02 + p22;

    // Ensure symmetry (p01, p02, p12 already correct from row operations)
    // Note: p01 updated in row 0, p11/p12 in row 1, p22 in row 2
    // Cross terms: p01 = p10, p02 = p20, p12 = p21 (maintained by symmetry)

    state->initialized = true;
}

// ----------------------------------------------------------------------------
// Combined Update
// ----------------------------------------------------------------------------

void altitude_kf_update(altitude_kf_state_t *state, float accel_z,
                        float measured_altitude, float dt) {
    altitude_kf_predict(state, accel_z, dt);
    altitude_kf_correct(state, measured_altitude);
}

// ----------------------------------------------------------------------------
// State Access
// ----------------------------------------------------------------------------

void altitude_kf_get_state(const altitude_kf_state_t *state, float *altitude,
                           float *velocity, float *accel_bias) {
    if (altitude != NULL) {
        *altitude = state->altitude;
    }
    if (velocity != NULL) {
        *velocity = state->velocity;
    }
    if (accel_bias != NULL) {
        *accel_bias = state->accel_bias;
    }
}

void altitude_kf_get_uncertainty(const altitude_kf_state_t *state,
                                 float *altitude_std, float *velocity_std,
                                 float *bias_std) {
    if (altitude_std != NULL) {
        *altitude_std = sqrtf(state->p00);
    }
    if (velocity_std != NULL) {
        *velocity_std = sqrtf(state->p11);
    }
    if (bias_std != NULL) {
        *bias_std = sqrtf(state->p22);
    }
}

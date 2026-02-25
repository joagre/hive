// Complementary filter for attitude estimation
//
// Portable sensor fusion that fuses accelerometer and gyroscope data.

#include "complementary_filter.h"
#include "types.h"
#include <math.h>
#include <stddef.h>

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

#define GRAVITY 9.81f
#define PI 3.14159265358979323846f
#define TWO_PI (2.0f * PI)

// ----------------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------------

// Normalize angle to [-PI, PI]
static float normalize_angle(float angle) {
    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}

// Calculate vector magnitude
static float vec3_magnitude(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

void cf_init(cf_state_t *state, const cf_config_t *config) {
    // Use provided config or defaults
    if (config != NULL) {
        state->config = *config;
    } else {
        state->config = (cf_config_t)CF_CONFIG_DEFAULT;
    }

    // Initialize attitude to zero
    state->roll = 0.0f;
    state->pitch = 0.0f;
    state->yaw = 0.0f;

    // Initialize gyro bias to zero
    state->gyro_bias[0] = 0.0f;
    state->gyro_bias[1] = 0.0f;
    state->gyro_bias[2] = 0.0f;

    // Initialize accel bias to zero
    state->accel_bias[0] = 0.0f;
    state->accel_bias[1] = 0.0f;
    state->accel_bias[2] = 0.0f;

    state->initialized = false;
}

void cf_reset(cf_state_t *state) {
    state->roll = 0.0f;
    state->pitch = 0.0f;
    state->yaw = 0.0f;
    state->initialized = false;
}

void cf_update(cf_state_t *state, const sensor_data_t *sensors, float dt) {
    // Apply gyro bias correction
    float gyro[3];
    gyro[0] = sensors->gyro[0] - state->gyro_bias[0];
    gyro[1] = sensors->gyro[1] - state->gyro_bias[1];
    gyro[2] = sensors->gyro[2] - state->gyro_bias[2];

    // Subtract estimated accel bias (online vibration bias correction)
    float accel_corrected[3];
    accel_corrected[0] = sensors->accel[0] - state->accel_bias[0];
    accel_corrected[1] = sensors->accel[1] - state->accel_bias[1];
    accel_corrected[2] = sensors->accel[2] - state->accel_bias[2];

    // -------------------------------------------------------------------------
    // Step 1: Integrate gyroscope for angle prediction
    // -------------------------------------------------------------------------
    // Simple Euler integration (good enough for small dt)

    float gyro_roll = state->roll + gyro[0] * dt;
    float gyro_pitch = state->pitch + gyro[1] * dt;
    float gyro_yaw = state->yaw + gyro[2] * dt;

    // -------------------------------------------------------------------------
    // Step 2: Calculate attitude from accelerometer (gravity reference)
    // -------------------------------------------------------------------------
    // Only valid when acceleration ~= gravity (not during aggressive maneuvers)

    bool accel_valid =
        cf_accel_valid(accel_corrected, state->config.accel_threshold_lo,
                       state->config.accel_threshold_hi);

    float accel_roll = 0.0f;
    float accel_pitch = 0.0f;

    if (accel_valid) {
        accel_roll = cf_accel_roll(accel_corrected);
        accel_pitch = cf_accel_pitch(accel_corrected);
    }

    // -------------------------------------------------------------------------
    // Step 3: Complementary filter fusion
    // -------------------------------------------------------------------------
    // angle = alpha * gyro_angle + (1 - alpha) * accel_angle
    //
    // alpha close to 1.0: Trust gyro more (responsive, but may drift)
    // alpha close to 0.0: Trust accel more (stable, but noisy/slow)

    if (accel_valid) {
        // Fuse gyro and accelerometer
        state->roll = state->config.alpha * gyro_roll +
                      (1.0f - state->config.alpha) * accel_roll;
        state->pitch = state->config.alpha * gyro_pitch +
                       (1.0f - state->config.alpha) * accel_pitch;
    } else {
        // Accelerometer invalid (maneuvering) - use gyro only
        state->roll = gyro_roll;
        state->pitch = gyro_pitch;
    }

    // Yaw: gyro integration by default
    state->yaw = gyro_yaw;

    // -------------------------------------------------------------------------
    // Step 3b: Online accelerometer bias estimation (Mahony-style integral)
    // -------------------------------------------------------------------------
    // When accel is valid (near 1g), the residual between measured and expected
    // gravity reveals vibration-induced bias. Integrating with slow Ki gives
    // the bias estimate. Only update when accel_valid to prevent maneuver
    // centripetal acceleration from corrupting the estimate.
    if (state->config.accel_bias_ki <= 0.0f) {
        // Feature disabled - clear any stale bias
        state->accel_bias[0] = 0.0f;
        state->accel_bias[1] = 0.0f;
        state->accel_bias[2] = 0.0f;
    } else if (accel_valid) {
        float cos_r = cosf(state->roll);
        float sin_r = sinf(state->roll);
        float cos_p = cosf(state->pitch);
        float sin_p = sinf(state->pitch);

        // Expected gravity in body frame from current attitude
        float expected[3];
        expected[0] = -GRAVITY * sin_p;
        expected[1] = GRAVITY * sin_r * cos_p;
        expected[2] = GRAVITY * cos_r * cos_p;

        for (int i = 0; i < 3; i++) {
            float residual = accel_corrected[i] - expected[i];
            state->accel_bias[i] += state->config.accel_bias_ki * residual * dt;
            // Clamp to maximum
            if (state->accel_bias[i] > state->config.accel_bias_max)
                state->accel_bias[i] = state->config.accel_bias_max;
            if (state->accel_bias[i] < -state->config.accel_bias_max)
                state->accel_bias[i] = -state->config.accel_bias_max;
        }
    }

    // -------------------------------------------------------------------------
    // Step 4: Magnetometer fusion for yaw (if available and enabled)
    // -------------------------------------------------------------------------
    if (state->config.use_mag && sensors->mag_valid) {
        // Tilt-compensated heading from magnetometer
        float cos_roll = cosf(state->roll);
        float sin_roll = sinf(state->roll);
        float cos_pitch = cosf(state->pitch);
        float sin_pitch = sinf(state->pitch);

        // Tilt compensation
        float mag_x_h = sensors->mag[0] * cos_pitch +
                        sensors->mag[1] * sin_roll * sin_pitch +
                        sensors->mag[2] * cos_roll * sin_pitch;

        float mag_y_h = sensors->mag[1] * cos_roll - sensors->mag[2] * sin_roll;

        // Calculate magnetic heading
        float mag_heading = atan2f(mag_y_h, mag_x_h);

        // Complementary filter for yaw - handle wrap-around at +/-PI
        float yaw_error = mag_heading - state->yaw;
        yaw_error = normalize_angle(yaw_error);

        // Apply correction
        state->yaw += (1.0f - state->config.mag_alpha) * yaw_error;
    }

    // Normalize angles to [-PI, PI]
    state->roll = normalize_angle(state->roll);
    state->pitch = normalize_angle(state->pitch);
    state->yaw = normalize_angle(state->yaw);

    state->initialized = true;
}

void cf_get_attitude(const cf_state_t *state, float *roll, float *pitch,
                     float *yaw) {
    if (roll != NULL)
        *roll = state->roll;
    if (pitch != NULL)
        *pitch = state->pitch;
    if (yaw != NULL)
        *yaw = state->yaw;
}

void cf_set_gyro_bias(cf_state_t *state, const float bias[3]) {
    state->gyro_bias[0] = bias[0];
    state->gyro_bias[1] = bias[1];
    state->gyro_bias[2] = bias[2];
}

void cf_get_accel_bias(const cf_state_t *state, float bias[3]) {
    bias[0] = state->accel_bias[0];
    bias[1] = state->accel_bias[1];
    bias[2] = state->accel_bias[2];
}

float cf_accel_roll(const float accel[3]) {
    // Roll from accelerometer:
    // roll = atan2(ay, az)
    //
    // When level: ay = 0, az = -g -> roll = 0
    // When tilted right: ay > 0 -> roll > 0
    return atan2f(accel[1], accel[2]);
}

float cf_accel_pitch(const float accel[3]) {
    // Pitch from accelerometer:
    // pitch = atan2(-ax, sqrt(ay^2 + az^2))
    //
    // When level: ax = 0 -> pitch = 0
    // When nose up: ax < 0 -> pitch > 0
    float ay_az = sqrtf(accel[1] * accel[1] + accel[2] * accel[2]);
    return atan2f(-accel[0], ay_az);
}

bool cf_accel_valid(const float accel[3], float threshold_lo,
                    float threshold_hi) {
    // Check if accelerometer magnitude is close to 1g
    // If not, the drone is accelerating and accel can't be used for attitude
    float mag = vec3_magnitude(accel) / GRAVITY; // Normalize to g

    return (mag >= threshold_lo && mag <= threshold_hi);
}

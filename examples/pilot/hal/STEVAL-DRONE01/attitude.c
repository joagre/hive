// Complementary filter for attitude estimation
//
// Fuses accelerometer and gyroscope data using a complementary filter.

#include "attitude.h"
#include <math.h>

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

#define GRAVITY 9.81f
#define PI      3.14159265358979323846f
#define TWO_PI  (2.0f * PI)

// ----------------------------------------------------------------------------
// Static state
// ----------------------------------------------------------------------------

static attitude_config_t s_config;
static attitude_t s_attitude;
static attitude_rates_t s_rates;

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

void attitude_init(const attitude_config_t *config, const attitude_t *initial) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (attitude_config_t)ATTITUDE_CONFIG_DEFAULT;
    }

    // Initialize attitude
    if (initial) {
        s_attitude = *initial;
    } else {
        s_attitude.roll = 0.0f;
        s_attitude.pitch = 0.0f;
        s_attitude.yaw = 0.0f;
    }

    // Initialize rates
    s_rates.roll_rate = 0.0f;
    s_rates.pitch_rate = 0.0f;
    s_rates.yaw_rate = 0.0f;
}

void attitude_reset(const attitude_t *initial) {
    if (initial) {
        s_attitude = *initial;
    } else {
        s_attitude.roll = 0.0f;
        s_attitude.pitch = 0.0f;
        s_attitude.yaw = 0.0f;
    }
}

void attitude_update(const float accel[3], const float gyro[3], float dt) {
    // Store angular rates (body frame)
    // Assuming gyro is [x, y, z] = [roll_rate, pitch_rate, yaw_rate]
    s_rates.roll_rate = gyro[0];
    s_rates.pitch_rate = gyro[1];
    s_rates.yaw_rate = gyro[2];

    // -------------------------------------------------------------------------
    // Step 1: Integrate gyroscope for angle prediction
    // -------------------------------------------------------------------------
    // Simple Euler integration (good enough for small dt)
    // For more accuracy with large angles, use quaternions or rotation matrices

    float gyro_roll = s_attitude.roll + gyro[0] * dt;
    float gyro_pitch = s_attitude.pitch + gyro[1] * dt;
    float gyro_yaw = s_attitude.yaw + gyro[2] * dt;

    // -------------------------------------------------------------------------
    // Step 2: Calculate attitude from accelerometer (gravity reference)
    // -------------------------------------------------------------------------
    // Only valid when acceleration ~= gravity (not during aggressive maneuvers)

    bool accel_valid = attitude_accel_valid(accel);

    float accel_roll = 0.0f;
    float accel_pitch = 0.0f;

    if (accel_valid) {
        accel_roll = attitude_accel_roll(accel);
        accel_pitch = attitude_accel_pitch(accel);
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
        s_attitude.roll = s_config.alpha * gyro_roll +
                          (1.0f - s_config.alpha) * accel_roll;
        s_attitude.pitch = s_config.alpha * gyro_pitch +
                           (1.0f - s_config.alpha) * accel_pitch;
    } else {
        // Accelerometer invalid (maneuvering) - use gyro only
        s_attitude.roll = gyro_roll;
        s_attitude.pitch = gyro_pitch;
    }

    // Yaw: gyro integration only (no gravity reference for yaw)
    // Magnetometer fusion handled separately in attitude_update_mag()
    s_attitude.yaw = gyro_yaw;

    // Normalize angles to [-PI, PI]
    s_attitude.roll = normalize_angle(s_attitude.roll);
    s_attitude.pitch = normalize_angle(s_attitude.pitch);
    s_attitude.yaw = normalize_angle(s_attitude.yaw);
}

void attitude_update_mag(const float mag[3]) {
    if (!s_config.use_mag) {
        return;
    }

    // -------------------------------------------------------------------------
    // Tilt-compensated heading from magnetometer
    // -------------------------------------------------------------------------
    // Rotate magnetometer readings into horizontal plane using current
    // roll and pitch estimates, then calculate heading.

    float cos_roll = cosf(s_attitude.roll);
    float sin_roll = sinf(s_attitude.roll);
    float cos_pitch = cosf(s_attitude.pitch);
    float sin_pitch = sinf(s_attitude.pitch);

    // Tilt compensation
    // X_h = X * cos(pitch) + Y * sin(roll) * sin(pitch) + Z * cos(roll) * sin(pitch)
    // Y_h = Y * cos(roll) - Z * sin(roll)
    float mag_x_h = mag[0] * cos_pitch +
                    mag[1] * sin_roll * sin_pitch +
                    mag[2] * cos_roll * sin_pitch;

    float mag_y_h = mag[1] * cos_roll - mag[2] * sin_roll;

    // Calculate magnetic heading
    float mag_heading = atan2f(mag_y_h, mag_x_h);

    // -------------------------------------------------------------------------
    // Complementary filter for yaw
    // -------------------------------------------------------------------------
    // Fuse gyro-integrated yaw with magnetometer heading

    // Handle wrap-around at ±PI
    float yaw_error = mag_heading - s_attitude.yaw;
    yaw_error = normalize_angle(yaw_error);

    // Apply correction
    s_attitude.yaw += (1.0f - s_config.mag_alpha) * yaw_error;
    s_attitude.yaw = normalize_angle(s_attitude.yaw);
}

void attitude_get(attitude_t *att) {
    *att = s_attitude;
}

void attitude_get_rates(attitude_rates_t *rates) {
    *rates = s_rates;
}

float attitude_get_roll(void) {
    return s_attitude.roll;
}

float attitude_get_pitch(void) {
    return s_attitude.pitch;
}

float attitude_get_yaw(void) {
    return s_attitude.yaw;
}

float attitude_accel_roll(const float accel[3]) {
    // Roll from accelerometer:
    // roll = atan2(ay, az)
    //
    // When level: ay = 0, az = -g → roll = 0
    // When tilted right: ay > 0 → roll > 0
    return atan2f(accel[1], accel[2]);
}

float attitude_accel_pitch(const float accel[3]) {
    // Pitch from accelerometer:
    // pitch = atan2(-ax, sqrt(ay² + az²))
    //
    // When level: ax = 0 → pitch = 0
    // When nose up: ax < 0 → pitch > 0
    float ay_az = sqrtf(accel[1] * accel[1] + accel[2] * accel[2]);
    return atan2f(-accel[0], ay_az);
}

bool attitude_accel_valid(const float accel[3]) {
    // Check if accelerometer magnitude is close to 1g
    // If not, the drone is accelerating and accel can't be used for attitude
    float mag = vec3_magnitude(accel) / GRAVITY;  // Normalize to g

    return (mag >= s_config.accel_threshold_lo &&
            mag <= s_config.accel_threshold_hi);
}

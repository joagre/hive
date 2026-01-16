// Shared platform types for STEVAL-DRONE01
//
// Used by both callback-based (platform.h) and Webots-compatible
// (platform_stm32f4.h) APIs.

#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H

#include <stdint.h>

// ----------------------------------------------------------------------------
// Loop timing configuration
// ----------------------------------------------------------------------------

#define PLATFORM_LOOP_FREQ_HZ 400
#define PLATFORM_LOOP_PERIOD_US (1000000 / PLATFORM_LOOP_FREQ_HZ)
#define PLATFORM_LOOP_DT (1.0f / PLATFORM_LOOP_FREQ_HZ)

// ----------------------------------------------------------------------------
// Platform state
// ----------------------------------------------------------------------------

typedef enum {
    PLATFORM_STATE_INIT,        // Initializing hardware
    PLATFORM_STATE_CALIBRATING, // Sensor calibration in progress
    PLATFORM_STATE_READY,       // Ready for flight
    PLATFORM_STATE_ARMED,       // Motors armed
    PLATFORM_STATE_FLYING,      // In flight
    PLATFORM_STATE_ERROR        // Hardware error
} platform_state_t;

// ----------------------------------------------------------------------------
// Sensor data
// ----------------------------------------------------------------------------

// Sensor data snapshot (updated each loop iteration)
typedef struct {
    // Attitude (from complementary filter)
    float roll;  // radians
    float pitch; // radians
    float yaw;   // radians

    // Angular rates (from gyro)
    float roll_rate;  // rad/s
    float pitch_rate; // rad/s
    float yaw_rate;   // rad/s

    // Altitude (from barometer)
    float altitude; // meters (relative to ground)
    float pressure; // hPa

    // Raw accelerometer (for vertical velocity estimation)
    float accel_x; // m/s^2
    float accel_y; // m/s^2
    float accel_z; // m/s^2

    // Timestamps
    uint32_t timestamp_ms; // Milliseconds since boot
    uint32_t loop_count;   // Loop iteration counter
} platform_sensors_t;

// ----------------------------------------------------------------------------
// Motor command
// ----------------------------------------------------------------------------

// Motor command (matches pilot example)
typedef struct {
    float m1; // Rear-left (0.0-1.0)
    float m2; // Front-left
    float m3; // Front-right
    float m4; // Rear-right
} platform_motors_t;

#endif // PLATFORM_TYPES_H

// Platform initialization and main loop for STEVAL-DRONE01
//
// Ties together all hardware drivers and provides the main control loop.
// This replaces the Webots simulation interface for real hardware.

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Loop timing configuration
// ----------------------------------------------------------------------------

// Main control loop frequency (Hz)
// IMU runs at this rate for responsive attitude control
#define PLATFORM_LOOP_FREQ_HZ       400

// Magnetometer update divider (runs at LOOP_FREQ / MAG_DIVIDER)
// 400 / 8 = 50 Hz
#define PLATFORM_MAG_DIVIDER        8

// Barometer update divider (runs at LOOP_FREQ / BARO_DIVIDER)
// 400 / 8 = 50 Hz
#define PLATFORM_BARO_DIVIDER       8

// Derived timing constants
#define PLATFORM_LOOP_PERIOD_US     (1000000 / PLATFORM_LOOP_FREQ_HZ)
#define PLATFORM_LOOP_DT            (1.0f / PLATFORM_LOOP_FREQ_HZ)

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

// Sensor data snapshot (updated each loop iteration)
typedef struct {
    // Attitude (from complementary filter)
    float roll;         // radians
    float pitch;        // radians
    float yaw;          // radians

    // Angular rates (from gyro)
    float roll_rate;    // rad/s
    float pitch_rate;   // rad/s
    float yaw_rate;     // rad/s

    // Altitude (from barometer)
    float altitude;     // meters (relative to ground)
    float pressure;     // hPa

    // Raw accelerometer (for vertical velocity estimation)
    float accel_x;      // m/s²
    float accel_y;      // m/s²
    float accel_z;      // m/s²

    // Timestamps
    uint32_t timestamp_ms;      // Milliseconds since boot
    uint32_t loop_count;        // Loop iteration counter
} platform_sensors_t;

// Motor command (matches pilot example)
typedef struct {
    float m1;   // Rear-left (0.0-1.0)
    float m2;   // Front-left
    float m3;   // Front-right
    float m4;   // Rear-right
} platform_motors_t;

// ----------------------------------------------------------------------------
// Callbacks (implement these in your application)
// ----------------------------------------------------------------------------

// Called once after hardware init, before main loop starts.
// Use this to initialize your actors/controllers.
typedef void (*platform_init_cb)(void);

// Called each control loop iteration with fresh sensor data.
// Return motor commands. This is your main control logic.
typedef void (*platform_control_cb)(const platform_sensors_t *sensors,
                                     platform_motors_t *motors);

// Called when platform state changes.
typedef void (*platform_state_cb)(platform_state_t old_state,
                                   platform_state_t new_state);

// Callback configuration
typedef struct {
    platform_init_cb on_init;
    platform_control_cb on_control;
    platform_state_cb on_state_change;
} platform_callbacks_t;

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize platform hardware.
// Returns true on success, false if any sensor fails.
bool platform_init(const platform_callbacks_t *callbacks);

// Run sensor calibration (gyro bias, baro reference).
// Drone must be stationary and level. Blocks until complete.
// Returns true on success.
bool platform_calibrate(void);

// Start the main control loop.
// This function does not return (runs forever).
// Call platform_init() and platform_calibrate() first.
void platform_run(void);

// Request state change.
// ARM: Enable motors (requires READY state)
// DISARM: Disable motors (any state)
bool platform_arm(void);
bool platform_disarm(void);

// Emergency stop - immediate motor shutoff.
void platform_emergency_stop(void);

// Get current platform state.
platform_state_t platform_get_state(void);

// Get current sensor data (thread-safe copy).
void platform_get_sensors(platform_sensors_t *sensors);

// Get system uptime in milliseconds.
uint32_t platform_get_time_ms(void);

// Delay for specified milliseconds.
void platform_delay_ms(uint32_t ms);

#endif // PLATFORM_H

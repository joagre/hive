// STEVAL-DRONE01 HAL Configuration
//
// Platform-specific constants for the STEVAL-DRONE01 hardware.
// These values may need tuning based on actual flight behavior.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// ----------------------------------------------------------------------------
// First Flight Test Mode
// ----------------------------------------------------------------------------
// Enable for initial hardware testing. Waypoint actor will:
//   1. Hover at 0.25m for 3 seconds
//   2. Land (descend to 0m)
//   3. Stay landed (no loop)
// Comment out for normal waypoint navigation.
#define HAL_FIRST_FLIGHT_TEST

// ----------------------------------------------------------------------------
// Thrust
// ----------------------------------------------------------------------------

// Base thrust for hover - calibrated via thrust_test.c
// 0.29 = just below liftoff, 0.30 = liftoff
#define HAL_BASE_THRUST  0.29f

// ----------------------------------------------------------------------------
// Altitude Control
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
#define HAL_ALT_PID_KP    0.5f     // Increased for faster response
#define HAL_ALT_PID_KI    0.1f     // Increased for steady-state error
#define HAL_ALT_PID_KD    0.0f     // Using velocity feedback instead
#define HAL_ALT_PID_IMAX  0.3f     // Integral limit
#define HAL_ALT_PID_OMAX  0.4f     // Allow significant climb authority

// Vertical velocity damping (measured velocity -> thrust correction)
#define HAL_VVEL_DAMPING_GAIN  0.15f

// ----------------------------------------------------------------------------
// Attitude Control
// ----------------------------------------------------------------------------

// Attitude PID gains (attitude angle error -> rate setpoint)
#define HAL_ATTITUDE_PID_KP    4.0f    // Typical autopilot: 4-8
#define HAL_ATTITUDE_PID_KI    0.0f
#define HAL_ATTITUDE_PID_KD    0.0f    // Derivative on error causes kick on setpoint change
#define HAL_ATTITUDE_PID_IMAX  0.5f    // Integral limit
#define HAL_ATTITUDE_PID_OMAX  3.0f    // Max rate setpoint (rad/s)

// ----------------------------------------------------------------------------
// Rate Control
// ----------------------------------------------------------------------------

// Rate PID gains (rate error -> torque)
#define HAL_RATE_PID_KP    0.02f
#define HAL_RATE_PID_KI    0.0f
#define HAL_RATE_PID_KD    0.001f
#define HAL_RATE_PID_IMAX  0.5f       // Integral limit
#define HAL_RATE_PID_OMAX_ROLL   0.1f
#define HAL_RATE_PID_OMAX_PITCH  0.1f
#define HAL_RATE_PID_OMAX_YAW    0.15f

// ----------------------------------------------------------------------------
// Bus Configuration
// ----------------------------------------------------------------------------
// STM32 has smaller memory limits than simulation
#define HAL_BUS_CONFIG { \
    .max_subscribers = 6, \
    .consume_after_reads = 0, \
    .max_age_ms = 0, \
    .max_entries = 1, \
    .max_entry_size = 128 \
}

#endif // HAL_CONFIG_H

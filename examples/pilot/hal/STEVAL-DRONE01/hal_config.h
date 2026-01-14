// STEVAL-DRONE01 HAL Configuration
//
// Platform-specific constants for the STEVAL-DRONE01 hardware.
// PID gains tuned for stability with realistic sensor noise.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

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
#define HAL_ALT_PID_KP    0.25f    // Conservative for noise rejection
#define HAL_ALT_PID_KI    0.05f
#define HAL_ALT_PID_KD    0.0f     // Using velocity feedback instead
#define HAL_ALT_PID_IMAX  0.2f     // Integral limit
#define HAL_ALT_PID_OMAX  0.15f    // Output limit

// Vertical velocity damping (measured velocity -> thrust correction)
#define HAL_VVEL_DAMPING_GAIN  0.25f  // Increased for oscillation damping

// ----------------------------------------------------------------------------
// Attitude Control
// ----------------------------------------------------------------------------

// Attitude PID gains (attitude angle error -> rate setpoint)
#define HAL_ATTITUDE_PID_KP    2.5f    // Reduced for noise rejection (was 4.0)
#define HAL_ATTITUDE_PID_KI    0.0f
#define HAL_ATTITUDE_PID_KD    0.15f   // Added damping for oscillation reduction
#define HAL_ATTITUDE_PID_IMAX  0.5f    // Integral limit
#define HAL_ATTITUDE_PID_OMAX  2.0f    // Reduced max rate setpoint (rad/s)

// ----------------------------------------------------------------------------
// Rate Control
// ----------------------------------------------------------------------------

// Rate PID gains (rate error -> torque)
#define HAL_RATE_PID_KP    0.015f    // Slightly reduced for noise rejection
#define HAL_RATE_PID_KI    0.0f
#define HAL_RATE_PID_KD    0.002f    // Increased damping
#define HAL_RATE_PID_IMAX  0.5f      // Integral limit
#define HAL_RATE_PID_OMAX_ROLL   0.08f   // Reduced for smoother control
#define HAL_RATE_PID_OMAX_PITCH  0.08f
#define HAL_RATE_PID_OMAX_YAW    0.12f

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

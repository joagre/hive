// Webots Crazyflie HAL Configuration
//
// Platform-specific constants for the Webots Crazyflie simulation.
// Tuned for the Webots Crazyflie model.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// ----------------------------------------------------------------------------
// Thrust
// ----------------------------------------------------------------------------

// Base thrust for hover (tuned for Webots Crazyflie simulation)
#define HAL_BASE_THRUST  0.553f

// ----------------------------------------------------------------------------
// Altitude Control
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
#define HAL_ALT_PID_KP    0.3f
#define HAL_ALT_PID_KI    0.05f
#define HAL_ALT_PID_KD    0.0f     // Using velocity feedback instead
#define HAL_ALT_PID_IMAX  0.2f     // Integral limit
#define HAL_ALT_PID_OMAX  0.15f    // Output limit

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
// Same limits as STM32 to ensure simulation matches production constraints
#define HAL_BUS_CONFIG { \
    .max_subscribers = 6, \
    .consume_after_reads = 0, \
    .max_age_ms = 0, \
    .max_entries = 1, \
    .max_entry_size = 128 \
}

#endif // HAL_CONFIG_H

// Crazyflie 2.1+ HAL Configuration
//
// Platform-specific constants for Crazyflie 2.1+ hardware.
// PID gains will need tuning once hardware is available.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// ----------------------------------------------------------------------------
// Thrust
// ----------------------------------------------------------------------------

// Base thrust for hover - needs calibration on actual hardware
// Crazyflie is lighter than STEVAL-DRONE01, likely needs less thrust
#define HAL_BASE_THRUST  0.35f

// ----------------------------------------------------------------------------
// Altitude Control
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
// Initial values based on STEVAL, will need tuning
#define HAL_ALT_PID_KP    0.25f
#define HAL_ALT_PID_KI    0.05f
#define HAL_ALT_PID_KD    0.0f
#define HAL_ALT_PID_IMAX  0.2f
#define HAL_ALT_PID_OMAX  0.15f

// Vertical velocity damping (measured velocity -> thrust correction)
#define HAL_VVEL_DAMPING_GAIN  0.25f

// ----------------------------------------------------------------------------
// Attitude Control
// ----------------------------------------------------------------------------

// Attitude PID gains (attitude angle error -> rate setpoint)
#define HAL_ATTITUDE_PID_KP    2.5f
#define HAL_ATTITUDE_PID_KI    0.0f
#define HAL_ATTITUDE_PID_KD    0.15f
#define HAL_ATTITUDE_PID_IMAX  0.5f
#define HAL_ATTITUDE_PID_OMAX  2.0f

// ----------------------------------------------------------------------------
// Rate Control
// ----------------------------------------------------------------------------

// Rate PID gains (rate error -> torque)
#define HAL_RATE_PID_KP    0.015f
#define HAL_RATE_PID_KI    0.0f
#define HAL_RATE_PID_KD    0.002f
#define HAL_RATE_PID_IMAX  0.5f
#define HAL_RATE_PID_OMAX_ROLL   0.08f
#define HAL_RATE_PID_OMAX_PITCH  0.08f
#define HAL_RATE_PID_OMAX_YAW    0.12f

// ----------------------------------------------------------------------------
// Position Control (using Flow Deck)
// ----------------------------------------------------------------------------

// Position PID gains - for optical flow based position hold
#define HAL_POS_PID_KP    0.5f
#define HAL_POS_PID_KI    0.0f
#define HAL_POS_PID_KD    0.2f
#define HAL_POS_PID_IMAX  0.5f
#define HAL_POS_PID_OMAX  0.3f   // Max tilt angle command (rad)

// ----------------------------------------------------------------------------
// Bus Configuration
// ----------------------------------------------------------------------------

// STM32F405 has more RAM than F401 (192KB vs 64KB)
#define HAL_BUS_CONFIG { \
    .max_subscribers = 6, \
    .consume_after_reads = 0, \
    .max_age_ms = 0, \
    .max_entries = 1, \
    .max_entry_size = 128 \
}

// ----------------------------------------------------------------------------
// Flow Deck Configuration
// ----------------------------------------------------------------------------

// Flow sensor scaling - converts raw flow to m/s at 1m height
#define HAL_FLOW_SCALE  0.0005f

// ToF sensor max range (mm)
#define HAL_TOF_MAX_RANGE_MM  4000

#endif // HAL_CONFIG_H

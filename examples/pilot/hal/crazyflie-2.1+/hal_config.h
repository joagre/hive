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
#define HAL_BASE_THRUST 0.35f

// ----------------------------------------------------------------------------
// Altitude Control
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
// Tuned in Webots simulation - may need adjustment on hardware
#define HAL_ALT_PID_KP 0.18f   // Reduced from 0.25 to decrease overshoot
#define HAL_ALT_PID_KI 0.03f   // Reduced from 0.05 to decrease integral windup
#define HAL_ALT_PID_KD 0.0f    // Using velocity feedback instead
#define HAL_ALT_PID_IMAX 0.2f  // Integral limit
#define HAL_ALT_PID_OMAX 0.15f // Output limit

// Vertical velocity damping (measured velocity -> thrust correction)
#define HAL_VVEL_DAMPING_GAIN 0.35f // Increased from 0.25 for better damping

// ----------------------------------------------------------------------------
// Attitude Control
// ----------------------------------------------------------------------------

// Attitude PID gains (attitude angle error -> rate setpoint)
#define HAL_ATTITUDE_PID_KP 2.5f
#define HAL_ATTITUDE_PID_KI 0.0f
#define HAL_ATTITUDE_PID_KD 0.15f
#define HAL_ATTITUDE_PID_IMAX 0.5f
#define HAL_ATTITUDE_PID_OMAX 2.0f

// ----------------------------------------------------------------------------
// Rate Control
// ----------------------------------------------------------------------------

// Rate PID gains (rate error -> torque)
#define HAL_RATE_PID_KP 0.015f
#define HAL_RATE_PID_KI 0.0f
#define HAL_RATE_PID_KD 0.002f
#define HAL_RATE_PID_IMAX 0.5f
#define HAL_RATE_PID_OMAX_ROLL 0.08f
#define HAL_RATE_PID_OMAX_PITCH 0.08f
#define HAL_RATE_PID_OMAX_YAW 0.12f

// ----------------------------------------------------------------------------
// Position Control (using Flow Deck)
// ----------------------------------------------------------------------------

// Position PID gains - for optical flow based position hold
#define HAL_POS_PID_KP 0.5f
#define HAL_POS_PID_KI 0.0f
#define HAL_POS_PID_KD 0.2f
#define HAL_POS_PID_IMAX 0.5f
#define HAL_POS_PID_OMAX 0.3f // Max tilt angle command (rad)

// ----------------------------------------------------------------------------
// Flow Deck Configuration
// ----------------------------------------------------------------------------

// Flow sensor scaling - converts raw flow to m/s at 1m height
#define HAL_FLOW_SCALE 0.0005f

// ToF sensor max range (mm) - using short distance mode for indoor flight
#define HAL_TOF_MAX_RANGE_MM 1300

#endif // HAL_CONFIG_H

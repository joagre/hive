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
#define HAL_BASE_THRUST 0.553f

// ----------------------------------------------------------------------------
// Altitude Control
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
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
#define HAL_ATTITUDE_PID_KP 2.5f // Reduced for noise rejection (was 4.0)
#define HAL_ATTITUDE_PID_KI 0.0f
#define HAL_ATTITUDE_PID_KD 0.15f  // Added damping for oscillation reduction
#define HAL_ATTITUDE_PID_IMAX 0.5f // Integral limit
#define HAL_ATTITUDE_PID_OMAX 2.0f // Reduced max rate setpoint (rad/s)

// ----------------------------------------------------------------------------
// Rate Control
// ----------------------------------------------------------------------------

// Rate PID gains (rate error -> torque)
// Tuned for 20ms motor response lag - higher gains compensate for motor delay
#define HAL_RATE_PID_KP 0.028f // Increased to compensate for motor lag
#define HAL_RATE_PID_KI 0.002f // Small integral for steady-state tracking
#define HAL_RATE_PID_KD 0.003f // Predictive damping
#define HAL_RATE_PID_IMAX 0.3f // Integral limit
#define HAL_RATE_PID_OMAX_ROLL \
    0.12f // Increased - motor lag limits actual response
#define HAL_RATE_PID_OMAX_PITCH 0.12f
#define HAL_RATE_PID_OMAX_YAW 0.15f

#endif // HAL_CONFIG_H

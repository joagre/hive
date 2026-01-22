// Crazyflie 2.1+ HAL Configuration
//
// Platform-specific constants for Crazyflie 2.1+ hardware.
//
// All controller gains tuned in Webots with realistic simulation:
// - Flow deck realistic mode (no artificial drift)
// - 20ms motor response lag
//
// Only HAL_BASE_THRUST needs calibration on actual hardware.
// Start at 0.38 and adjust until stable hover is achieved.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// ----------------------------------------------------------------------------
// Thrust
// ----------------------------------------------------------------------------

// Base thrust for hover - MUST BE CALIBRATED on actual hardware
// Start low and increase until hover is achieved
// Typical Crazyflie value: 0.35-0.45 depending on battery
#define HAL_BASE_THRUST 0.38f

// ----------------------------------------------------------------------------
// Altitude Control (tuned in Webots)
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
#define HAL_ALT_PID_KP 0.18f   // Position error gain
#define HAL_ALT_PID_KI 0.03f   // Integral for steady-state
#define HAL_ALT_PID_KD 0.0f    // Using velocity feedback instead
#define HAL_ALT_PID_IMAX 0.2f  // Integral limit
#define HAL_ALT_PID_OMAX 0.15f // Output limit

// Vertical velocity damping (measured velocity -> thrust correction)
#define HAL_VVEL_DAMPING_GAIN 0.35f

// ----------------------------------------------------------------------------
// Attitude Control (tuned in Webots)
// ----------------------------------------------------------------------------

// Attitude PID gains (attitude angle error -> rate setpoint)
#define HAL_ATTITUDE_PID_KP 2.5f   // Angle error gain
#define HAL_ATTITUDE_PID_KI 0.0f   // No integral needed
#define HAL_ATTITUDE_PID_KD 0.15f  // Damping for oscillation reduction
#define HAL_ATTITUDE_PID_IMAX 0.5f // Integral limit
#define HAL_ATTITUDE_PID_OMAX 2.0f // Max rate setpoint (rad/s)

// ----------------------------------------------------------------------------
// Rate Control (tuned for motor response lag)
// ----------------------------------------------------------------------------

// Rate PID gains (rate error -> torque)
// Tuned in Webots with 20ms motor lag simulation - should transfer to hardware
#define HAL_RATE_PID_KP 0.028f       // Compensates for motor lag
#define HAL_RATE_PID_KI 0.002f       // Small integral for steady-state tracking
#define HAL_RATE_PID_KD 0.003f       // Predictive damping
#define HAL_RATE_PID_IMAX 0.3f       // Integral limit
#define HAL_RATE_PID_OMAX_ROLL 0.12f // Motor lag limits actual response
#define HAL_RATE_PID_OMAX_PITCH 0.12f
#define HAL_RATE_PID_OMAX_YAW 0.15f

// ----------------------------------------------------------------------------
// Flow Deck Configuration
// ----------------------------------------------------------------------------

// Flow sensor scaling - converts raw flow to m/s at 1m height
#define HAL_FLOW_SCALE 0.0005f

// ToF sensor max range (mm) - using short distance mode for indoor flight
#define HAL_TOF_MAX_RANGE_MM 1300

#endif // HAL_CONFIG_H

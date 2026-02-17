// Webots Crazyflie HAL Configuration
//
// Platform-specific constants for the Webots Crazyflie simulation.
// Tuned for the Webots Crazyflie model.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// ----------------------------------------------------------------------------
// Altitude Control
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
// KP and damping match original Webots-tuned values (commit 50cdb64).
// Hardware values (KP=0.18, damping=0.35) were tuned for real Crazyflie
// with different thrust dynamics; Webots needs stronger P and weaker damping.
#define HAL_ALT_PID_KP 0.30f  // Original Webots value
#define HAL_ALT_PID_KI 0.03f  // Small integral for steady-state
#define HAL_ALT_PID_KD 0.0f   // Using velocity feedback instead
#define HAL_ALT_PID_IMAX 0.2f // Integral limit
#define HAL_ALT_PID_OMAX \
    0.50f // Larger than hardware (0.15) - 20ms motor lag \
          // needs stronger correction authority

// Vertical velocity damping (measured velocity -> thrust correction)
// Must be weaker than KP to avoid oscillation where damping fights PID
// correction during descent (see altitude overshoot analysis).
#define HAL_VVEL_DAMPING_GAIN 0.15f // Original Webots value

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

// Yaw rate gains - match roll/pitch for simulation. The Webots motor model
// has no CW/CCW torque imbalance, so aggressive yaw gains are unnecessary
// and exceed the gain margin due to the 20ms motor time constant, causing
// yaw instability. Real hardware uses higher gains (see crazyflie hal_config.h).
#define HAL_RATE_YAW_PID_KP 0.028f // Same as roll/pitch - no torque bias in sim
#define HAL_RATE_YAW_PID_KI 0.002f // Same as roll/pitch
#define HAL_RATE_YAW_PID_KD 0.003f // Same as roll/pitch

#endif // HAL_CONFIG_H

// Crazyflie 2.1+ HAL Configuration
//
// Platform-specific constants for Crazyflie 2.1+ hardware.
//
// FIRST FLIGHT GAINS - Conservative values for initial hardware testing.
// These are deliberately sluggish for safety. After successful hover:
//   1. Use ground_station.py to capture telemetry
//   2. Analyze with tools/analyze_pid.py
//   3. Gradually increase gains toward Webots-tuned values
//
// Webots reference values (for comparison):
//   Altitude: Kp=0.18, Ki=0.03, Vvel=0.35
//   Attitude: Kp=2.5, Kd=0.15
//   Rate: Kp=0.015, Kd=0.002
//   Position: Kp=0.12, Kd=0.18

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
// Altitude Control (conservative first-flight)
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
// Reduced ~30% from Webots values for noise tolerance
#define HAL_ALT_PID_KP 0.12f   // Conservative (Webots: 0.18)
#define HAL_ALT_PID_KI 0.0f    // Disabled for first flight (avoid windup)
#define HAL_ALT_PID_KD 0.0f    // Using velocity feedback instead
#define HAL_ALT_PID_IMAX 0.15f // Integral limit (when Ki enabled)
#define HAL_ALT_PID_OMAX 0.10f // Reduced output limit for gentle response

// Vertical velocity damping (measured velocity -> thrust correction)
// Slightly higher for stability with noisy sensors
#define HAL_VVEL_DAMPING_GAIN 0.40f // Conservative (Webots: 0.35)

// ----------------------------------------------------------------------------
// Attitude Control (conservative first-flight)
// ----------------------------------------------------------------------------

// Attitude PID gains (attitude angle error -> rate setpoint)
// Reduced for noise rejection on real IMU
#define HAL_ATTITUDE_PID_KP 1.8f   // Conservative (Webots: 2.5)
#define HAL_ATTITUDE_PID_KI 0.0f   // Disabled for first flight
#define HAL_ATTITUDE_PID_KD 0.08f  // Reduced D (noise sensitive)
#define HAL_ATTITUDE_PID_IMAX 0.3f // Integral limit (when Ki enabled)
#define HAL_ATTITUDE_PID_OMAX 1.5f // Reduced max rate setpoint (rad/s)

// ----------------------------------------------------------------------------
// Rate Control (conservative first-flight)
// ----------------------------------------------------------------------------

// Rate PID gains (rate error -> torque)
// Innermost loop - most sensitive to noise, be conservative
#define HAL_RATE_PID_KP 0.010f       // Conservative (Webots: 0.015)
#define HAL_RATE_PID_KI 0.0f         // Disabled for first flight
#define HAL_RATE_PID_KD 0.001f       // Reduced D (noise sensitive)
#define HAL_RATE_PID_IMAX 0.3f       // Integral limit (when Ki enabled)
#define HAL_RATE_PID_OMAX_ROLL 0.06f // Reduced for smoother control
#define HAL_RATE_PID_OMAX_PITCH 0.06f
#define HAL_RATE_PID_OMAX_YAW 0.08f

// ----------------------------------------------------------------------------
// Flow Deck Configuration
// ----------------------------------------------------------------------------

// Flow sensor scaling - converts raw flow to m/s at 1m height
#define HAL_FLOW_SCALE 0.0005f

// ToF sensor max range (mm) - using short distance mode for indoor flight
#define HAL_TOF_MAX_RANGE_MM 1300

#endif // HAL_CONFIG_H

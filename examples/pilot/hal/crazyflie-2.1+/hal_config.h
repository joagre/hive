// Crazyflie 2.1+ HAL Configuration
//
// Platform-specific constants for Crazyflie 2.1+ hardware.
//
// Rate controller gains are tuned in Webots with realistic motor lag (20ms).
// Altitude and attitude controllers remain conservative for first flights.
//
// After successful hover, tune remaining controllers using:
//   1. tools/ground_station.py to capture telemetry
//   2. tools/analyze_pid.py to analyze performance
//   3. Gradually increase gains toward Webots-tuned values
//
// Webots reference values (tuned with motor lag):
//   Altitude: Kp=0.18, Ki=0.03, Vvel=0.35
//   Attitude: Kp=2.5, Kd=0.15
//   Rate: Kp=0.028, Ki=0.002, Kd=0.003 (motor lag compensated)

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

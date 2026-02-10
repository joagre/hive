// Crazyflie 2.1+ HAL Configuration

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// Calibrated hover thrust (normalized 0.0-1.0)
// Measured with thrust test - drone light on skids at this value
#define HAL_HOVER_THRUST 0.38f

/*
 * Motor mapping (Bitcraze standard, viewed from above):
 *
 *        Front
 *    M4(CW)     M1(CCW)
 *         \   /
 *          x
 *         /   \
 *    M3(CCW)   M2(CW)
 *        Back
 */
// M1 (front-right, CCW): TIM2_CH2 (PA1)
// M2 (back-right, CW):   TIM2_CH4 (PB11)
// M3 (back-left, CCW):   TIM2_CH1 (PA15)
// M4 (front-left, CW):   TIM4_CH4 (PB9)

// Sensor update rate (Hz)
#define HAL_SENSOR_RATE 1000

// Enable radio (syslink to nRF51)
#define HAL_HAS_RADIO 1

// ----------------------------------------------------------------------------
// Altitude Control (conservative for first flights)
// ----------------------------------------------------------------------------

// Altitude PID gains (position error -> thrust correction)
#define HAL_ALT_PID_KP 0.12f   // Conservative (Webots: 0.18)
#define HAL_ALT_PID_KI 0.01f   // Conservative (Webots: 0.03)
#define HAL_ALT_PID_KD 0.0f    // Using velocity feedback instead
#define HAL_ALT_PID_IMAX 0.2f  // Integral limit
#define HAL_ALT_PID_OMAX 0.15f // Output limit

// Vertical velocity damping (measured velocity -> thrust correction)
#define HAL_VVEL_DAMPING_GAIN 0.25f // Conservative (Webots: 0.35)

// ----------------------------------------------------------------------------
// Attitude Control (tuned - between old weak and new oscillating values)
// ----------------------------------------------------------------------------

// Attitude PID gains (attitude angle error -> rate setpoint)
#define HAL_ATTITUDE_PID_KP 1.8f // Tuned (was 1.5 weak, 2.2 oscillated)
#define HAL_ATTITUDE_PID_KI 0.0f
#define HAL_ATTITUDE_PID_KD 0.10f  // Tuned (was 0.08 weak, 0.12 oscillated)
#define HAL_ATTITUDE_PID_IMAX 0.5f // Integral limit
#define HAL_ATTITUDE_PID_OMAX 2.0f // Max rate setpoint (rad/s)

// ----------------------------------------------------------------------------
// Rate Control (tuned - between old weak and new oscillating values)
// ----------------------------------------------------------------------------

// Rate PID gains (rate error -> torque)
#define HAL_RATE_PID_KP 0.020f // Tuned (was 0.018 weak, 0.024 oscillated)
#define HAL_RATE_PID_KI \
    0.001f // Back to original (integral can cause overshoot)
#define HAL_RATE_PID_KD 0.0015f // Tuned (was 0.001 weak, 0.002 oscillated)
#define HAL_RATE_PID_IMAX 0.3f  // Integral limit
#define HAL_RATE_PID_OMAX_ROLL 0.12f
#define HAL_RATE_PID_OMAX_PITCH 0.12f
#define HAL_RATE_PID_OMAX_YAW 0.15f

// ----------------------------------------------------------------------------
// Flow Deck Configuration
// ----------------------------------------------------------------------------

// Flow sensor scaling - converts raw flow to m/s at 1m height
// PMW3901 outputs pixel deltas; this converts to velocity
#define HAL_FLOW_SCALE 0.0005f

// ToF sensor max range (mm) - readings above this are considered invalid
// VL53L1x in short distance mode is reliable up to ~1.3m
#define HAL_TOF_MAX_RANGE_MM 1300

#endif // HAL_CONFIG_H

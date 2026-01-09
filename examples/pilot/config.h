// Configuration constants for pilot example
//
// Shared constants used across multiple actors.

#ifndef PILOT_CONFIG_H
#define PILOT_CONFIG_H

// ----------------------------------------------------------------------------
// Math utilities
// ----------------------------------------------------------------------------

#define CLAMPF(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

// Low-pass filter: LPF(state, new_sample, alpha)
// alpha=0: instant response, alpha=1: no response
#define LPF(state, sample, alpha) ((alpha) * (state) + (1.0f - (alpha)) * (sample))

#define RAD_TO_DEG  57.2957795f  // 180/π
#define DEG_TO_RAD  0.0174533f   // π/180

// ----------------------------------------------------------------------------
// Hardware configuration
// ----------------------------------------------------------------------------

#define NUM_MOTORS  4

// Motor velocity limits (rad/s)
#define MOTOR_MAX_VELOCITY  100.0f

// ----------------------------------------------------------------------------
// Timing
// ----------------------------------------------------------------------------

#define TIME_STEP_MS  4        // Control loop period (milliseconds)
#define TIME_STEP_S   0.004f   // Control loop period (seconds)

#define DEBUG_PRINT_INTERVAL  250  // Print every N iterations (250 = 1 second at 250Hz)

// Motor watchdog timeout in iterations (~200ms at 250Hz)
#define MOTOR_WATCHDOG_TIMEOUT  50

// ----------------------------------------------------------------------------
// Estimator parameters
// ----------------------------------------------------------------------------

// Low-pass filter coefficient for vertical velocity (0.0 to 1.0)
// Higher = more smoothing, slower response
// Lower = less smoothing, more noise
#define VVEL_FILTER_ALPHA  0.8f

// ----------------------------------------------------------------------------
// Control parameters (tuned for Webots Crazyflie)
// ----------------------------------------------------------------------------

// Altitude control
#define TARGET_ALTITUDE  1.0f    // meters
#define BASE_THRUST      0.553f  // Hover thrust

// Altitude PID gains (position error → thrust correction)
#define ALT_PID_KP   0.3f
#define ALT_PID_KI   0.05f
#define ALT_PID_KD   0.0f    // Using velocity feedback instead
#define ALT_PID_IMAX 0.2f    // Integral limit
#define ALT_PID_OMAX 0.15f   // Output limit

// Vertical velocity damping (measured velocity → thrust correction)
// Provides smoother response than differentiating position error
#define VVEL_DAMPING_GAIN  0.15f

// Angle PID gains (angle error → rate setpoint)
#define ANGLE_PID_KP   4.0f
#define ANGLE_PID_KI   0.0f
#define ANGLE_PID_KD   0.1f
#define ANGLE_PID_OMAX 2.0f  // Max rate setpoint (rad/s)

// Rate PID gains (rate error → torque)
#define RATE_PID_KP          0.02f
#define RATE_PID_KI          0.0f
#define RATE_PID_KD          0.001f
#define RATE_ROLL_PID_OMAX   0.1f
#define RATE_PITCH_PID_OMAX  0.1f
#define RATE_YAW_PID_OMAX    0.15f

#endif // PILOT_CONFIG_H

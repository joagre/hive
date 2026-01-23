// Configuration constants for pilot example
//
// Shared constants used across multiple actors.
// Platform-specific tuning parameters are in hal_config.h.
// Math utilities are in math_utils.h.

#ifndef PILOT_CONFIG_H
#define PILOT_CONFIG_H

// ----------------------------------------------------------------------------
// Flight Profiles
// ----------------------------------------------------------------------------
// Select profile with -DFLIGHT_PROFILE=X (see Makefile)

#define FLIGHT_PROFILE_FIRST_TEST 1 // First flight test (hover, land)
#define FLIGHT_PROFILE_ALTITUDE 2   // Altitude-only waypoints
#define FLIGHT_PROFILE_FULL_3D 3    // Full 3D waypoint navigation

// Auto-select default profile based on platform if not specified
#ifndef FLIGHT_PROFILE
#define FLIGHT_PROFILE FLIGHT_PROFILE_FULL_3D
#endif

// ----------------------------------------------------------------------------
// Hardware configuration
// ----------------------------------------------------------------------------

#define NUM_MOTORS 4

// Bus configuration (same for all platforms)
#define HAL_BUS_CONFIG                                                   \
    {                                                                    \
        .max_subscribers = 6, .consume_after_reads = 0, .max_age_ms = 0, \
        .max_entries = 1, .max_entry_size = 128                          \
    }

// Motor velocity limits (rad/s)
#define MOTOR_MAX_VELOCITY 100.0f

// ----------------------------------------------------------------------------
// Timing
// ----------------------------------------------------------------------------

#define TIME_STEP_MS 4 // Control loop period (milliseconds)
// Note: Actors measure actual dt using hive_get_time(), not a fixed timestep

// Motor deadman timeout - if no torque command received within this time,
// motors are zeroed. Should be several control cycles to tolerate jitter.
// 50ms = ~12 control cycles at 250Hz, provides margin while still being safe.
#define MOTOR_DEADMAN_TIMEOUT_MS 50

#define DEBUG_PRINT_INTERVAL \
    250 // Print every N iterations (250 = 1 second at 250Hz)

// ----------------------------------------------------------------------------
// Estimator parameters
// ----------------------------------------------------------------------------

// Enable Kalman filter for altitude estimation (1 = KF, 0 = simple LPF)
// KF provides better velocity estimates and handles sensor noise optimally
#ifndef USE_ALTITUDE_KF
#define USE_ALTITUDE_KF 1
#endif

// Altitude Kalman filter parameters
// Tune these based on sensor characteristics and flight test analysis
// Process noise (Q matrix diagonal) - how much we trust the model
#define ALT_KF_Q_ALTITUDE 0.0001f // Position process noise (m^2)
#define ALT_KF_Q_VELOCITY \
    1.0f // Velocity process noise (m^2/s^2) - HIGH to track fast
#define ALT_KF_Q_BIAS 0.0001f // Accel bias random walk (m^2/s^4)

// Measurement noise (R) - how much we trust the rangefinder/baro
// Set to match actual sensor noise variance: (0.03m)^2 = 0.0009
#define ALT_KF_R_ALTITUDE 0.001f // Rangefinder noise (m^2) - realistic noise

// Initial uncertainty (P0 diagonal)
#define ALT_KF_P0_ALTITUDE 1.0f // Initial altitude uncertainty (m^2)
#define ALT_KF_P0_VELOCITY 1.0f // Initial velocity uncertainty (m^2/s^2)
#define ALT_KF_P0_BIAS 0.1f     // Initial bias uncertainty (m^2/s^4)

// Low-pass filter coefficient for vertical velocity (0.0 to 1.0)
// Used when USE_ALTITUDE_KF=0, or for horizontal velocity
// Higher = more smoothing, slower response
// Lower = less smoothing, more noise
#define VVEL_FILTER_ALPHA 0.8f

// Low-pass filter coefficient for horizontal velocity
// Higher alpha needed with noisy optical flow/GPS position data
#define HVEL_FILTER_ALPHA 0.95f

// ----------------------------------------------------------------------------
// Safety thresholds (altitude_actor emergency detection)
// ----------------------------------------------------------------------------

#define EMERGENCY_TILT_LIMIT 0.78f  // ~45 degrees in radians
#define EMERGENCY_ALTITUDE_MAX 2.0f // meters - cut motors if exceeded
#define LANDED_TARGET_THRESHOLD \
    0.05f // meters - target altitude indicating land command
#define LANDED_ACTUAL_THRESHOLD \
    0.08f // meters - actual altitude confirming landed (tight!)

// ----------------------------------------------------------------------------
// Waypoint navigation (mission parameters)
// ----------------------------------------------------------------------------

#define WAYPOINT_TOLERANCE_XY 0.15f // meters - horizontal arrival radius
#define WAYPOINT_TOLERANCE_Z \
    0.08f // meters - altitude tolerance (tight for landing)
#define WAYPOINT_TOLERANCE_YAW 0.1f  // radians (~6 degrees)
#define WAYPOINT_TOLERANCE_VEL 0.05f // m/s - must be nearly stopped

// ----------------------------------------------------------------------------
// Position control
// ----------------------------------------------------------------------------

#ifdef SIMULATED_TIME
// Webots with sensor noise - reduced gains for stability
// D term especially sensitive to noisy velocity estimates
#define POS_KP 0.08f
#define POS_KD 0.06f         // Low D - noisy velocity
#define MAX_TILT_ANGLE 0.20f // ~11 degrees - limit for noise tolerance
#else
// Conservative first-flight values for real hardware
// Optical flow is noisy; start sluggish and tune up gradually
#define POS_KP 0.08f         // Conservative (Webots: 0.12)
#define POS_KD 0.10f         // Reduced D for noise (Webots: 0.18)
#define MAX_TILT_ANGLE 0.25f // ~14 degrees (safer limit)
#endif

// ----------------------------------------------------------------------------
// Platform-specific control parameters
// ----------------------------------------------------------------------------
// The following are defined in hal_config.h (platform-specific):
//
// Thrust:
//   HAL_BASE_THRUST
//
// Altitude control:
//   HAL_ALT_PID_KP, HAL_ALT_PID_KI, HAL_ALT_PID_KD
//   HAL_ALT_PID_IMAX, HAL_ALT_PID_OMAX
//   HAL_VVEL_DAMPING_GAIN
//
// Attitude control:
//   HAL_ATTITUDE_PID_KP, HAL_ATTITUDE_PID_KI, HAL_ATTITUDE_PID_KD
//   HAL_ATTITUDE_PID_IMAX, HAL_ATTITUDE_PID_OMAX
//
// Rate control:
//   HAL_RATE_PID_KP, HAL_RATE_PID_KI, HAL_RATE_PID_KD
//   HAL_RATE_PID_IMAX
//   HAL_RATE_PID_OMAX_ROLL, HAL_RATE_PID_OMAX_PITCH, HAL_RATE_PID_OMAX_YAW

#endif // PILOT_CONFIG_H

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

#define FLIGHT_PROFILE_FIRST_TEST  1  // First flight test (hover, land)
#define FLIGHT_PROFILE_ALTITUDE    2  // Altitude-only waypoints
#define FLIGHT_PROFILE_FULL_3D     3  // Full 3D waypoint navigation

// Auto-select default profile based on platform if not specified
#ifndef FLIGHT_PROFILE
  #ifdef PLATFORM_STEVAL_DRONE01
    #define FLIGHT_PROFILE FLIGHT_PROFILE_FIRST_TEST
  #else
    #define FLIGHT_PROFILE FLIGHT_PROFILE_FULL_3D
  #endif
#endif

// ----------------------------------------------------------------------------
// Hardware configuration
// ----------------------------------------------------------------------------

#define NUM_MOTORS  4

// Bus configuration (same for all platforms)
#define HAL_BUS_CONFIG { \
    .max_subscribers = 6, \
    .consume_after_reads = 0, \
    .max_age_ms = 0, \
    .max_entries = 1, \
    .max_entry_size = 128 \
}

// Motor velocity limits (rad/s)
#define MOTOR_MAX_VELOCITY  100.0f

// ----------------------------------------------------------------------------
// Timing
// ----------------------------------------------------------------------------

#define TIME_STEP_MS  4        // Control loop period (milliseconds)
// Note: Actors measure actual dt using hive_get_time(), not a fixed timestep

#define DEBUG_PRINT_INTERVAL  250  // Print every N iterations (250 = 1 second at 250Hz)

// ----------------------------------------------------------------------------
// Estimator parameters
// ----------------------------------------------------------------------------

// Low-pass filter coefficient for vertical velocity (0.0 to 1.0)
// Higher = more smoothing, slower response
// Lower = less smoothing, more noise
#define VVEL_FILTER_ALPHA  0.8f

// Low-pass filter coefficient for horizontal velocity
#define HVEL_FILTER_ALPHA  0.8f

// ----------------------------------------------------------------------------
// Safety thresholds (altitude_actor emergency detection)
// ----------------------------------------------------------------------------

#define EMERGENCY_TILT_LIMIT     0.78f   // ~45 degrees in radians
#define EMERGENCY_ALTITUDE_MAX   2.0f    // meters - cut motors if exceeded
#define LANDED_TARGET_THRESHOLD  0.05f   // meters - target altitude indicating land command
#define LANDED_ACTUAL_THRESHOLD  0.08f   // meters - actual altitude confirming landed (tight!)

// ----------------------------------------------------------------------------
// Waypoint navigation (mission parameters)
// ----------------------------------------------------------------------------

#define WAYPOINT_TOLERANCE_XY   0.15f  // meters - horizontal arrival radius
#define WAYPOINT_TOLERANCE_Z    0.08f  // meters - altitude tolerance (tight for landing)
#define WAYPOINT_TOLERANCE_YAW  0.1f   // radians (~6 degrees)
#define WAYPOINT_TOLERANCE_VEL  0.05f  // m/s - must be nearly stopped

// ----------------------------------------------------------------------------
// Position control (mission parameters - no GPS on STEVAL anyway)
// ----------------------------------------------------------------------------

#define POS_KP           0.2f    // Position gain: rad per meter error
#define POS_KD           0.1f    // Velocity damping: rad per m/s

// Maximum tilt angle for position control (safety limit)
#define MAX_TILT_ANGLE   0.35f   // ~20 degrees

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

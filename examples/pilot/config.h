// Configuration constants for pilot example
//
// Shared constants used across multiple actors.
// Platform-specific tuning parameters are in hal_config.h.

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
#define M_PI_F      3.14159265f  // π as float

// Normalize angle to [-π, π] range
#define NORMALIZE_ANGLE(a) ({ \
    float _a = (a); \
    while (_a > M_PI_F) _a -= 2.0f * M_PI_F; \
    while (_a < -M_PI_F) _a += 2.0f * M_PI_F; \
    _a; \
})

// Initialize roll/pitch/yaw PIDs with same gains (common pattern)
#define PID_INIT_RPY(roll, pitch, yaw, kp, ki, kd, imax, omax) do { \
    pid_init_full(&(roll),  (kp), (ki), (kd), (imax), (omax)); \
    pid_init_full(&(pitch), (kp), (ki), (kd), (imax), (omax)); \
    pid_init_full(&(yaw),   (kp), (ki), (kd), (imax), (omax)); \
} while (0)

// Read latest value from bus into destination variable.
// Returns true if read succeeded, false otherwise.
// Usage: if (BUS_READ(bus, &var)) { /* use var */ }
#define BUS_READ(bus, dest_ptr) ({ \
    size_t _len; \
    hive_bus_read((bus), (dest_ptr), sizeof(*(dest_ptr)), &_len).code == HIVE_OK; \
})

// Blocking bus read - waits forever for data
// Usage: BUS_READ_WAIT(bus, &var);
#define BUS_READ_WAIT(bus, dest_ptr) do { \
    size_t _len; \
    hive_bus_read_wait((bus), (dest_ptr), sizeof(*(dest_ptr)), &_len, -1); \
} while(0)

// Subscribe to bus with assert (common pattern in actors)
// Usage: BUS_SUBSCRIBE(s_state_bus);
#define BUS_SUBSCRIBE(bus) \
    assert(HIVE_SUCCEEDED(hive_bus_subscribe(bus)))

// Debug throttle: returns true every N calls
// Usage: int count = 0; ... if (DEBUG_THROTTLE(count, interval)) { log(...); }
#define DEBUG_THROTTLE(counter, interval) (++(counter) % (interval) == 0)

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
// Waypoint navigation (mission parameters)
// ----------------------------------------------------------------------------

#define WAYPOINT_TOLERANCE_XY   0.15f  // meters - horizontal arrival radius
#define WAYPOINT_TOLERANCE_Z    0.15f  // meters - altitude tolerance
#define WAYPOINT_TOLERANCE_YAW  0.1f   // radians (~6 degrees)
#define WAYPOINT_TOLERANCE_VEL  0.1f   // m/s - must be nearly stopped

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
//
// Waypoint:
//   HAL_WAYPOINT_HOVER_TICKS

#endif // PILOT_CONFIG_H

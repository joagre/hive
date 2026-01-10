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
// Control parameters (tuned for Webots Crazyflie)
// ----------------------------------------------------------------------------

// Altitude control (target altitude comes from waypoint actor)
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

// Waypoint navigation
#define WAYPOINT_TOLERANCE_XY   0.15f  // meters - horizontal arrival radius
#define WAYPOINT_TOLERANCE_Z    0.15f  // meters - altitude tolerance
#define WAYPOINT_TOLERANCE_YAW  0.1f   // radians (~6 degrees)
#define WAYPOINT_TOLERANCE_VEL  0.1f   // m/s - must be nearly stopped
#define WAYPOINT_HOVER_TICKS    50     // iterations to hover before advancing (~200ms)

// Position PD gains (simple PD controller)
#define POS_KP           0.2f    // Position gain: rad per meter error
#define POS_KD           0.1f    // Velocity damping: rad per m/s

// Maximum tilt angle for position control (safety limit)
#define MAX_TILT_ANGLE   0.35f   // ~20 degrees

// Angle PID gains (angle error → rate setpoint)
#define ANGLE_PID_KP   4.0f     // Typical autopilot: 4-8
#define ANGLE_PID_KI   0.0f
#define ANGLE_PID_KD   0.0f     // Derivative on error causes kick on setpoint change
#define ANGLE_PID_IMAX 0.5f     // Integral limit
#define ANGLE_PID_OMAX 3.0f     // Max rate setpoint (rad/s)

// Rate PID gains (rate error → torque)
#define RATE_PID_KP   0.02f
#define RATE_PID_KI   0.0f
#define RATE_PID_KD   0.001f
#define RATE_PID_IMAX 0.5f      // Integral limit
#define RATE_PID_OMAX_ROLL   0.1f
#define RATE_PID_OMAX_PITCH  0.1f
#define RATE_PID_OMAX_YAW    0.15f

#endif // PILOT_CONFIG_H

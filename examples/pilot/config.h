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

#define FLIGHT_PROFILE_GROUND_TEST 0 // Ground test (motors disabled, ESB only)
#define FLIGHT_PROFILE_FIRST_TEST 1  // First flight test (hover, land)
#define FLIGHT_PROFILE_ALTITUDE 2    // Altitude-only waypoints
#define FLIGHT_PROFILE_FULL_3D 3     // Full 3D waypoint navigation

// Auto-select default profile based on platform if not specified
#ifndef FLIGHT_PROFILE
#define FLIGHT_PROFILE FLIGHT_PROFILE_FULL_3D
#endif

// ----------------------------------------------------------------------------
// Hardware configuration
// ----------------------------------------------------------------------------

#define NUM_MOTORS 4

// Bus configuration (same for all platforms)
// max_subscribers=8: state_bus has 7 subscribers when comms+logger both active
#define HAL_BUS_CONFIG                                                   \
    {                                                                    \
        .max_subscribers = 8, .consume_after_reads = 0, .max_age_ms = 0, \
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

// Low-pass filter coefficient for horizontal velocity
// Higher alpha needed with noisy optical flow/GPS position data
#define HVEL_FILTER_ALPHA 0.95f

// ----------------------------------------------------------------------------
// Liftoff detection (altitude_actor)
// ----------------------------------------------------------------------------

// Instead of a fixed hover thrust, ramp thrust linearly until the rangefinder
// detects liftoff. The thrust at liftoff is the hover thrust for this battery
// state. Self-calibrates every flight.
#ifndef LIFTOFF_RAMP_RATE
#define LIFTOFF_RAMP_RATE 0.4f // thrust/second during detection ramp
#endif
#ifndef LIFTOFF_ALT_THRESHOLD
#define LIFTOFF_ALT_THRESHOLD 0.05f // meters - altitude indicating liftoff
#endif
#define LIFTOFF_MAX_THRUST 0.95f // safety cap during ramp

// After liftoff, the altitude target ramps from current altitude toward the
// waypoint target at this rate. Prevents step-change overshoot by giving
// the PID a smooth reference to track at full authority.
#define LIFTOFF_CLIMB_RATE 0.3f // m/s - climb rate after liftoff detection

// Position control authority scales linearly from 0 to 1 between the
// liftoff threshold and this height above it. Below threshold: no tilt.
// Above threshold + transition: full authority. Tied to the altitude
// where flow deck velocity estimates become reliable (~10-15cm AGL).
#define POSITION_AUTHORITY_TRANSITION 0.15f // meters above liftoff threshold

// ----------------------------------------------------------------------------
// Safety thresholds (altitude_actor emergency detection)
// ----------------------------------------------------------------------------

#define EMERGENCY_TILT_LIMIT 0.78f  // ~45 degrees in radians
#define EMERGENCY_ALTITUDE_MAX 2.0f // meters - cut motors if exceeded

// Minimum thrust when airborne. Keeps enough base thrust for the mixer
// to produce some attitude corrections. Without this, the altitude PID
// can drive thrust to zero during overshoot, leaving the attitude PIDs
// with no motor authority (motors off = no differential possible).
// Low value (0.05) preserves downward correction authority while still
// providing enough base for partial roll/pitch differentials.
#define MIN_AIRBORNE_THRUST 0.05f
#define LANDED_TARGET_THRESHOLD \
    0.05f // meters - target altitude indicating land command
#define LANDED_ACTUAL_THRESHOLD \
    0.08f // meters - actual altitude confirming landed
// Touchdown requires near-zero vertical velocity. Threshold set above
// the rangefinder noise floor (~0.03 m/s) with margin for KF lag.
#define LANDED_VELOCITY_THRESHOLD 0.1f // m/s

// ----------------------------------------------------------------------------
// Rangefinder (VL53L1x) characteristics
// ----------------------------------------------------------------------------
// From VL53L1x datasheet and observed behavior on Crazyflie flow deck.

// Minimum measurable distance. Below this the sensor returns 0.
// Drone chassis sits ~20-30mm above ground, so readings below this
// threshold on the ground are noise.
#define RANGEFINDER_MIN_M 0.01f // 10mm - VL53L1x minimum range

// Maximum reliable range in short-distance mode (configured at 40Hz).
// Beyond this the sensor returns invalid readings or times out.
// Overridable by HAL: Webots GPS has no physical range limit.
#ifndef RANGEFINDER_MAX_M
#define RANGEFINDER_MAX_M 1.3f // 1300mm - VL53L1x short mode max
#endif

// Innovation gating for altitude Kalman filter. Reject rangefinder
// measurements where |measured - predicted| exceeds this threshold.
// Prevents single spurious readings from corrupting the KF state and
// velocity estimate (which then causes violent thrust oscillations
// via the velocity damping term). At 3 m/s vertical and 40Hz
// rangefinder rate (25ms), real change is at most 0.075m. 0.3m gives
// generous margin while catching the >0.5m glitches from flight test 32.
#define KF_MAX_INNOVATION 0.3f

// Altitude below which the drone is considered ground-level.
// Used for drift prevention: when the last valid reading was below
// this and the rangefinder goes silent (below its minimum range),
// we inject a ground-level measurement to prevent KF drift.
// Set above chassis height (~30mm) with margin.
#define RANGEFINDER_GROUND_ALT_M 0.1f // 100mm

// How long to wait before injecting ground-level correction.
// The rangefinder updates at 40Hz (25ms). If no valid reading for
// 500ms (20 missed samples), the drone is likely on the ground
// below the sensor's minimum range.
#define RANGEFINDER_GROUND_TIMEOUT_US \
    500000 // 500ms = 20 missed samples at 40Hz

// ----------------------------------------------------------------------------
// Motor validation (motor_actor sanity checks)
// ----------------------------------------------------------------------------
// Last line of defense - values outside these indicate control failure.
// Derived from motor operating envelope, not control tuning.

// Thrust can slightly exceed 1.0 due to floating-point PID output.
// Allow headroom so minor overshoot isn't falsely rejected.
#define MOTOR_MAX_THRUST 1.5f

// Maximum torque magnitude for roll/pitch/yaw axes.
// Bounded by motor differential thrust capability. At max tilt
// the mixer produces ~1.5 on the torque axes; 2.0 provides margin.
#define MOTOR_MAX_TORQUE 2.0f

// Tolerance for small negative thrust from floating-point jitter.
// The PID can output slightly below zero; this is harmless and
// clamped to 0 by the mixer. Larger negatives indicate failure.
#define MOTOR_THRUST_NEGATIVE_TOLERANCE 0.1f

// Threshold for detecting first non-zero thrust (takeoff moment).
// Set above the PID output noise floor for clean detection.
#define MOTOR_ENGAGED_THRESHOLD 0.01f

// ----------------------------------------------------------------------------
// Waypoint navigation (mission parameters)
// ----------------------------------------------------------------------------

#define WAYPOINT_TOLERANCE_XY 0.15f // meters - horizontal arrival radius
#define WAYPOINT_TOLERANCE_Z \
    0.08f // meters - altitude tolerance (tight for landing)
#define WAYPOINT_TOLERANCE_YAW 0.1f  // radians (~6 degrees)
#define WAYPOINT_TOLERANCE_VEL 0.05f // m/s - must be nearly stopped

// ----------------------------------------------------------------------------
// Flight manager timing
// ----------------------------------------------------------------------------

// Flight duration per profile. These are safety limits - the waypoint actor
// controls actual flight behavior. Duration should exceed expected mission
// time with margin for position settling.
#if FLIGHT_PROFILE == FLIGHT_PROFILE_FIRST_TEST
#define FLIGHT_DURATION_S 6 // Short for safety during initial testing
#elif FLIGHT_PROFILE == FLIGHT_PROFILE_ALTITUDE
#define FLIGHT_DURATION_S 40 // ~8 waypoints at ~5s each
#elif FLIGHT_PROFILE == FLIGHT_PROFILE_FULL_3D
#define FLIGHT_DURATION_S 60 // ~10 waypoints at ~5s each + margin
#else
#define FLIGHT_DURATION_S 20 // Default
#endif
#define FLIGHT_DURATION_US ((uint64_t)FLIGHT_DURATION_S * 1000000)

// Landing timeout. Derived from worst case: maximum altitude (2m) divided
// by minimum descent rate (0.2 m/s) = 10s, plus margin for ground effect.
#define LANDING_TIMEOUT_S 10
#define LANDING_TIMEOUT_US ((uint64_t)LANDING_TIMEOUT_S * 1000000)

// ----------------------------------------------------------------------------
// Telemetry and logging
// ----------------------------------------------------------------------------

#define TELEMETRY_LOG_RATE_HZ 25 // CSV telemetry logging rate

// Hive log sync interval. Syncing flushes the ring buffer to flash/disk.
// 4 seconds balances data safety with flash write wear.
#define HIVE_LOG_SYNC_SAMPLES (TELEMETRY_LOG_RATE_HZ * 4) // 100 samples = 4s

// CSV sync interval. More frequent than hive log because CSV data is
// the primary flight record. 1 second keeps at most 25 rows at risk.
#define CSV_SYNC_SAMPLES TELEMETRY_LOG_RATE_HZ // 25 samples = 1s

// Comms actor trace logging interval. Produces ~1 log line per second
// at the typical ~500Hz ESB poll rate.
#define COMMS_TRACE_INTERVAL 500

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
// Flight-tested gains - flow deck tracks well, tighter position hold
#define POS_KP 0.14f
#define POS_KD 0.16f
#define MAX_TILT_ANGLE 0.25f // ~14 degrees
#endif

// ----------------------------------------------------------------------------
// Platform-specific control parameters
// ----------------------------------------------------------------------------
// The following are defined in hal_config.h (platform-specific):
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

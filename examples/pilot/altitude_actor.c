// Altitude actor - Altitude hold control
//
// Subscribes to state and position target buses, runs altitude PI with velocity damping,
// publishes thrust commands.
//
// Uses measured vertical velocity for damping instead of differentiating
// the position error. This provides smoother response with less noise.

#include "altitude_actor.h"
#include "types.h"
#include "config.h"
#include "hal_config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_log.h"
#include <assert.h>
#include <math.h>

static bus_id s_state_bus;
static bus_id s_thrust_bus;
static bus_id s_position_target_bus;

void altitude_actor_init(bus_id state_bus, bus_id thrust_bus, bus_id position_target_bus) {
    s_state_bus = state_bus;
    s_thrust_bus = thrust_bus;
    s_position_target_bus = position_target_bus;
}

// Thrust ramp duration for gentle takeoff (microseconds)
#define THRUST_RAMP_DURATION_US  (2 * 1000000)  // 2 seconds

void altitude_actor(void *arg) {
    (void)arg;

    BUS_SUBSCRIBE(s_state_bus);
    BUS_SUBSCRIBE(s_position_target_bus);

    pid_state_t alt_pid;
    pid_init_full(&alt_pid, HAL_ALT_PID_KP, HAL_ALT_PID_KI, HAL_ALT_PID_KD, HAL_ALT_PID_IMAX, HAL_ALT_PID_OMAX);

    // Target altitude (updated from waypoint actor)
    float target_altitude = 0.0f;  // Default to ground (safe)
    uint64_t ramp_start_time = 0;  // For thrust ramp
    int count = 0;

    while (1) {
        state_estimate_t state;
        position_target_t target;

        // Block until state available
        BUS_READ_WAIT(s_state_bus, &state);

        // Read target altitude (non-blocking, use last known if not available)
        if (BUS_READ(s_position_target_bus, &target)) {
            target_altitude = target.z;
        }

        // Emergency cutoff conditions
        bool attitude_emergency = (fabsf(state.roll) > EMERGENCY_TILT_LIMIT) ||
                                  (fabsf(state.pitch) > EMERGENCY_TILT_LIMIT);
        bool altitude_emergency = (state.altitude > EMERGENCY_ALTITUDE_MAX);
        bool landed = (target_altitude < LANDED_TARGET_THRESHOLD) &&
                      (state.altitude < LANDED_ACTUAL_THRESHOLD);

        bool cutoff = attitude_emergency || altitude_emergency || landed;

        float thrust;
        if (cutoff) {
            // Cut motors: emergency or landed
            thrust = 0.0f;
            pid_reset(&alt_pid);  // Reset integrator for next takeoff
            ramp_start_time = 0;  // Reset ramp for next takeoff
        } else {
            // Start ramp timer on first non-cutoff iteration
            if (ramp_start_time == 0) {
                ramp_start_time = hive_get_time();
            }

            // Position control (PI)
            float pos_correction = pid_update(&alt_pid, target_altitude, state.altitude, TIME_STEP_S);

            // Velocity damping: reduce thrust when moving up, increase when moving down
            float vel_damping = -HAL_VVEL_DAMPING_GAIN * state.vertical_velocity;

            // Thrust ramp for gentle takeoff
            uint64_t elapsed_us = hive_get_time() - ramp_start_time;
            float ramp = (elapsed_us < THRUST_RAMP_DURATION_US)
                       ? (float)elapsed_us / THRUST_RAMP_DURATION_US
                       : 1.0f;

            thrust = ramp * CLAMPF(HAL_BASE_THRUST + pos_correction + vel_damping, 0.0f, 1.0f);
        }

        thrust_cmd_t cmd = {.thrust = thrust};
        hive_bus_publish(s_thrust_bus, &cmd, sizeof(cmd));

        if (DEBUG_THROTTLE(count, DEBUG_PRINT_INTERVAL)) {
            HIVE_LOG_DEBUG("[ALT] tgt=%.2f alt=%.2f vvel=%.2f thrust=%.3f",
                           target_altitude, state.altitude, state.vertical_velocity, thrust);
        }
    }
}

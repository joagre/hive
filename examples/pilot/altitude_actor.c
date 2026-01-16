// Altitude actor - Altitude hold control with controlled landing
//
// Normal mode: PID altitude control with velocity damping
// Landing mode: Fixed descent rate until touchdown
//
// Landing is triggered by NOTIFY_LANDING message. When complete,
// sends NOTIFY_FLIGHT_LANDED to flight manager.
//
// Uses name registry:
// - Registers self as "altitude"
// - Uses whereis() to find "flight_manager"

#include "altitude_actor.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "hal_config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_log.h"
#include <assert.h>
#include <math.h>

static bus_id s_state_bus;
static bus_id s_thrust_bus;
static bus_id s_position_target_bus;

void altitude_actor_init(bus_id state_bus, bus_id thrust_bus,
                         bus_id position_target_bus) {
    s_state_bus = state_bus;
    s_thrust_bus = thrust_bus;
    s_position_target_bus = position_target_bus;
}

// Thrust ramp duration for gentle takeoff (microseconds)
#define THRUST_RAMP_DURATION_US (500000) // 0.5 seconds

// Landing parameters
#define LANDING_DESCENT_RATE (-0.15f) // m/s - gentle descent
#define LANDING_VELOCITY_GAIN 0.5f // thrust adjustment per m/s velocity error

void altitude_actor(void *arg) {
    (void)arg;

    // Register self with name registry
    hive_status status = hive_register("altitude");
    assert(HIVE_SUCCEEDED(status));

    status = hive_bus_subscribe(s_state_bus);
    assert(HIVE_SUCCEEDED(status));
    status = hive_bus_subscribe(s_position_target_bus);
    assert(HIVE_SUCCEEDED(status));

    pid_state_t alt_pid;
    pid_init_full(&alt_pid, HAL_ALT_PID_KP, HAL_ALT_PID_KI, HAL_ALT_PID_KD,
                  HAL_ALT_PID_IMAX, HAL_ALT_PID_OMAX);

    // State
    float target_altitude = 0.0f;
    uint64_t ramp_start_time = 0;
    bool landing_mode = false;
    bool landed = false;
    int count = 0;

    HIVE_LOG_INFO("[ALT] Started, waiting for target altitude");

    uint64_t prev_time = hive_get_time();

    while (1) {
        state_estimate_t state;
        position_target_t target;
        hive_message msg;
        size_t len;

        // Block until state available
        hive_bus_read_wait(s_state_bus, &state, sizeof(state), &len, -1);

        // Measure dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Check for landing command (non-blocking)
        if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY,
                                               NOTIFY_LANDING, &msg, 0))) {
            if (!landing_mode) {
                HIVE_LOG_INFO("[ALT] Landing initiated");
                landing_mode = true;
            }
        }

        // Read target altitude (non-blocking)
        if (hive_bus_read(s_position_target_bus, &target, sizeof(target), &len)
                .code == HIVE_OK) {
            target_altitude = target.z;
        }

        // Emergency cutoff conditions
        bool attitude_emergency = (fabsf(state.roll) > EMERGENCY_TILT_LIMIT) ||
                                  (fabsf(state.pitch) > EMERGENCY_TILT_LIMIT);
        bool altitude_emergency = (state.altitude > EMERGENCY_ALTITUDE_MAX);

        // Touchdown detection (only in landing mode)
        bool touchdown = landing_mode &&
                         (state.altitude < LANDED_ACTUAL_THRESHOLD) &&
                         (fabsf(state.vertical_velocity) < 0.1f);

        bool cutoff = attitude_emergency || altitude_emergency || touchdown;

        float thrust;
        if (cutoff) {
            thrust = 0.0f;
            pid_reset(&alt_pid);
            ramp_start_time = 0;

            // Notify flight manager once when landed
            if (touchdown && !landed) {
                landed = true;
                HIVE_LOG_INFO("[ALT] Touchdown - notifying flight manager");

                // Look up flight manager and notify
                actor_id fm;
                status = hive_whereis("flight_manager", &fm);
                assert(HIVE_SUCCEEDED(status));
                hive_ipc_notify(fm, NOTIFY_FLIGHT_LANDED, NULL, 0);
            }
        } else if (landing_mode) {
            // Landing mode: control descent rate, not altitude
            // Target velocity = LANDING_DESCENT_RATE, adjust thrust to achieve
            // it
            float velocity_error =
                LANDING_DESCENT_RATE - state.vertical_velocity;
            thrust = HAL_BASE_THRUST + LANDING_VELOCITY_GAIN * velocity_error;
            thrust = CLAMPF(thrust, 0.0f, 1.0f);
        } else {
            // Normal altitude hold mode
            if (ramp_start_time == 0) {
                ramp_start_time = hive_get_time();
            }

            // PID altitude control
            float pos_correction =
                pid_update(&alt_pid, target_altitude, state.altitude, dt);

            // Velocity damping
            float vel_damping =
                -HAL_VVEL_DAMPING_GAIN * state.vertical_velocity;

            // Thrust ramp for gentle takeoff
            uint64_t elapsed_us = hive_get_time() - ramp_start_time;
            float ramp = (elapsed_us < THRUST_RAMP_DURATION_US)
                             ? (float)elapsed_us / THRUST_RAMP_DURATION_US
                             : 1.0f;

            thrust =
                ramp * CLAMPF(HAL_BASE_THRUST + pos_correction + vel_damping,
                              0.0f, 1.0f);
        }

        thrust_cmd_t cmd = {.thrust = thrust};
        hive_bus_publish(s_thrust_bus, &cmd, sizeof(cmd));

        if (++count % DEBUG_PRINT_INTERVAL == 0) {
            HIVE_LOG_DEBUG("[ALT] tgt=%.2f alt=%.2f vvel=%.2f thrust=%.3f %s",
                           target_altitude, state.altitude,
                           state.vertical_velocity, thrust,
                           landing_mode ? "[LANDING]" : "");
        }
    }
}

// Altitude actor - Altitude hold control with controlled landing
//
// Normal mode: PID altitude control with velocity damping
// Landing mode: Fixed descent rate until touchdown
//
// Landing is triggered by NOTIFY_LANDING message. When complete,
// sends NOTIFY_FLIGHT_LANDED to flight manager.

#include "altitude_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "hal_config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_timer.h"
#include "hive_log.h"
#include <math.h>
#include <string.h>

// Thrust ramp duration for gentle takeoff (microseconds)
#define THRUST_RAMP_DURATION_US (500000) // 0.5 seconds

// Landing parameters
#define LANDING_DESCENT_RATE (-0.15f) // m/s - gentle descent
#define LANDING_VELOCITY_GAIN 0.5f // thrust adjustment per m/s velocity error

// Actor state - initialized by altitude_actor_init
typedef struct {
    bus_id_t state_bus;
    bus_id_t thrust_bus;
    bus_id_t position_target_bus;
    actor_id_t flight_manager;
} altitude_state_t;

void *altitude_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static altitude_state_t state;
    state.state_bus = buses->state_bus;
    state.thrust_bus = buses->thrust_bus;
    state.position_target_bus = buses->position_target_bus;
    state.flight_manager = HIVE_ACTOR_ID_INVALID; // Set from siblings in actor
    return &state;
}

void altitude_actor(void *args, const hive_spawn_info_t *siblings,
                    size_t sibling_count) {
    altitude_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[ALT] Failed to find flight_manager sibling");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->position_target_bus))) {
        HIVE_LOG_ERROR("[ALT] Bus subscribe failed");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

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

    // Set up hive_select() sources: state bus + landing command
    enum { SEL_STATE, SEL_LANDING };
    hive_select_source_t sources[] = {
        [SEL_STATE] = {HIVE_SEL_BUS, .bus = state->state_bus},
        [SEL_LANDING] = {HIVE_SEL_IPC,
                         .ipc = {state->flight_manager, HIVE_MSG_NOTIFY,
                                 NOTIFY_LANDING}},
    };

    while (1) {
        state_estimate_t est;
        position_target_t target;
        size_t len;
        hive_status_t status;

        // Wait for state update OR landing command (unified event waiting)
        hive_select_result_t result;
        status = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[ALT] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_LANDING) {
            // Landing command received - respond immediately
            if (!landing_mode) {
                HIVE_LOG_INFO("[ALT] Landing initiated");
                landing_mode = true;
            }
            continue; // Loop back to wait for next event
        }

        // SEL_STATE: Copy state data from select result
        if (result.bus.len != sizeof(est)) {
            HIVE_LOG_WARN("[ALT] Invalid state bus message size: %zu",
                          result.bus.len);
            continue;
        }
        memcpy(&est, result.bus.data, sizeof(est));

        // Measure dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Read target altitude (non-blocking)
        if (hive_bus_read(state->position_target_bus, &target, sizeof(target),
                          &len, HIVE_TIMEOUT_NONBLOCKING)
                .code == HIVE_OK) {
            target_altitude = target.z;
        }

        // Emergency cutoff conditions
        bool attitude_emergency = (fabsf(est.roll) > EMERGENCY_TILT_LIMIT) ||
                                  (fabsf(est.pitch) > EMERGENCY_TILT_LIMIT);
        bool altitude_emergency = (est.altitude > EMERGENCY_ALTITUDE_MAX);

        // Touchdown detection (only in landing mode)
        bool touchdown = landing_mode &&
                         (est.altitude < LANDED_ACTUAL_THRESHOLD) &&
                         (fabsf(est.vertical_velocity) < 0.1f);

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
                hive_ipc_notify(state->flight_manager, NOTIFY_FLIGHT_LANDED,
                                NULL, 0);
            }
        } else if (landing_mode) {
            // Landing mode: control descent rate, not altitude
            // Target velocity = LANDING_DESCENT_RATE, adjust thrust to achieve
            // it
            float velocity_error = LANDING_DESCENT_RATE - est.vertical_velocity;
            thrust = HAL_BASE_THRUST + LANDING_VELOCITY_GAIN * velocity_error;
            thrust = CLAMPF(thrust, 0.0f, 1.0f);
        } else {
            // Normal altitude hold mode
            if (ramp_start_time == 0) {
                ramp_start_time = hive_get_time();
            }

            // PID altitude control
            float pos_correction =
                pid_update(&alt_pid, target_altitude, est.altitude, dt);

            // Velocity damping
            float vel_damping = -HAL_VVEL_DAMPING_GAIN * est.vertical_velocity;

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
        hive_bus_publish(state->thrust_bus, &cmd, sizeof(cmd));

        if (++count % DEBUG_PRINT_INTERVAL == 0) {
            HIVE_LOG_DEBUG("[ALT] tgt=%.2f alt=%.2f vvel=%.2f thrust=%.3f %s",
                           target_altitude, est.altitude, est.vertical_velocity,
                           thrust, landing_mode ? "[LANDING]" : "");
        }
    }
}

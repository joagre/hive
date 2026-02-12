// Altitude actor - Altitude hold control with controlled landing
//
// Normal mode: PID altitude control with velocity damping
// Landing mode: Fixed descent rate until touchdown
//
// Landing is triggered by NOTIFY_LANDING message. When complete,
// notifies NOTIFY_FLIGHT_LANDED to flight manager.

#include "altitude_actor.h"
#include "pilot_buses.h"
#include "tunable_params.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_timer.h"
#include "hive_log.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

// Actor state - initialized by altitude_actor_init
typedef struct {
    hive_bus_id_t state_bus;
    hive_bus_id_t thrust_bus;
    hive_bus_id_t position_target_bus;
    hive_actor_id_t flight_manager;
    tunable_params_t *params;
} altitude_state_t;

void *altitude_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static altitude_state_t state;
    state.state_bus = buses->state_bus;
    state.thrust_bus = buses->thrust_bus;
    state.position_target_bus = buses->position_target_bus;
    state.flight_manager = HIVE_ACTOR_ID_INVALID; // Set from siblings in actor
    state.params = buses->params;
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

    // Use tunable params for initial values
    tunable_params_t *p = state->params;
    pid_state_t alt_pid;
    pid_init_full(&alt_pid, p->alt_kp, p->alt_ki, p->alt_kd, p->alt_imax,
                  p->alt_omax);

    // State
    float target_altitude = 0.0f;
    uint64_t ramp_start_time = 0;
    bool landing_mode = false;
    bool landed = false;
    bool crash_detected = false; // Latched - once true, motors stay off
    int count = 0;

    HIVE_LOG_INFO("[ALT] Started, waiting for target altitude");

    uint64_t prev_time = hive_get_time();

    // Track crash logging (not static so RESET can clear it)
    bool logged_crash = false;

    // Set up hive_select() sources: state bus + landing command + RESET
    enum { SEL_STATE, SEL_LANDING, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_STATE] = {.type = HIVE_SEL_BUS, .bus = state->state_bus},
        [SEL_LANDING] = {.type = HIVE_SEL_IPC,
                         .ipc = {.sender = state->flight_manager,
                                 .class = HIVE_MSG_NOTIFY,
                                 .id = NOTIFY_LANDING}},
        [SEL_RESET] = {.type = HIVE_SEL_IPC,
                       .ipc = {.sender = state->flight_manager,
                               .class = HIVE_MSG_NOTIFY,
                               .id = NOTIFY_RESET}},
    };

    while (true) {
        state_estimate_t est;
        position_target_t target;
        size_t len;
        hive_status_t status;

        // Wait for state update OR landing command OR RESET
        hive_select_result_t result;
        status = hive_select(sources, 3, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[ALT] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_RESET) {
            HIVE_LOG_INFO("[ALT] RESET - clearing PID and landing state");
            pid_reset(&alt_pid);
            target_altitude = 0.0f;
            ramp_start_time = 0;
            landing_mode = false;
            landed = false;
            crash_detected = false;
            logged_crash = false;
            count = 0;
            prev_time = hive_get_time();
            continue;
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
            HIVE_LOG_ERROR("[ALT] State bus corrupted: size=%zu expected=%zu",
                           result.bus.len, sizeof(est));
            continue;
        }
        memcpy(&est, result.bus.data, sizeof(est));

        // Measure dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Guard against bad dt (logged by PID, but also check here for context)
        if (dt <= 0.0f || dt > 1.0f) {
            HIVE_LOG_WARN("[ALT] bad dt=%.6f, skipping cycle", dt);
            continue;
        }

        // Read target altitude (non-blocking)
        hive_status_t read_status =
            hive_bus_read(state->position_target_bus, &target, sizeof(target),
                          &len, HIVE_TIMEOUT_NONBLOCKING);
        if (read_status.code == HIVE_OK) {
            // Log target changes
            if (target.z != target_altitude) {
                HIVE_LOG_INFO("[ALT] Target changed: %.2f -> %.2f",
                              target_altitude, target.z);
            }
            target_altitude = target.z;
        }

        // Update PID gains from tunable params (allows live tuning)
        pid_set_gains(&alt_pid, p->alt_kp, p->alt_ki, p->alt_kd);
        pid_set_limits(&alt_pid, p->alt_imax, p->alt_omax);

        // Emergency cutoff conditions (use tunable limits)
        bool attitude_emergency = (fabsf(est.roll) > p->emergency_tilt_limit) ||
                                  (fabsf(est.pitch) > p->emergency_tilt_limit);
        bool altitude_emergency = (est.altitude > p->emergency_alt_max);

        // Latch crash condition - once triggered, motors stay off until reboot
        if (attitude_emergency || altitude_emergency) {
            crash_detected = true;
        }

        // Touchdown detection (only in landing mode)
        bool touchdown = landing_mode &&
                         (est.altitude < LANDED_ACTUAL_THRESHOLD) &&
                         (fabsf(est.vertical_velocity) < 0.1f);

        bool cutoff = crash_detected || touchdown;

        // Log emergency conditions (once per event)
        if (crash_detected && !logged_crash) {
            HIVE_LOG_ERROR(
                "[ALT] CRASH DETECTED - motors disabled until reboot! "
                "roll=%.1f pitch=%.1f alt=%.2f",
                est.roll * RAD_TO_DEG, est.pitch * RAD_TO_DEG, est.altitude);
            logged_crash = true;

            // Notify flight manager that we've crashed (treat as landed)
            if (!landed) {
                landed = true;
                hive_ipc_notify(state->flight_manager, NOTIFY_FLIGHT_LANDED,
                                NULL, 0);
            }
        }

        float thrust;
        if (cutoff) {
            thrust = 0.0f;
            pid_reset(&alt_pid);
            ramp_start_time = 0;

            // Notify flight manager once when landed
            if (touchdown && !landed) {
                landed = true;
                HIVE_LOG_INFO("[ALT] Touchdown at alt=%.3f vvel=%.3f - "
                              "notifying flight manager",
                              est.altitude, est.vertical_velocity);
                hive_ipc_notify(state->flight_manager, NOTIFY_FLIGHT_LANDED,
                                NULL, 0);
            }
        } else if (target_altitude <= 0.0f) {
            // No target yet - stay on ground, keep ramp and PID fresh
            thrust = 0.0f;
            ramp_start_time = 0;
            pid_reset(&alt_pid);
        } else if (landing_mode) {
            // Landing mode: control descent rate, not altitude
            // Target velocity = landing_descent_rate, adjust thrust to achieve it
            if (!isfinite(est.vertical_velocity)) {
                HIVE_LOG_ERROR(
                    "[ALT] NaN velocity in landing - estimator failure!");
                thrust = 0.0f;
            } else {
                float velocity_error =
                    p->landing_descent_rate - est.vertical_velocity;
                thrust =
                    p->hover_thrust + p->landing_velocity_gain * velocity_error;
                thrust = CLAMPF(thrust, 0.0f, 1.0f);
            }
        } else {
            // Normal altitude hold mode
            uint64_t thrust_ramp_us = (uint64_t)(p->thrust_ramp_ms * 1000.0f);
            if (ramp_start_time == 0) {
                ramp_start_time = hive_get_time();
                HIVE_LOG_INFO("[ALT] Takeoff ramp start: roll=%.1f pitch=%.1f "
                              "yaw=%.1f alt=%.2f",
                              est.roll * RAD_TO_DEG, est.pitch * RAD_TO_DEG,
                              est.yaw * RAD_TO_DEG, est.altitude);
            }

            // PID altitude control
            float pos_correction =
                pid_update(&alt_pid, target_altitude, est.altitude, dt);

            // Velocity damping (use tunable param)
            float vel_damping = -p->vvel_damping * est.vertical_velocity;

            // Thrust ramp for gentle takeoff (use tunable ramp duration)
            uint64_t elapsed_us = hive_get_time() - ramp_start_time;
            float ramp = (elapsed_us < thrust_ramp_us)
                             ? (float)elapsed_us / (float)thrust_ramp_us
                             : 1.0f;

            thrust =
                ramp * CLAMPF(p->hover_thrust + pos_correction + vel_damping,
                              0.0f, 1.0f);
        }

        thrust_cmd_t cmd = {.thrust = thrust};
        status = hive_bus_publish(state->thrust_bus, &cmd, sizeof(cmd));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[ALT] bus publish failed: %s", HIVE_ERR_STR(status));
        }

        if (++count % DEBUG_PRINT_INTERVAL == 0) {
            HIVE_LOG_DEBUG("[ALT] tgt=%.2f alt=%.2f vvel=%.2f thrust=%.3f %s",
                           target_altitude, est.altitude, est.vertical_velocity,
                           thrust, landing_mode ? "[LANDING]" : "");
        }
    }
}

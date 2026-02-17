// Altitude actor - Altitude hold with liftoff detection and controlled landing
//
// Takeoff: Ramp thrust linearly until rangefinder detects liftoff,
//   then capture thrust as discovered hover baseline. Self-calibrates
//   each flight for current battery state.
// Hold: PID altitude control with velocity damping around discovered thrust.
// Landing: Fixed descent rate until touchdown.
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
    bool landing_mode = false;
    bool landed = false;
    bool crash_detected = false; // Latched - once true, motors stay off
    int count = 0;

    // Liftoff detection state
    bool liftoff_detected = false;
    float discovered_hover_thrust = 0.0f;
    float ramp_thrust = 0.0f;
    float climb_target = 0.0f; // Smooth reference trajectory after liftoff

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
            landing_mode = false;
            landed = false;
            crash_detected = false;
            logged_crash = false;
            liftoff_detected = false;
            discovered_hover_thrust = 0.0f;
            ramp_thrust = 0.0f;
            climb_target = 0.0f;
            count = 0;
            prev_time = hive_get_time();
            continue;
        }

        if (result.index == SEL_LANDING) {
            // Landing command received - respond immediately
            if (!landing_mode) {
                HIVE_LOG_INFO("[ALT] Landing initiated");
                landing_mode = true;
                pid_reset(&alt_pid); // Clear stale integral from hover
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
        // Only active when in flight (target > 0). On the ground, the
        // altitude Kalman filter may drift because the rangefinder can't
        // measure below ~40mm (drone sits at ~20-30mm on surface).
        // False crash detection on a grounded drone blocks the next flight.
        if (target_altitude > 0.0f) {
            bool attitude_emergency =
                (fabsf(est.roll) > p->emergency_tilt_limit) ||
                (fabsf(est.pitch) > p->emergency_tilt_limit);
            bool altitude_emergency = (est.altitude > p->emergency_alt_max);

            // Latch crash condition - once triggered, motors stay off
            if (attitude_emergency || altitude_emergency) {
                crash_detected = true;
            }
        }

        // Touchdown detection (only in landing mode)
        bool touchdown =
            landing_mode && (est.altitude < LANDED_ACTUAL_THRESHOLD) &&
            (fabsf(est.vertical_velocity) < LANDED_VELOCITY_THRESHOLD);

        bool cutoff = crash_detected || touchdown;

        // Log emergency conditions (once per event)
        if (crash_detected && !logged_crash) {
            HIVE_LOG_ERROR(
                "[ALT] CRASH DETECTED - motors disabled until RESET! "
                "roll=%.1f pitch=%.1f alt=%.2f",
                est.roll * RAD_TO_DEG, est.pitch * RAD_TO_DEG, est.altitude);
            logged_crash = true;

            // Notify flight manager that we've crashed (treat as landed)
            if (!landed) {
                landed = true;
                hive_status_t s = hive_ipc_notify(
                    state->flight_manager, NOTIFY_FLIGHT_LANDED, NULL, 0);
                if (HIVE_FAILED(s)) {
                    HIVE_LOG_ERROR("[ALT] LANDED notify failed: %s",
                                   HIVE_ERR_STR(s));
                }
            }
        }

        float thrust;
        if (cutoff) {
            thrust = 0.0f;
            pid_reset(&alt_pid);

            // Notify flight manager once when landed
            if (touchdown && !landed) {
                landed = true;
                HIVE_LOG_INFO("[ALT] Touchdown at alt=%.3f vvel=%.3f - "
                              "notifying flight manager",
                              est.altitude, est.vertical_velocity);
                hive_status_t s = hive_ipc_notify(
                    state->flight_manager, NOTIFY_FLIGHT_LANDED, NULL, 0);
                if (HIVE_FAILED(s)) {
                    HIVE_LOG_ERROR("[ALT] LANDED notify failed: %s",
                                   HIVE_ERR_STR(s));
                }
            }
        } else if (target_altitude <= 0.0f) {
            // No target yet - stay on ground
            thrust = 0.0f;
            pid_reset(&alt_pid);
            ramp_thrust = 0.0f;
        } else if (landing_mode) {
            // Landing mode: control descent rate, not altitude
            if (!isfinite(est.vertical_velocity)) {
                HIVE_LOG_ERROR(
                    "[ALT] NaN velocity in landing - estimator failure!");
                thrust = 0.0f;
            } else {
                float velocity_error =
                    p->landing_descent_rate - est.vertical_velocity;
                float vel_correction =
                    p->landing_velocity_gain * velocity_error;
                // Clamp velocity correction to prevent corrupted velocity
                // estimates from causing violent thrust swings
                vel_correction =
                    CLAMPF(vel_correction, -p->alt_omax, p->alt_omax);
                thrust = discovered_hover_thrust + vel_correction;
                thrust = CLAMPF(thrust, MIN_AIRBORNE_THRUST, 1.0f);
            }
        } else if (!liftoff_detected) {
            // Liftoff detection: ramp thrust until rangefinder sees altitude
            ramp_thrust += LIFTOFF_RAMP_RATE * dt;

            if (ramp_thrust > LIFTOFF_MAX_THRUST) {
                // Failed to lift off at max thrust - something is wrong
                HIVE_LOG_ERROR("[ALT] Liftoff failed at thrust=%.2f - "
                               "check propellers/battery",
                               ramp_thrust);
                ramp_thrust = 0.0f;
                crash_detected = true;
                thrust = 0.0f;
            } else if (est.altitude > LIFTOFF_ALT_THRESHOLD) {
                // Liftoff detected - capture hover thrust with correction.
                // The ramp continues past actual hover because detection
                // has physical lag (drone must rise LIFTOFF_ALT_THRESHOLD).
                discovered_hover_thrust =
                    ramp_thrust * LIFTOFF_THRUST_CORRECTION;
                liftoff_detected = true;
                climb_target = est.altitude;
                HIVE_LOG_INFO("[ALT] Liftoff at thrust=%.3f corrected=%.3f "
                              "alt=%.3f",
                              ramp_thrust, discovered_hover_thrust,
                              est.altitude);
                thrust = discovered_hover_thrust;

                // Notify flight manager to start flight duration timer
                hive_ipc_notify(state->flight_manager, NOTIFY_LIFTOFF, NULL, 0);
            } else {
                thrust = ramp_thrust;
            }
        } else {
            // Normal altitude hold using discovered hover thrust.
            // Instead of tracking target_altitude directly (step change
            // causes overshoot), ramp a smooth reference trajectory from
            // liftoff altitude toward the target at LIFTOFF_CLIMB_RATE.
            // PID runs at full authority tracking the moving reference.
            if (climb_target < target_altitude) {
                climb_target += LIFTOFF_CLIMB_RATE * dt;
                if (climb_target > target_altitude) {
                    climb_target = target_altitude;
                }
            } else if (climb_target > target_altitude) {
                climb_target -= LIFTOFF_CLIMB_RATE * dt;
                if (climb_target < target_altitude) {
                    climb_target = target_altitude;
                }
            }

            float pos_correction =
                pid_update(&alt_pid, climb_target, est.altitude, dt);
            float vel_damping = -p->vvel_damping * est.vertical_velocity;
            // Clamp velocity damping to same magnitude as PID output limit.
            // Without this, corrupted velocity estimates (from rangefinder
            // glitches) produce huge damping values that saturate thrust
            // and cause unrecoverable oscillations.
            vel_damping = CLAMPF(vel_damping, -p->alt_omax, p->alt_omax);

            // Feedforward: cancel the velocity damping penalty during
            // active climb/descent ramp. Without this, the damping term
            // fights the expected ramp velocity, forcing the PID integral
            // to wind up to compensate, then overshoot when the ramp stops.
            float feedforward = 0.0f;
            if (climb_target < target_altitude) {
                feedforward = LIFTOFF_CLIMB_RATE * p->vvel_damping;
            } else if (climb_target > target_altitude) {
                feedforward = -LIFTOFF_CLIMB_RATE * p->vvel_damping;
            }

            float correction = pos_correction + vel_damping + feedforward;
            thrust = CLAMPF(discovered_hover_thrust + correction,
                            MIN_AIRBORNE_THRUST, 1.0f);
        }

        thrust_cmd_t cmd = {.thrust = thrust};
        status = hive_bus_publish(state->thrust_bus, &cmd, sizeof(cmd));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[ALT] bus publish failed: %s", HIVE_ERR_STR(status));
        }

        if (++count % DEBUG_PRINT_INTERVAL == 0) {
            HIVE_LOG_DEBUG("[ALT] r=%.1f p=%.1f alt=%.2f vvel=%.2f thr=%.3f %s",
                           est.roll * RAD_TO_DEG, est.pitch * RAD_TO_DEG,
                           est.altitude, est.vertical_velocity, thrust,
                           landing_mode ? "[LND]" : "");
        }
    }
}

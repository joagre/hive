// Rate actor - Angular rate stabilization
//
// Subscribes to state, thrust, and rate setpoint buses, runs rate PIDs,
// publishes torque commands.

#include "rate_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "tunable_params.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_log.h"
#include "hive_timer.h"
#include <string.h>

// Actor state - initialized by rate_actor_init
typedef struct {
    hive_bus_id_t state_bus;
    hive_bus_id_t thrust_bus;
    hive_bus_id_t rate_setpoint_bus;
    hive_bus_id_t torque_bus;
    tunable_params_t *params;
    hive_actor_id_t flight_manager;
} rate_state_t;

void *rate_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static rate_state_t state;
    state.state_bus = buses->state_bus;
    state.thrust_bus = buses->thrust_bus;
    state.rate_setpoint_bus = buses->rate_setpoint_bus;
    state.torque_bus = buses->torque_bus;
    state.params = buses->params;
    state.flight_manager = HIVE_ACTOR_ID_INVALID;
    return &state;
}

void rate_actor(void *args, const hive_spawn_info_t *siblings,
                size_t sibling_count) {
    rate_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[RATE] Failed to find flight_manager sibling");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->thrust_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->rate_setpoint_bus))) {
        HIVE_LOG_ERROR("[RATE] Bus subscribe failed");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    pid_state_t roll_pid, pitch_pid, yaw_pid;
    // Note: Different output limits per axis (yaw needs more authority)
    // Use tunable params for initial values
    tunable_params_t *p = state->params;
    pid_init_full(&roll_pid, p->rate_kp, p->rate_ki, p->rate_kd, p->rate_imax,
                  p->rate_omax_roll);
    pid_init_full(&pitch_pid, p->rate_kp, p->rate_ki, p->rate_kd, p->rate_imax,
                  p->rate_omax_pitch);
    pid_init_full(&yaw_pid, p->rate_kp, p->rate_ki, p->rate_kd, p->rate_imax,
                  p->rate_omax_yaw);

    float thrust = 0.0f;
    rate_setpoint_t rate_sp = RATE_SETPOINT_ZERO;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    // Set up hive_select() sources: state bus + RESET notification
    enum { SEL_STATE, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_STATE] = {HIVE_SEL_BUS, .bus = state->state_bus},
        [SEL_RESET] = {HIVE_SEL_IPC, .ipc = {state->flight_manager,
                                             HIVE_MSG_NOTIFY, NOTIFY_RESET}},
    };

    while (1) {
        state_estimate_t est;
        thrust_cmd_t thrust_cmd;
        rate_setpoint_t new_rate_sp;
        size_t len;
        hive_status_t status;

        // Wait for state OR RESET request
        hive_select_result_t result;
        status = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[RATE] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_RESET) {
            HIVE_LOG_INFO("[RATE] RESET - clearing PIDs");
            pid_reset(&roll_pid);
            pid_reset(&pitch_pid);
            pid_reset(&yaw_pid);
            thrust = 0.0f;
            rate_sp = (rate_setpoint_t)RATE_SETPOINT_ZERO;
            prev_time = hive_get_time();
            continue;
        }

        // SEL_STATE: Copy state data from select result
        if (result.bus.len != sizeof(est)) {
            HIVE_LOG_ERROR("[RATE] State bus corrupted: size=%zu expected=%zu",
                           result.bus.len, sizeof(est));
            continue;
        }
        memcpy(&est, result.bus.data, sizeof(est));

        // Measure actual dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Guard against bad dt
        if (dt <= 0.0f || dt > 1.0f) {
            HIVE_LOG_WARN("[RATE] bad dt=%.6f, skipping cycle", dt);
            continue;
        }

        // Read thrust and rate setpoints (non-blocking, use last known)
        if (hive_bus_read(state->thrust_bus, &thrust_cmd, sizeof(thrust_cmd),
                          &len, HIVE_TIMEOUT_NONBLOCKING)
                .code == HIVE_OK) {
            thrust = thrust_cmd.thrust;
        }

        if (hive_bus_read(state->rate_setpoint_bus, &new_rate_sp,
                          sizeof(new_rate_sp), &len, HIVE_TIMEOUT_NONBLOCKING)
                .code == HIVE_OK) {
            rate_sp = new_rate_sp;
        }

        // Update PID gains from tunable params (allows live tuning)
        pid_set_gains(&roll_pid, p->rate_kp, p->rate_ki, p->rate_kd);
        pid_set_gains(&pitch_pid, p->rate_kp, p->rate_ki, p->rate_kd);
        pid_set_gains(&yaw_pid, p->rate_kp, p->rate_ki, p->rate_kd);
        pid_set_limits(&roll_pid, p->rate_imax, p->rate_omax_roll);
        pid_set_limits(&pitch_pid, p->rate_imax, p->rate_omax_pitch);
        pid_set_limits(&yaw_pid, p->rate_imax, p->rate_omax_yaw);

        // Torque command uses standard conventions (HAL handles platform
        // differences)
        torque_cmd_t cmd;
        cmd.thrust = thrust;
        cmd.roll = pid_update(&roll_pid, rate_sp.roll, est.roll_rate, dt);
        cmd.pitch = pid_update(&pitch_pid, rate_sp.pitch, est.pitch_rate, dt);
        cmd.yaw = pid_update(&yaw_pid, rate_sp.yaw, est.yaw_rate, dt);

        status = hive_bus_publish(state->torque_bus, &cmd, sizeof(cmd));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[RATE] bus publish failed: %s",
                          HIVE_ERR_STR(status));
        }
    }
}

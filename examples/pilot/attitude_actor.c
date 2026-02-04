// Attitude actor - Attitude angle control
//
// Subscribes to state and attitude setpoint buses, runs attitude PID
// controllers, publishes rate setpoints.

#include "attitude_actor.h"
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

// Actor state - initialized by attitude_actor_init
typedef struct {
    hive_bus_id_t state_bus;
    hive_bus_id_t attitude_setpoint_bus;
    hive_bus_id_t rate_setpoint_bus;
    tunable_params_t *params;
    hive_actor_id_t flight_manager;
} attitude_state_t;

void *attitude_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static attitude_state_t state;
    state.state_bus = buses->state_bus;
    state.attitude_setpoint_bus = buses->attitude_setpoint_bus;
    state.rate_setpoint_bus = buses->rate_setpoint_bus;
    state.params = buses->params;
    state.flight_manager = HIVE_ACTOR_ID_INVALID;
    return &state;
}

void attitude_actor(void *args, const hive_spawn_info_t *siblings,
                    size_t sibling_count) {
    attitude_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[ATT] Failed to find flight_manager sibling");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->attitude_setpoint_bus))) {
        HIVE_LOG_ERROR("[ATT] Bus subscribe failed");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Use tunable params for initial values
    tunable_params_t *p = state->params;
    pid_state_t roll_pid, pitch_pid, yaw_pid;
    pid_init_full(&roll_pid, p->att_kp, p->att_ki, p->att_kd, p->att_imax,
                  p->att_omax);
    pid_init_full(&pitch_pid, p->att_kp, p->att_ki, p->att_kd, p->att_imax,
                  p->att_omax);
    pid_init_full(&yaw_pid, p->att_kp, p->att_ki, p->att_kd, p->att_imax,
                  p->att_omax);

    // Target attitudes (updated from attitude_setpoint_bus)
    attitude_setpoint_t attitude_sp = ATTITUDE_SETPOINT_ZERO;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    // Set up hive_select() sources: state bus + RESET request
    enum { SEL_STATE, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_STATE] = {HIVE_SEL_BUS, .bus = state->state_bus},
        [SEL_RESET] = {HIVE_SEL_IPC, .ipc = {state->flight_manager,
                                             HIVE_MSG_REQUEST, HIVE_TAG_ANY}},
    };

    while (1) {
        state_estimate_t est;
        attitude_setpoint_t new_attitude_sp;
        size_t len;
        hive_status_t status;

        // Wait for state OR RESET request
        hive_select_result_t result;
        status = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[ATT] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_RESET) {
            // Verify it's a RESET request
            uint8_t reply = REPLY_OK;
            if (result.ipc.len != 1 ||
                ((uint8_t *)result.ipc.data)[0] != REQUEST_RESET) {
                HIVE_LOG_WARN("[ATT] Unknown request ignored");
                reply = REPLY_FAIL;
            } else {
                HIVE_LOG_INFO("[ATT] RESET - clearing PIDs");
                pid_reset(&roll_pid);
                pid_reset(&pitch_pid);
                pid_reset(&yaw_pid);
                attitude_sp = (attitude_setpoint_t)ATTITUDE_SETPOINT_ZERO;
                prev_time = hive_get_time();
            }
            hive_ipc_reply(&result.ipc, &reply, sizeof(reply));
            continue;
        }

        // SEL_STATE: Copy state data from select result
        if (result.bus.len != sizeof(est)) {
            HIVE_LOG_ERROR("[ATT] State bus corrupted: size=%zu expected=%zu",
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
            HIVE_LOG_WARN("[ATT] bad dt=%.6f, skipping cycle", dt);
            continue;
        }

        // Read attitude setpoints from position controller (non-blocking, use
        // last known)
        if (hive_bus_read(state->attitude_setpoint_bus, &new_attitude_sp,
                          sizeof(new_attitude_sp), &len,
                          HIVE_TIMEOUT_NONBLOCKING)
                .code == HIVE_OK) {
            attitude_sp = new_attitude_sp;
        }

        // Update PID gains from tunable params (allows live tuning)
        pid_set_gains(&roll_pid, p->att_kp, p->att_ki, p->att_kd);
        pid_set_gains(&pitch_pid, p->att_kp, p->att_ki, p->att_kd);
        pid_set_gains(&yaw_pid, p->att_kp, p->att_ki, p->att_kd);
        pid_set_limits(&roll_pid, p->att_imax, p->att_omax);
        pid_set_limits(&pitch_pid, p->att_imax, p->att_omax);
        pid_set_limits(&yaw_pid, p->att_imax, p->att_omax);

        rate_setpoint_t setpoint;
        setpoint.roll = pid_update(&roll_pid, attitude_sp.roll, est.roll, dt);
        setpoint.pitch =
            pid_update(&pitch_pid, attitude_sp.pitch, est.pitch, dt);
        setpoint.yaw = pid_update_angle(&yaw_pid, attitude_sp.yaw, est.yaw, dt);

        status = hive_bus_publish(state->rate_setpoint_bus, &setpoint,
                                  sizeof(setpoint));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[ATT] bus publish failed: %s", HIVE_ERR_STR(status));
        }
    }
}

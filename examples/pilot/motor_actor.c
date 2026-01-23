// Motor actor - Output layer
//
// Subscribes to torque bus, writes to hardware via HAL.
// The HAL handles mixing (converting torque to individual motor commands).
// Uses hive_select() to wait on torque bus OR STOP notification simultaneously,
// ensuring immediate response to STOP commands (critical for safety).
//
// DEADMAN WATCHDOG: If no torque command is received within
// MOTOR_DEADMAN_TIMEOUT_MS, motors are automatically zeroed. This prevents
// runaway motors if upstream control actors crash.

#include "motor_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_log.h"
#include "hive_select.h"
#include <string.h>

// Actor state - initialized by motor_actor_init
typedef struct {
    bus_id torque_bus;
    actor_id flight_manager;
} motor_state;

void *motor_actor_init(void *init_args) {
    const pilot_buses *buses = init_args;
    static motor_state state;
    state.torque_bus = buses->torque_bus;
    state.flight_manager = ACTOR_ID_INVALID; // Set from siblings in actor
    return &state;
}

void motor_actor(void *args, const hive_spawn_info *siblings,
                 size_t sibling_count) {
    motor_state *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[MOTOR] flight_manager sibling not found");
        return;
    }

    hive_status status = hive_bus_subscribe(state->torque_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[MOTOR] failed to subscribe to torque bus");
        return;
    }

    bool stopped = false;

    // Set up hive_select() sources: torque bus + STOP notification
    enum { SEL_TORQUE, SEL_STOP };
    hive_select_source sources[] = {
        [SEL_TORQUE] = {HIVE_SEL_BUS, .bus = state->torque_bus},
        [SEL_STOP] = {HIVE_SEL_IPC,
                      .ipc = {state->flight_manager, HIVE_MSG_NOTIFY,
                              NOTIFY_FLIGHT_STOP}},
    };

    while (1) {
        torque_cmd_t torque;

        // Wait for torque command OR STOP notification (unified event waiting)
        // Timeout implements deadman watchdog - zero motors if no command
        hive_select_result result;
        status = hive_select(sources, 2, &result, MOTOR_DEADMAN_TIMEOUT_MS);

        if (status.code == HIVE_ERR_TIMEOUT) {
            // DEADMAN TIMEOUT - no torque command received, zero motors
            HIVE_LOG_WARN("[MOTOR] Deadman timeout - zeroing motors");
            torque = (torque_cmd_t)TORQUE_CMD_ZERO;
            hal_write_torque(&torque);
            continue;
        }

        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[MOTOR] select failed: %s", HIVE_ERR_STR(status));
            return;
        }

        if (result.index == SEL_STOP) {
            // STOP received - respond immediately
            stopped = true;
            torque = (torque_cmd_t)TORQUE_CMD_ZERO;
            hal_write_torque(&torque);
            continue; // Loop back to wait for next event
        }

        // SEL_TORQUE: Copy torque data from select result
        if (result.bus.len != sizeof(torque)) {
            HIVE_LOG_WARN("[MOTOR] Invalid torque bus message size: %zu",
                          result.bus.len);
            continue;
        }
        memcpy(&torque, result.bus.data, sizeof(torque));

        if (stopped) {
            torque = (torque_cmd_t)TORQUE_CMD_ZERO;
        }

        hal_write_torque(&torque);
    }
}

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
#include <math.h>

// Torque validation limits
// These are sanity checks - values outside indicate control system failure
#define MAX_THRUST 1.5f     // Allow some headroom above 1.0
#define MAX_TORQUE_MAG 2.0f // Roll/pitch/yaw torque magnitude limit

// Validate torque command, return true if sane
// Checks for NaN/Inf and reasonable magnitude
static bool validate_torque(const torque_cmd_t *cmd) {
    // Check for NaN or Inf
    if (!isfinite(cmd->thrust) || !isfinite(cmd->roll) ||
        !isfinite(cmd->pitch) || !isfinite(cmd->yaw)) {
        return false;
    }
    // Check magnitude limits
    if (cmd->thrust < -0.1f || cmd->thrust > MAX_THRUST) {
        return false;
    }
    if (fabsf(cmd->roll) > MAX_TORQUE_MAG ||
        fabsf(cmd->pitch) > MAX_TORQUE_MAG ||
        fabsf(cmd->yaw) > MAX_TORQUE_MAG) {
        return false;
    }
    return true;
}

// Actor state - initialized by motor_actor_init
typedef struct {
    hive_bus_id_t torque_bus;
    hive_actor_id_t flight_manager;
} motor_state_t;

void *motor_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static motor_state_t state;
    state.torque_bus = buses->torque_bus;
    state.flight_manager = HIVE_ACTOR_ID_INVALID; // Set from siblings in actor
    return &state;
}

void motor_actor(void *args, const hive_spawn_info_t *siblings,
                 size_t sibling_count) {
    motor_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[MOTOR] flight_manager sibling not found");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    hive_status_t status = hive_bus_subscribe(state->torque_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[MOTOR] failed to subscribe to torque bus");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    bool stopped = false;

    // Set up hive_select() sources: torque bus + STOP notification
    enum { SEL_TORQUE, SEL_STOP };
    hive_select_source_t sources[] = {
        [SEL_TORQUE] = {HIVE_SEL_BUS, .bus = state->torque_bus},
        [SEL_STOP] = {HIVE_SEL_IPC,
                      .ipc = {state->flight_manager, HIVE_MSG_NOTIFY,
                              NOTIFY_FLIGHT_STOP}},
    };

    while (1) {
        torque_cmd_t torque;

        // Wait for torque command OR STOP notification (unified event waiting)
        // Timeout implements deadman watchdog - zero motors if no command
        hive_select_result_t result;
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
            hive_exit(HIVE_EXIT_REASON_CRASH);
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

        // Validate torque command - last line of defense against garbage data
        if (!validate_torque(&torque)) {
            HIVE_LOG_WARN(
                "[MOTOR] invalid torque rejected: t=%.2f r=%.2f p=%.2f y=%.2f",
                torque.thrust, torque.roll, torque.pitch, torque.yaw);
            torque = (torque_cmd_t)TORQUE_CMD_ZERO;
        }

        if (stopped) {
            torque = (torque_cmd_t)TORQUE_CMD_ZERO;
        }

        hal_write_torque(&torque);
    }
}

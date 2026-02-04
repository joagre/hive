// Motor actor - Output layer
//
// Subscribes to torque bus, writes to hardware via HAL.
// The HAL handles mixing (converting torque to individual motor commands).
// Uses hive_select() to wait on torque bus OR control messages simultaneously.
//
// Lifecycle messages from flight_manager:
//   ARM - call hal_arm(), enable motor output
//   DISARM - call hal_disarm(), disable motor output
//   RESET - clear state for new flight
//   START - begin flight, arm must have been called first
//   STOP - stop motors
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

    // Motor state
    bool armed = false;
    bool started = false;
    bool stopped = false;
    bool first_thrust_logged = false;

    HIVE_LOG_INFO("[MOTOR] Waiting for ARM command");

    // Set up hive_select() sources
    // Note: REQUEST uses HIVE_TAG_ANY since hive_ipc_request() generates tags
    enum { SEL_TORQUE, SEL_START, SEL_STOP, SEL_REQUEST };
    hive_select_source_t sources[] = {
        [SEL_TORQUE] = {HIVE_SEL_BUS, .bus = state->torque_bus},
        [SEL_START] = {HIVE_SEL_IPC,
                       .ipc = {state->flight_manager, HIVE_MSG_NOTIFY,
                               NOTIFY_FLIGHT_START}},
        [SEL_STOP] = {HIVE_SEL_IPC,
                      .ipc = {state->flight_manager, HIVE_MSG_NOTIFY,
                              NOTIFY_FLIGHT_STOP}},
        [SEL_REQUEST] = {HIVE_SEL_IPC, .ipc = {state->flight_manager,
                                               HIVE_MSG_REQUEST, HIVE_TAG_ANY}},
    };

    while (1) {
        torque_cmd_t torque = TORQUE_CMD_ZERO;

        // Wait for torque command OR control messages
        // Use deadman timeout only when started (flying)
        int64_t timeout = started ? MOTOR_DEADMAN_TIMEOUT_MS : -1;
        hive_select_result_t result;
        status = hive_select(sources, 4, &result, timeout);

        if (status.code == HIVE_ERR_TIMEOUT) {
            // DEADMAN TIMEOUT - no torque command received, zero motors
            HIVE_LOG_WARN("[MOTOR] Deadman timeout - zeroing motors");
            hal_write_torque(&torque);
            continue;
        }

        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[MOTOR] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        switch (result.index) {
        case SEL_REQUEST: {
            // Handle REQUEST from flight_manager
            // Dispatch based on payload byte: RESET, ARM, or DISARM
            uint8_t reply = REPLY_OK;
            uint8_t req_type =
                (result.ipc.len == 1) ? ((uint8_t *)result.ipc.data)[0] : 0xFF;

            if (req_type == REQUEST_RESET) {
                // RESET request - clear state for new flight
                HIVE_LOG_INFO("[MOTOR] RESET - clearing state");
                started = false;
                stopped = false;
                first_thrust_logged = false;
                hal_write_torque(&torque); // Zero motors
            } else if (req_type == REQUEST_ARM) {
                // ARM request - enable motor output
                HIVE_LOG_INFO("[MOTOR] ARM - enabling motors");
                hal_arm();
                armed = true;
            } else if (req_type == REQUEST_DISARM) {
                // DISARM request - disable motor output
                HIVE_LOG_INFO("[MOTOR] DISARM - disabling motors");
                hal_write_torque(&torque); // Zero motors first
                hal_disarm();
                armed = false;
                started = false;
                stopped = false;
            } else {
                HIVE_LOG_WARN("[MOTOR] Unknown request (len=%zu)",
                              result.ipc.len);
                reply = REPLY_FAIL;
            }
            hive_ipc_reply(&result.ipc, &reply, sizeof(reply));
            continue;
        }

        case SEL_START: {
            // START notification - begin flight
            if (!armed) {
                HIVE_LOG_WARN("[MOTOR] START received but not armed");
            } else {
                HIVE_LOG_INFO("[MOTOR] START - flight authorized");
                started = true;
                stopped = false;
            }
            continue;
        }

        case SEL_STOP: {
            // STOP notification - stop motors
            if (!stopped) {
                HIVE_LOG_INFO("[MOTOR] STOP received - motors disabled");
                stopped = true;
            }
            hal_write_torque(&torque); // Zero motors
            continue;
        }

        case SEL_TORQUE: {
            // SEL_TORQUE: Copy torque data from select result
            if (result.bus.len != sizeof(torque)) {
                HIVE_LOG_ERROR(
                    "[MOTOR] Torque bus corrupted: size=%zu expected=%zu",
                    result.bus.len, sizeof(torque));
                continue;
            }
            memcpy(&torque, result.bus.data, sizeof(torque));

            // Validate torque command - last line of defense
            if (!validate_torque(&torque)) {
                HIVE_LOG_ERROR("[MOTOR] Invalid torque rejected: t=%.2f r=%.2f "
                               "p=%.2f y=%.2f - control system failure!",
                               torque.thrust, torque.roll, torque.pitch,
                               torque.yaw);
                torque = (torque_cmd_t)TORQUE_CMD_ZERO;
            }

            // Only apply torque if armed, started, and not stopped
            if (!armed || !started || stopped) {
                torque = (torque_cmd_t)TORQUE_CMD_ZERO;
            }

            // Log first non-zero thrust (takeoff moment)
            if (!first_thrust_logged && torque.thrust > 0.01f) {
                HIVE_LOG_INFO("[MOTOR] First thrust: %.3f - MOTORS ENGAGED",
                              torque.thrust);
                first_thrust_logged = true;
            }

            hal_write_torque(&torque);
            continue;
        }
        }
    }
}

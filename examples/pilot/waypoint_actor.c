// Waypoint actor - Waypoint navigation manager
//
// Subscribes to state bus to monitor position, publishes current target
// to position target bus. Advances through waypoint list when arrival detected.

#include "waypoint_actor.h"
#include "pilot_buses.h"
#include "tunable_params.h"
#include "flight_profiles.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_timer.h"
#include "hive_log.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

// Actor state - initialized by waypoint_actor_init
typedef struct {
    hive_bus_id_t state_bus;
    hive_bus_id_t position_target_bus;
    hive_actor_id_t flight_manager;
    tunable_params_t *params;
} waypoint_state_t;

void *waypoint_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static waypoint_state_t state;
    state.state_bus = buses->state_bus;
    state.position_target_bus = buses->position_target_bus;
    state.flight_manager = HIVE_ACTOR_ID_INVALID; // Set from siblings in actor
    state.params = buses->params;
    return &state;
}

// Check if drone has arrived at waypoint (uses tunable params)
static bool check_arrival(const waypoint_t *wp, const state_estimate_t *est,
                          const tunable_params_t *p) {
    // Validate position - NaN means we don't know where we are
    if (!isfinite(est->x) || !isfinite(est->y) || !isfinite(est->altitude)) {
        return false; // Can't declare arrival with unknown position
    }

    float dx = wp->x - est->x;
    float dy = wp->y - est->y;
    float dist_xy = sqrtf(dx * dx + dy * dy);
    float alt_err = fabsf(wp->z - est->altitude);
    float yaw_err = fabsf(normalize_angle(wp->yaw - est->yaw));
    float vel = sqrtf(est->x_velocity * est->x_velocity +
                      est->y_velocity * est->y_velocity);

    return (dist_xy < p->wp_tolerance_xy) && (alt_err < p->wp_tolerance_z) &&
           (yaw_err < p->wp_tolerance_yaw) && (vel < p->wp_tolerance_vel);
}

void waypoint_actor(void *args, const hive_spawn_info_t *siblings,
                    size_t sibling_count) {
    waypoint_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[WPT] Failed to find flight_manager sibling");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    hive_status_t status = hive_bus_subscribe(state->state_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[WPT] Failed to subscribe to state bus: %s",
                       HIVE_ERR_STR(status));
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Wait for START signal from flight manager before beginning flight
    // Also handle RESET requests during this wait
    HIVE_LOG_INFO("[WPT] Flight profile: %s (%d waypoints, %.0fs hover)",
                  FLIGHT_PROFILE_NAME, (int)NUM_WAYPOINTS,
                  WAYPOINT_HOVER_TIME_US / 1000000.0f);
    HIVE_LOG_INFO("[WPT] Waiting for flight manager START signal");

    // Wait for START or RESET using hive_select
    enum { SEL_START, SEL_RESET_WAIT };
    hive_select_source_t wait_sources[] = {
        [SEL_START] = {HIVE_SEL_IPC,
                       .ipc = {state->flight_manager, HIVE_MSG_NOTIFY,
                               NOTIFY_FLIGHT_START}},
        [SEL_RESET_WAIT] = {HIVE_SEL_IPC,
                            .ipc = {state->flight_manager, HIVE_MSG_REQUEST,
                                    HIVE_TAG_ANY}},
    };

    while (1) {
        hive_select_result_t wait_result;
        status = hive_select(wait_sources, 2, &wait_result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[WPT] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (wait_result.index == SEL_RESET_WAIT) {
            // Handle RESET request while waiting for START
            uint8_t reply = REPLY_OK;
            if (wait_result.ipc.len != 1 ||
                ((uint8_t *)wait_result.ipc.data)[0] != REQUEST_RESET) {
                HIVE_LOG_WARN("[WPT] Unknown request ignored");
                reply = REPLY_FAIL;
            } else {
                HIVE_LOG_INFO("[WPT] RESET (while waiting for START)");
            }
            hive_ipc_reply(&wait_result.ipc, &reply, sizeof(reply));
            continue; // Keep waiting for START
        }

        // SEL_START: START signal received
        break;
    }

    HIVE_LOG_INFO("[WPT] START received - beginning flight sequence");
    HIVE_LOG_INFO("[WPT] First waypoint: (%.1f, %.1f, %.1f) yaw=%.0f",
                  waypoints[0].x, waypoints[0].y, waypoints[0].z,
                  waypoints[0].yaw * 57.3f);

    int waypoint_index = 0;
    hive_timer_id_t hover_timer = HIVE_TIMER_ID_INVALID;
    bool hovering = false;
    bool first_publish = true;

    // Set up hive_select() sources (dynamically adjust count based on hovering)
    enum { SEL_STATE, SEL_HOVER_TIMER, SEL_RESET };

    while (1) {
        const waypoint_t *wp = &waypoints[waypoint_index];

        // Publish current target
        position_target_t target = {
            .x = wp->x, .y = wp->y, .z = wp->z, .yaw = wp->yaw};
        status = hive_bus_publish(state->position_target_bus, &target,
                                  sizeof(target));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[WPT] bus publish failed: %s", HIVE_ERR_STR(status));
        }

        // Log first publish only
        if (first_publish) {
            HIVE_LOG_INFO("[WPT] Published target z=%.2f", target.z);
            first_publish = false;
        }

        // Wait for state update OR hover timer OR RESET
        hive_select_source_t sources[] = {
            [SEL_STATE] = {HIVE_SEL_BUS, .bus = state->state_bus},
            [SEL_HOVER_TIMER] = {HIVE_SEL_IPC,
                                 .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER,
                                         hover_timer}},
            [SEL_RESET] = {HIVE_SEL_IPC,
                           .ipc = {state->flight_manager, HIVE_MSG_REQUEST,
                                   HIVE_TAG_ANY}},
        };

        hive_select_result_t result;
        // If not hovering, use only state bus and RESET (skip hover timer)
        if (!hovering) {
            // Reorder sources to [SEL_STATE, SEL_RESET]
            hive_select_source_t sources_no_timer[] = {
                [0] = {HIVE_SEL_BUS, .bus = state->state_bus},
                [1] = {HIVE_SEL_IPC, .ipc = {state->flight_manager,
                                             HIVE_MSG_REQUEST, HIVE_TAG_ANY}},
            };
            status = hive_select(sources_no_timer, 2, &result, -1);
            if (HIVE_FAILED(status)) {
                HIVE_LOG_ERROR("[WPT] select failed: %s", HIVE_ERR_STR(status));
                hive_exit(HIVE_EXIT_REASON_CRASH);
            }
            // Map result index back to SEL_* enum
            if (result.index == 1) {
                result.index = SEL_RESET;
            }
        } else {
            status = hive_select(sources, 3, &result, -1);
            if (HIVE_FAILED(status)) {
                HIVE_LOG_ERROR("[WPT] select failed: %s", HIVE_ERR_STR(status));
                hive_exit(HIVE_EXIT_REASON_CRASH);
            }
        }

        if (result.index == SEL_RESET) {
            // Verify it's a RESET request
            uint8_t reply = REPLY_OK;
            if (result.ipc.len != 1 ||
                ((uint8_t *)result.ipc.data)[0] != REQUEST_RESET) {
                HIVE_LOG_WARN("[WPT] Unknown request ignored");
                reply = REPLY_FAIL;
            } else {
                HIVE_LOG_INFO("[WPT] RESET - resetting to waypoint 0");
                waypoint_index = 0;
                if (hover_timer != HIVE_TIMER_ID_INVALID) {
                    hive_timer_cancel(hover_timer);
                    hover_timer = HIVE_TIMER_ID_INVALID;
                }
                hovering = false;
                first_publish = true;
            }
            hive_ipc_reply(&result.ipc, &reply, sizeof(reply));
            continue;
        }

        if (result.index == SEL_HOVER_TIMER) {
            // Hover timer fired - advance to next waypoint (loops back to 0)
            hovering = false;
            hover_timer = HIVE_TIMER_ID_INVALID;
            waypoint_index = (waypoint_index + 1) % (int)NUM_WAYPOINTS;
            HIVE_LOG_INFO("[WPT] Advancing to waypoint %d: (%.1f, %.1f, "
                          "%.1f) yaw=%.0f deg",
                          waypoint_index, waypoints[waypoint_index].x,
                          waypoints[waypoint_index].y,
                          waypoints[waypoint_index].z,
                          waypoints[waypoint_index].yaw * RAD_TO_DEG);
            continue; // Loop back to publish new target
        }

        // SEL_STATE: Copy state data from select result
        state_estimate_t est;
        if (result.bus.len != sizeof(est)) {
            HIVE_LOG_ERROR("[WPT] State bus corrupted: size=%zu expected=%zu",
                           result.bus.len, sizeof(est));
            continue;
        }
        memcpy(&est, result.bus.data, sizeof(est));

        // Check arrival and start hover timer (use tunable params)
        tunable_params_t *p = state->params;
        if (!hovering && check_arrival(wp, &est, p)) {
            HIVE_LOG_INFO("[WPT] Arrived at waypoint %d - hovering",
                          waypoint_index);
            // Convert hover time from seconds to microseconds
            uint64_t hover_time_us =
                (uint64_t)(p->wp_hover_time_s * 1000000.0f);
            hive_status_t timer_status =
                hive_timer_after(hover_time_us, &hover_timer);
            if (HIVE_FAILED(timer_status)) {
                HIVE_LOG_ERROR("[WPT] timer_after failed: %s - cannot hover",
                               HIVE_ERR_STR(timer_status));
                // Don't set hovering=true without a timer, or we'd hang forever
                continue;
            }
            hovering = true;
        }
    }
}

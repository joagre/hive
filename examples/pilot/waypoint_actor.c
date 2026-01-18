// Waypoint actor - Waypoint navigation manager
//
// Subscribes to state bus to monitor position, publishes current target
// to position target bus. Advances through waypoint list when arrival detected.
//
// Uses auto_register:
// - Auto-registered as "waypoint" via child spec

#include "waypoint_actor.h"
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
#include <assert.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

static bus_id s_state_bus;
static bus_id s_position_target_bus;

void waypoint_actor_init(bus_id state_bus, bus_id position_target_bus) {
    s_state_bus = state_bus;
    s_position_target_bus = position_target_bus;
}

// Check if drone has arrived at waypoint
static bool check_arrival(const waypoint_t *wp, const state_estimate_t *state) {
    float dx = wp->x - state->x;
    float dy = wp->y - state->y;
    float dist_xy = sqrtf(dx * dx + dy * dy);
    float alt_err = fabsf(wp->z - state->altitude);
    float yaw_err = fabsf(NORMALIZE_ANGLE(wp->yaw - state->yaw));
    float vel = sqrtf(state->x_velocity * state->x_velocity +
                      state->y_velocity * state->y_velocity);

    return (dist_xy < WAYPOINT_TOLERANCE_XY) &&
           (alt_err < WAYPOINT_TOLERANCE_Z) &&
           (yaw_err < WAYPOINT_TOLERANCE_YAW) && (vel < WAYPOINT_TOLERANCE_VEL);
}

void waypoint_actor(void *args, const hive_spawn_info *siblings,
                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Auto-registered as "waypoint" via child spec
    hive_status status = hive_bus_subscribe(s_state_bus);
    assert(HIVE_SUCCEEDED(status));

    // Wait for START signal from flight manager before beginning flight
    HIVE_LOG_INFO("[WPT] Flight profile: %s (%d waypoints, %.0fs hover)",
                  FLIGHT_PROFILE_NAME, (int)NUM_WAYPOINTS,
                  WAYPOINT_HOVER_TIME_US / 1000000.0f);
    HIVE_LOG_INFO("[WPT] Waiting for flight manager START signal");
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, NOTIFY_FLIGHT_START,
                        &msg, -1);
    HIVE_LOG_INFO("[WPT] START received - beginning flight sequence");

    int waypoint_index = 0;
    timer_id hover_timer = TIMER_ID_INVALID;
    bool hovering = false;

    // Set up hive_select() sources (dynamically adjust count based on hovering)
    enum { SEL_STATE, SEL_HOVER_TIMER };

    while (1) {
        const waypoint_t *wp = &waypoints[waypoint_index];

        // Publish current target
        position_target_t target = {
            .x = wp->x, .y = wp->y, .z = wp->z, .yaw = wp->yaw};
        hive_bus_publish(s_position_target_bus, &target, sizeof(target));

        // Wait for state update OR hover timer (unified event waiting)
        hive_select_source sources[] = {
            [SEL_STATE] = {HIVE_SEL_BUS, .bus = s_state_bus},
            [SEL_HOVER_TIMER] = {HIVE_SEL_IPC,
                                 .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER,
                                         hover_timer}},
        };

        hive_select_result result;
        // Only include hover timer source when hovering
        size_t num_sources = hovering ? 2 : 1;
        hive_select(sources, num_sources, &result, -1);

        if (result.index == SEL_HOVER_TIMER) {
            // Hover timer fired - advance to next waypoint (loops back to 0)
            hovering = false;
            hover_timer = TIMER_ID_INVALID;
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
        state_estimate_t state;
        assert(result.bus.len == sizeof(state));
        memcpy(&state, result.bus.data, sizeof(state));

        // Check arrival and start hover timer
        if (!hovering && check_arrival(wp, &state)) {
            HIVE_LOG_INFO("[WPT] Arrived at waypoint %d - hovering",
                          waypoint_index);
            hive_timer_after(WAYPOINT_HOVER_TIME_US, &hover_timer);
            hovering = true;
        }
    }
}

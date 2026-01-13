// Waypoint actor - Waypoint navigation manager
//
// Subscribes to state bus to monitor position, publishes current target
// to position target bus. Advances through waypoint list when arrival detected.

#include "waypoint_actor.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_log.h"
#include <assert.h>
#include <stdbool.h>
#include <math.h>

// Waypoint definition
typedef struct {
    float x, y, z;  // Position (meters, world frame)
    float yaw;      // Heading (radians)
} waypoint_t;

#ifdef HAL_FIRST_FLIGHT_TEST
// First flight test: hover briefly at low altitude, then land
// Safe profile for initial hardware validation
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 0.25f, 0.0f},  // Hover at 0.25m (tethered test)
    {0.0f, 0.0f, 0.0f, 0.0f},   // Land
};
#define WAYPOINT_HOVER_TIME_US  (6 * 1000000)  // 6 seconds
#elif defined(PLATFORM_STEVAL_DRONE01)
// No GPS: altitude-only waypoints (x,y fixed at origin)
// Position actor sees zero error, so drone hovers in place.
// Conservative test profile: low altitudes, slow transitions
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 0.5f, 0.0f},   // 0.5m - start low
    {0.0f, 0.0f, 1.0f, 0.0f},   // 1.0m
    {0.0f, 0.0f, 1.5f, 0.0f},   // 1.5m - max height
    {0.0f, 0.0f, 1.0f, 0.0f},   // 1.0m - descend
};
#define WAYPOINT_HOVER_TIME_US  (5 * 1000000)  // 5 seconds
#else
// GPS available: full 3D waypoint navigation
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 1.0f, 0.0f},              // Start: origin, 1.0m
    {1.0f, 0.0f, 1.2f, 0.0f},              // Waypoint 1: +X, rise to 1.2m
    {1.0f, 1.0f, 1.4f, M_PI_F / 2.0f},     // Waypoint 2: corner, rise to 1.4m, face east
    {0.0f, 1.0f, 1.2f, M_PI_F},            // Waypoint 3: -X, drop to 1.2m, face south
    {0.0f, 0.0f, 1.0f, 0.0f},              // Return: origin, 1.0m, face north
};
#define WAYPOINT_HOVER_TIME_US  (200 * 1000)  // 200ms - fast for simulation
#endif

#define NUM_WAYPOINTS (sizeof(waypoints) / sizeof(waypoints[0]))

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
           (yaw_err < WAYPOINT_TOLERANCE_YAW) &&
           (vel < WAYPOINT_TOLERANCE_VEL);
}

void waypoint_actor(void *arg) {
    (void)arg;

    BUS_SUBSCRIBE(s_state_bus);

    // Wait for START signal from supervisor before beginning flight
    HIVE_LOG_INFO("[WPT] Waiting for supervisor START signal");
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, NOTIFY_FLIGHT_START, &msg, -1);
    HIVE_LOG_INFO("[WPT] START received - beginning flight sequence");

    int waypoint_index = 0;
    timer_id hover_timer = TIMER_ID_INVALID;
    bool hovering = false;

    while (1) {
        const waypoint_t *wp = &waypoints[waypoint_index];

        // Publish current target
        position_target_t target = {
            .x = wp->x,
            .y = wp->y,
            .z = wp->z,
            .yaw = wp->yaw
        };
        hive_bus_publish(s_position_target_bus, &target, sizeof(target));

        // Wait for state update
        state_estimate_t state;
        BUS_READ_WAIT(s_state_bus, &state);

        // Check for hover timer expiry
        if (hovering) {
            hive_message timer_msg;
            if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER,
                                                    HIVE_TAG_ANY, &timer_msg, 0))) {
                // Timer fired - advance to next waypoint
                hovering = false;
                hover_timer = TIMER_ID_INVALID;

#ifdef HAL_FIRST_FLIGHT_TEST
                // First flight test: advance once, then stay landed
                if (waypoint_index < NUM_WAYPOINTS - 1) {
                    waypoint_index++;
                    if (waypoint_index == NUM_WAYPOINTS - 1) {
                        HIVE_LOG_INFO("[WPT] LANDED - test complete");
                    } else {
                        HIVE_LOG_INFO("[WPT] Advancing to waypoint %d", waypoint_index);
                    }
                }
#else
                // Normal operation: loop through waypoints
                waypoint_index = (waypoint_index + 1) % NUM_WAYPOINTS;
                HIVE_LOG_INFO("[WPT] Advancing to waypoint %d: (%.1f, %.1f, %.1f) yaw=%.0f deg",
                              waypoint_index, waypoints[waypoint_index].x,
                              waypoints[waypoint_index].y,
                              waypoints[waypoint_index].z,
                              waypoints[waypoint_index].yaw * RAD_TO_DEG);
#endif
            }
        }

        // Check arrival and start hover timer
        if (!hovering && check_arrival(wp, &state)) {
            HIVE_LOG_INFO("[WPT] Arrived at waypoint %d - hovering", waypoint_index);
            hive_timer_after(WAYPOINT_HOVER_TIME_US, &hover_timer);
            hovering = true;
        }
    }
}

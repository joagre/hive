// Waypoint actor - Waypoint navigation manager
//
// Subscribes to state bus to monitor position, publishes current target
// to position target bus. Advances through waypoint list when arrival detected.

#include "waypoint_actor.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "hal_config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
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
#define FIRST_FLIGHT_HOVER_TICKS  750  // 3 seconds at 250Hz
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
#else
// GPS available: full 3D waypoint navigation
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 1.0f, 0.0f},              // Start: origin, 1.0m
    {1.0f, 0.0f, 1.2f, 0.0f},              // Waypoint 1: +X, rise to 1.2m
    {1.0f, 1.0f, 1.4f, M_PI_F / 2.0f},     // Waypoint 2: corner, rise to 1.4m, face east
    {0.0f, 1.0f, 1.2f, M_PI_F},            // Waypoint 3: -X, drop to 1.2m, face south
    {0.0f, 0.0f, 1.0f, 0.0f},              // Return: origin, 1.0m, face north
};
#endif

#define NUM_WAYPOINTS (sizeof(waypoints) / sizeof(waypoints[0]))

static bus_id s_state_bus;
static bus_id s_position_target_bus;

void waypoint_actor_init(bus_id state_bus, bus_id position_target_bus) {
    s_state_bus = state_bus;
    s_position_target_bus = position_target_bus;
}

void waypoint_actor(void *arg) {
    (void)arg;

    BUS_SUBSCRIBE(s_state_bus);

    // Wait for START signal from supervisor before beginning flight
    // Use selective receive - match specific notification tag
    HIVE_LOG_INFO("[WPT] Waiting for supervisor START signal");
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, NOTIFY_FLIGHT_START, &msg, -1);
    HIVE_LOG_INFO("[WPT] START received - beginning flight sequence");

    int waypoint_index = 0;
    int hover_ticks = 0;
    int count = 0;

    while (1) {
        state_estimate_t state;

        // Block until state available
        BUS_READ_WAIT(s_state_bus, &state);

        // Get current waypoint
        const waypoint_t *wp = &waypoints[waypoint_index];

        // Publish current target
        position_target_t target = {
            .x = wp->x,
            .y = wp->y,
            .z = wp->z,
            .yaw = wp->yaw
        };
        hive_bus_publish(s_position_target_bus, &target, sizeof(target));

        // Check arrival
        {
            // Compute horizontal distance to waypoint
            float dx = wp->x - state.x;
            float dy = wp->y - state.y;
            float dist_xy = sqrtf(dx * dx + dy * dy);

            // Compute altitude error
            float alt_err = fabsf(wp->z - state.altitude);

            // Compute yaw error (normalized to [-π, π])
            float yaw_err = fabsf(NORMALIZE_ANGLE(wp->yaw - state.yaw));

            // Compute horizontal velocity magnitude
            float vel = sqrtf(state.x_velocity * state.x_velocity +
                              state.y_velocity * state.y_velocity);

            // Check if within tolerance (position, altitude, heading, AND nearly stopped)
            bool arrived = (dist_xy < WAYPOINT_TOLERANCE_XY) &&
                           (alt_err < WAYPOINT_TOLERANCE_Z) &&
                           (yaw_err < WAYPOINT_TOLERANCE_YAW) &&
                           (vel < WAYPOINT_TOLERANCE_VEL);

            if (arrived) {
                hover_ticks++;
#ifdef HAL_FIRST_FLIGHT_TEST
                // First flight test: advance once, then stay landed
                int hover_threshold = FIRST_FLIGHT_HOVER_TICKS;
                bool at_final_waypoint = (waypoint_index == NUM_WAYPOINTS - 1);
#else
                // Normal operation: loop through waypoints
                int hover_threshold = HAL_WAYPOINT_HOVER_TICKS;
                bool at_final_waypoint = false;  // Always advance (will wrap)
#endif
                if (hover_ticks >= hover_threshold && !at_final_waypoint) {
                    waypoint_index = (waypoint_index + 1) % NUM_WAYPOINTS;
                    hover_ticks = 0;
#ifdef HAL_FIRST_FLIGHT_TEST
                    if (waypoint_index == NUM_WAYPOINTS - 1) {
                        HIVE_LOG_INFO("[WPT] LANDED - test complete");
                    } else {
                        HIVE_LOG_INFO("[WPT] Advancing to waypoint %d: (%.1f, %.1f, %.1f)",
                                      waypoint_index, waypoints[waypoint_index].x,
                                      waypoints[waypoint_index].y,
                                      waypoints[waypoint_index].z);
                    }
#else
                    HIVE_LOG_INFO("[WPT] Advancing to waypoint %d: (%.1f, %.1f, %.1f) yaw=%.0f deg",
                                  waypoint_index, waypoints[waypoint_index].x,
                                  waypoints[waypoint_index].y,
                                  waypoints[waypoint_index].z,
                                  waypoints[waypoint_index].yaw * RAD_TO_DEG);
#endif
                }
            } else {
                hover_ticks = 0;  // Reset if we leave tolerance
            }

            // Debug output
            if (DEBUG_THROTTLE(count, DEBUG_PRINT_INTERVAL)) {
                HIVE_LOG_DEBUG("[WPT] wp=%d/%d xy=%.2f z=%.2f yaw=%.1f deg hover=%d%s",
                               waypoint_index, (int)NUM_WAYPOINTS - 1, dist_xy, alt_err,
                               yaw_err * RAD_TO_DEG, hover_ticks,
                               arrived ? " ARRIVED" : "");
            }
        }
    }
}

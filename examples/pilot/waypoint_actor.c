// Waypoint actor - Waypoint navigation manager
//
// Subscribes to state bus to monitor position, publishes current target
// to target bus. Advances through waypoint list when arrival detected.

#include "waypoint_actor.h"
#include "types.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include <stdbool.h>
#include <math.h>

// Waypoint definition
typedef struct {
    float x, y, z;  // Position (meters, world frame)
    float yaw;      // Heading (radians)
} waypoint_t;

// Demo waypoint route: square pattern with gentle altitude changes
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 1.0f, 0.0f},              // Start: origin, 1.0m
    {1.0f, 0.0f, 1.2f, 0.0f},              // Waypoint 1: +X, rise to 1.2m
    {1.0f, 1.0f, 1.4f, M_PI_F / 2.0f},     // Waypoint 2: corner, rise to 1.4m, face east
    {0.0f, 1.0f, 1.2f, M_PI_F},            // Waypoint 3: -X, drop to 1.2m, face south
    {0.0f, 0.0f, 1.0f, 0.0f},              // Return: origin, 1.0m, face north
};

#define NUM_WAYPOINTS (sizeof(waypoints) / sizeof(waypoints[0]))

static bus_id s_state_bus;
static bus_id s_target_bus;

void waypoint_actor_init(bus_id state_bus, bus_id target_bus) {
    s_state_bus = state_bus;
    s_target_bus = target_bus;
}

void waypoint_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_state_bus);

    int waypoint_index = 0;
    int hover_ticks = 0;
    int count = 0;

    while (1) {
        state_estimate_t state;

        // Get current waypoint
        const waypoint_t *wp = &waypoints[waypoint_index];

        // Publish current target
        position_target_t target = {
            .x = wp->x,
            .y = wp->y,
            .z = wp->z,
            .yaw = wp->yaw
        };
        hive_bus_publish(s_target_bus, &target, sizeof(target));

        // Check arrival if we have state data
        if (BUS_READ(s_state_bus, &state)) {
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
                // Advance after hovering at waypoint (loop back to start)
                if (hover_ticks >= WAYPOINT_HOVER_TICKS) {
                    waypoint_index = (waypoint_index + 1) % NUM_WAYPOINTS;
                    hover_ticks = 0;
                    HIVE_LOG_INFO("[WPT] Advancing to waypoint %d: (%.1f, %.1f, %.1f) yaw=%.0f deg",
                                  waypoint_index, waypoints[waypoint_index].x,
                                  waypoints[waypoint_index].y,
                                  waypoints[waypoint_index].z,
                                  waypoints[waypoint_index].yaw * RAD_TO_DEG);
                }
            } else {
                hover_ticks = 0;  // Reset if we leave tolerance
            }

            // Debug output
            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                HIVE_LOG_DEBUG("[WPT] wp=%d/%d xy=%.2f z=%.2f yaw=%.1f deg hover=%d%s",
                               waypoint_index, (int)NUM_WAYPOINTS - 1, dist_xy, alt_err,
                               yaw_err * RAD_TO_DEG, hover_ticks,
                               arrived ? " ARRIVED" : "");
            }
        }

        hive_yield();
    }
}

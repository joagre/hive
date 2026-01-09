// Waypoint actor - Waypoint navigation manager
//
// Subscribes to state bus to monitor position, publishes current target
// to target bus. Advances through waypoint list when arrival detected.

#include "waypoint_actor.h"
#include "types.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Waypoint definition
typedef struct {
    float x, y;   // Position (meters, world frame)
    float yaw;    // Heading (radians)
} waypoint_t;

// Demo waypoint route: square pattern
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 0.0f},              // Start: origin, facing north
    {1.0f, 0.0f, 0.0f},              // Waypoint 1: +X, north
    {1.0f, 1.0f, M_PI_F / 2.0f},     // Waypoint 2: corner, face east
    {0.0f, 1.0f, M_PI_F},            // Waypoint 3: -X, face south
    {0.0f, 0.0f, 0.0f},              // Return: origin, face north
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
        size_t len;

        // Get current waypoint
        const waypoint_t *wp = &waypoints[waypoint_index];

        // Publish current target
        position_target_t target = {
            .x = wp->x,
            .y = wp->y,
            .yaw = wp->yaw
        };
        hive_bus_publish(s_target_bus, &target, sizeof(target));

        // Check arrival if we have state data
        if (hive_bus_read(s_state_bus, &state, sizeof(state), &len).code == HIVE_OK) {
            // Compute distance to waypoint
            float dx = wp->x - state.x;
            float dy = wp->y - state.y;
            float dist = sqrtf(dx * dx + dy * dy);

            // Compute yaw error (normalized to [-π, π])
            float yaw_err = fabsf(NORMALIZE_ANGLE(wp->yaw - state.yaw));

            // Compute horizontal velocity magnitude
            float vel = sqrtf(state.x_velocity * state.x_velocity +
                              state.y_velocity * state.y_velocity);

            // Check if within tolerance (position, heading, AND nearly stopped)
            bool arrived = (dist < WAYPOINT_TOLERANCE_XY) &&
                           (yaw_err < WAYPOINT_TOLERANCE_YAW) &&
                           (vel < WAYPOINT_TOLERANCE_VEL);

            if (arrived) {
                hover_ticks++;
                // Advance after hovering at waypoint (loop back to start)
                if (hover_ticks >= WAYPOINT_HOVER_TICKS) {
                    waypoint_index = (waypoint_index + 1) % NUM_WAYPOINTS;
                    hover_ticks = 0;
                    printf("[WPT] Advancing to waypoint %d: (%.1f, %.1f) yaw=%.0f°\n",
                           waypoint_index, waypoints[waypoint_index].x,
                           waypoints[waypoint_index].y,
                           waypoints[waypoint_index].yaw * RAD_TO_DEG);
                }
            } else {
                hover_ticks = 0;  // Reset if we leave tolerance
            }

            // Debug output
            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                printf("[WPT] wp=%d/%d dist=%.2f yaw_err=%.1f° hover=%d%s\n",
                       waypoint_index, (int)NUM_WAYPOINTS - 1, dist,
                       yaw_err * RAD_TO_DEG, hover_ticks,
                       arrived ? " ARRIVED" : "");
            }
        }

        hive_yield();
    }
}

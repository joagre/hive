// Waypoint actor - Waypoint navigation manager
//
// Manages a list of waypoints and publishes the current target position
// to the position target bus. Monitors state bus to detect waypoint arrival.

#ifndef WAYPOINT_ACTOR_H
#define WAYPOINT_ACTOR_H

#include "hive_runtime.h"

// Init function - extracts bus IDs from pilot_buses
void *waypoint_actor_init(void *init_args);

// Actor entry point
void waypoint_actor(void *args, const hive_spawn_info *siblings,
                    size_t sibling_count);

#endif // WAYPOINT_ACTOR_H

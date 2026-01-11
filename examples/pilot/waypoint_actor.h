// Waypoint actor - Waypoint navigation manager
//
// Manages a list of waypoints and publishes the current target position
// to the position target bus. Monitors state bus to detect waypoint arrival.

#ifndef WAYPOINT_ACTOR_H
#define WAYPOINT_ACTOR_H

#include "hive_bus.h"

// Initialize the waypoint actor module with bus IDs.
// Must be called before spawning the actor.
void waypoint_actor_init(bus_id state_bus, bus_id position_target_bus);

// Actor entry point - spawn this with hive_spawn()
void waypoint_actor(void *arg);

#endif // WAYPOINT_ACTOR_H

// Position actor - Horizontal position hold control
//
// Subscribes to state bus, computes desired roll/pitch angles to hold
// target XY position, publishes attitude setpoints.

#ifndef POSITION_ACTOR_H
#define POSITION_ACTOR_H

#include "hive_bus.h"

// Initialize the position actor module with bus IDs.
// Must be called before spawning the actor.
void position_actor_init(bus_id state_bus, bus_id attitude_setpoint_bus, bus_id position_target_bus);

// Actor entry point - spawn this with hive_spawn()
void position_actor(void *arg);

#endif // POSITION_ACTOR_H

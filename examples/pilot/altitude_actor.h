// Altitude actor - Altitude hold control
//
// Subscribes to state and target buses, runs altitude PID controller,
// publishes thrust commands to thrust bus.

#ifndef ALTITUDE_ACTOR_H
#define ALTITUDE_ACTOR_H

#include "hive_bus.h"

// Initialize the altitude actor module with bus IDs.
// Must be called before spawning the actor.
void altitude_actor_init(bus_id state_bus, bus_id thrust_bus, bus_id target_bus);

// Actor entry point - spawn this with hive_spawn()
void altitude_actor(void *arg);

#endif // ALTITUDE_ACTOR_H

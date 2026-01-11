// Rate actor - Angular rate stabilization
//
// Subscribes to state, thrust, and rate setpoint buses, runs rate PID
// controllers for roll/pitch/yaw, publishes torque commands to torque bus.

#ifndef RATE_ACTOR_H
#define RATE_ACTOR_H

#include "hive_bus.h"

// Initialize the rate actor module with bus IDs.
// Must be called before spawning the actor.
void rate_actor_init(bus_id state_bus, bus_id thrust_bus,
                     bus_id rate_setpoint_bus, bus_id torque_bus);

// Actor entry point - spawn this with hive_spawn()
void rate_actor(void *arg);

#endif // RATE_ACTOR_H

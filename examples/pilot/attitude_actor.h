// Attitude actor - Rate stabilization
//
// Subscribes to state, thrust, and rate setpoint buses, runs rate PID
// controllers for roll/pitch/yaw, publishes torque commands to torque bus.

#ifndef ATTITUDE_ACTOR_H
#define ATTITUDE_ACTOR_H

#include "hive_bus.h"

// Initialize the attitude actor module with bus IDs.
// Must be called before spawning the actor.
void attitude_actor_init(bus_id state_bus, bus_id thrust_bus,
                         bus_id rate_setpoint_bus, bus_id torque_bus);

// Actor entry point - spawn this with hive_spawn()
void attitude_actor(void *arg);

#endif // ATTITUDE_ACTOR_H

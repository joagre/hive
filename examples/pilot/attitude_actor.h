// Attitude actor - Attitude angle control
//
// Subscribes to state and attitude setpoint buses, runs attitude PID controllers
// for roll/pitch/yaw, publishes rate setpoints.

#ifndef ATTITUDE_ACTOR_H
#define ATTITUDE_ACTOR_H

#include "hive_bus.h"

// Initialize the attitude actor module with bus IDs.
// Must be called before spawning the actor.
void attitude_actor_init(bus_id state_bus, bus_id attitude_setpoint_bus, bus_id rate_setpoint_bus);

// Actor entry point - spawn this with hive_spawn()
void attitude_actor(void *arg);

#endif // ATTITUDE_ACTOR_H

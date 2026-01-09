// Angle actor - Attitude angle control
//
// Subscribes to state bus, runs angle PID controllers for roll/pitch/yaw,
// publishes rate setpoints to rate setpoint bus.

#ifndef ANGLE_ACTOR_H
#define ANGLE_ACTOR_H

#include "hive_bus.h"

// Initialize the angle actor module with bus IDs.
// Must be called before spawning the actor.
void angle_actor_init(bus_id state_bus, bus_id rate_setpoint_bus);

// Actor entry point - spawn this with hive_spawn()
void angle_actor(void *arg);

#endif // ANGLE_ACTOR_H

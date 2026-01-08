// Attitude actor - Inner loop rate control
//
// Subscribes to IMU bus and thrust bus, runs rate PID controllers
// for roll/pitch/yaw, publishes motor commands to motor bus.

#ifndef ATTITUDE_ACTOR_H
#define ATTITUDE_ACTOR_H

#include "hive_bus.h"

// Initialize the attitude actor module with bus IDs.
// Must be called before spawning the actor.
void attitude_actor_init(bus_id imu_bus, bus_id thrust_bus, bus_id motor_bus);

// Actor entry point - spawn this with hive_spawn()
void attitude_actor(void *arg);

#endif // ATTITUDE_ACTOR_H

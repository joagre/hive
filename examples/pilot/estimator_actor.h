// Estimator actor - Sensor fusion and state estimation
//
// Subscribes to IMU bus (raw sensor data), applies sensor fusion,
// publishes state estimates to state bus.
//
// For Webots: Mostly pass-through since inertial_unit provides clean attitude.
//             Computes vertical velocity by differentiating GPS altitude.
//
// For real hardware: Would implement complementary filter or Kalman filter
//                    to fuse accelerometer and gyroscope for attitude.

#ifndef ESTIMATOR_ACTOR_H
#define ESTIMATOR_ACTOR_H

#include "hive_bus.h"

// Initialize the estimator actor module with bus IDs.
// Must be called before spawning the actor.
void estimator_actor_init(bus_id imu_bus, bus_id state_bus);

// Actor entry point - spawn this with hive_spawn()
void estimator_actor(void *arg);

#endif // ESTIMATOR_ACTOR_H

// Sensor actor - Hardware sensor reading
//
// Reads IMU/GPS sensors and publishes to IMU bus.

#ifndef SENSOR_ACTOR_H
#define SENSOR_ACTOR_H

#include "hive_bus.h"
#include "types.h"

// Platform read function type - provided by platform layer
typedef void (*read_imu_fn)(imu_data_t *imu);

// Initialize the sensor actor module with bus ID and platform function.
// Must be called before spawning the actor.
void sensor_actor_init(bus_id imu_bus, read_imu_fn read_imu);

// Actor entry point - spawn this with hive_spawn()
void sensor_actor(void *arg);

#endif // SENSOR_ACTOR_H

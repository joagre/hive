// Sensor actor - Timer-driven sensor reading
//
// Periodically reads raw sensors via HAL, publishes to sensor bus.
// Sensor fusion is done by the estimator actor.

#ifndef SENSOR_ACTOR_H
#define SENSOR_ACTOR_H

#include "hive_runtime.h"

// Init function - extracts bus IDs from pilot_buses
void *sensor_actor_init(void *init_args);

// Actor entry point
void sensor_actor(void *args, const hive_spawn_info_t *siblings,
                  size_t sibling_count);

#endif // SENSOR_ACTOR_H

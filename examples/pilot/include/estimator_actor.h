// Estimator actor - Attitude estimation and state computation
//
// Subscribes to sensor bus, runs complementary filter for attitude,
// computes velocities, publishes state estimate.

#ifndef ESTIMATOR_ACTOR_H
#define ESTIMATOR_ACTOR_H

#include "hive_runtime.h"

// Init function - extracts bus IDs from pilot_buses
void *estimator_actor_init(void *init_args);

// Actor entry point
void estimator_actor(void *args, const hive_spawn_info *siblings,
                     size_t sibling_count);

#endif // ESTIMATOR_ACTOR_H

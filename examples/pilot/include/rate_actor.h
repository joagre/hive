// Rate actor - Angular rate stabilization
//
// Subscribes to state, thrust, and rate setpoint buses, runs rate PID
// controllers for roll/pitch/yaw, publishes torque commands to torque bus.

#ifndef RATE_ACTOR_H
#define RATE_ACTOR_H

#include "hive_runtime.h"

// Init function - extracts bus IDs from pilot_buses
void *rate_actor_init(void *init_args);

// Actor entry point
void rate_actor(void *args, const hive_spawn_info *siblings,
                size_t sibling_count);

#endif // RATE_ACTOR_H

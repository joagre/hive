// Attitude actor - Attitude angle control
//
// Subscribes to state and attitude setpoint buses, runs attitude PID
// controllers for roll/pitch/yaw, publishes rate setpoints.

#ifndef ATTITUDE_ACTOR_H
#define ATTITUDE_ACTOR_H

#include "hive_runtime.h"

// Init function - extracts bus IDs from pilot_buses
void *attitude_actor_init(void *init_args);

// Actor entry point
void attitude_actor(void *args, const hive_spawn_info *siblings,
                    size_t sibling_count);

#endif // ATTITUDE_ACTOR_H

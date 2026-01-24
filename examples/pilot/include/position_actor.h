// Position actor - Horizontal position hold control
//
// Subscribes to state bus, computes desired roll/pitch angles to hold
// target XY position, publishes attitude setpoints.

#ifndef POSITION_ACTOR_H
#define POSITION_ACTOR_H

#include "hive_runtime.h"

// Init function - extracts bus IDs from pilot_buses
void *position_actor_init(void *init_args);

// Actor entry point
void position_actor(void *args, const hive_spawn_info_t *siblings,
                    size_t sibling_count);

#endif // POSITION_ACTOR_H

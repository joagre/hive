// Altitude actor - Altitude hold control
//
// Subscribes to state and position target buses, runs altitude PID controller,
// publishes thrust commands to thrust bus.
//
// Supports controlled landing via NOTIFY_LANDING message. When received,
// descends at a fixed rate until touchdown, then notifies flight manager.

#ifndef ALTITUDE_ACTOR_H
#define ALTITUDE_ACTOR_H

#include "hive_runtime.h"

// Init function - extracts bus IDs from pilot_buses
void *altitude_actor_init(void *init_args);

// Actor entry point
void altitude_actor(void *args, const hive_spawn_info_t *siblings,
                    size_t sibling_count);

#endif // ALTITUDE_ACTOR_H

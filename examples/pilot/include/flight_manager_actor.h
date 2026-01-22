// Flight manager actor - Flight authority and safety monitoring
//
// Controls flight lifecycle:
// 1. Startup delay (real hardware only)
// 2. Open log file (ARM phase)
// 3. Send START to waypoint actor
// 4. Periodic log sync (every 4 seconds)
// 5. Flight duration timer
// 6. Send LANDING to altitude actor
// 7. Wait for LANDED, then send STOP to motor actor
// 8. Close log file (DISARM phase)
//
// Uses sibling info to find waypoint, altitude, motor actors.

#ifndef FLIGHT_MANAGER_ACTOR_H
#define FLIGHT_MANAGER_ACTOR_H

#include "hive_runtime.h"

// Init function - no buses needed, returns NULL
void *flight_manager_actor_init(void *init_args);

// Flight manager actor entry point
void flight_manager_actor(void *args, const hive_spawn_info *siblings,
                          size_t sibling_count);

#endif // FLIGHT_MANAGER_ACTOR_H

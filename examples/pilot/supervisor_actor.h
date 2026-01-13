// Supervisor actor - Startup coordination and safety monitoring
//
// Handles startup delay and coordinates flight start.
// Sends START notification to waypoint actor after delay.

#ifndef SUPERVISOR_ACTOR_H
#define SUPERVISOR_ACTOR_H

#include "hive_actor.h"

// Initialize supervisor with actor IDs to coordinate
void supervisor_actor_init(actor_id waypoint_actor, actor_id motor_actor);

// Supervisor actor entry point
void supervisor_actor(void *arg);

// Startup delay in seconds (real hardware only)
#ifndef SIMULATED_TIME
#define SUPERVISOR_STARTUP_DELAY_US  (60 * 1000000)  // 60 seconds
#endif

#endif // SUPERVISOR_ACTOR_H

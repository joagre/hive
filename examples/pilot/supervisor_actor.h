// Supervisor actor - Flight authority and safety monitoring
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

#ifndef SUPERVISOR_ACTOR_H
#define SUPERVISOR_ACTOR_H

#include "hive_actor.h"

// Initialize supervisor with actor IDs to coordinate
void supervisor_actor_init(actor_id waypoint_actor, actor_id altitude_actor, actor_id motor_actor);

// Supervisor actor entry point
void supervisor_actor(void *arg);

// Startup delay in seconds (real hardware only)
#ifndef SIMULATED_TIME
#define SUPERVISOR_STARTUP_DELAY_US  (60 * 1000000)  // 60 seconds
#endif

#endif // SUPERVISOR_ACTOR_H

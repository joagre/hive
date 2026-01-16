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
// Uses name registry:
// - Registers self as "flight_manager"
// - Uses whereis() to find "waypoint", "altitude", "motor"

#ifndef FLIGHT_MANAGER_ACTOR_H
#define FLIGHT_MANAGER_ACTOR_H

#include "hive_actor.h"

// Initialize flight manager (no parameters - uses whereis for targets)
void flight_manager_actor_init(void);

// Flight manager actor entry point
void flight_manager_actor(void *arg);

// Startup delay in seconds (real hardware only)
#ifndef SIMULATED_TIME
#define FLIGHT_MANAGER_STARTUP_DELAY_US (60 * 1000000) // 60 seconds
#endif

#endif // FLIGHT_MANAGER_ACTOR_H

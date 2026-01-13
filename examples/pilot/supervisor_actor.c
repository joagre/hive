// Supervisor actor - Startup coordination and safety monitoring
//
// Handles startup delay before flight begins. Sends START notification
// to waypoint actor after delay completes.

#include "supervisor_actor.h"
#include "notifications.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_log.h"

static actor_id s_waypoint_actor;
static actor_id s_motor_actor;

void supervisor_actor_init(actor_id waypoint_actor, actor_id motor_actor) {
    s_waypoint_actor = waypoint_actor;
    s_motor_actor = motor_actor;
}

void supervisor_actor(void *arg) {
    (void)arg;

#ifndef SIMULATED_TIME
    // Real hardware: wait for startup delay before allowing flight
    HIVE_LOG_INFO("[SUP] Startup delay: 60 seconds");

    // Sleep in 10-second intervals with progress logging
    for (int i = 6; i > 0; i--) {
        hive_sleep(10 * 1000000);  // 10 seconds
        if (i > 1) {
            HIVE_LOG_INFO("[SUP] Startup delay: %d seconds remaining", (i - 1) * 10);
        }
    }

    HIVE_LOG_INFO("[SUP] Startup delay complete - sending START");
#else
    // Simulation: no delay needed
    HIVE_LOG_INFO("[SUP] Simulation mode - sending START immediately");
#endif

    // Notify waypoint actor to begin flight sequence
    // Tag carries the notification type, no payload needed
    hive_ipc_notify(s_waypoint_actor, NOTIFY_FLIGHT_START, NULL, 0);
    HIVE_LOG_INFO("[SUP] Flight authorized");

#ifndef SIMULATED_TIME
    // Flight window: hard cutoff after 5 seconds
    HIVE_LOG_INFO("[SUP] Flight window: 5 seconds");
    hive_sleep(5 * 1000000);  // 5 seconds

    // Send STOP to motor actor
    hive_ipc_notify(s_motor_actor, NOTIFY_FLIGHT_STOP, NULL, 0);
    HIVE_LOG_INFO("[SUP] Flight window expired - motors stopped");
#endif

    hive_exit();
}

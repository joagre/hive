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

#include "supervisor_actor.h"
#include "notifications.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_log.h"
#include "hive_static_config.h"

// Flight duration per profile (supervisor decides when to land)
#if FLIGHT_PROFILE == FLIGHT_PROFILE_FIRST_TEST
#define FLIGHT_DURATION_US  (10 * 1000000)  // 10 seconds
#elif FLIGHT_PROFILE == FLIGHT_PROFILE_ALTITUDE
#define FLIGHT_DURATION_US  (40 * 1000000)  // 40 seconds
#elif FLIGHT_PROFILE == FLIGHT_PROFILE_FULL_3D
#define FLIGHT_DURATION_US  (60 * 1000000)  // 60 seconds
#else
#define FLIGHT_DURATION_US  (20 * 1000000)  // Default: 20 seconds
#endif

// Log sync interval (4 seconds)
#define LOG_SYNC_INTERVAL_US (4 * 1000000)

static actor_id s_waypoint_actor;
static actor_id s_altitude_actor;
static actor_id s_motor_actor;

void supervisor_actor_init(actor_id waypoint_actor, actor_id altitude_actor, actor_id motor_actor) {
    s_waypoint_actor = waypoint_actor;
    s_altitude_actor = altitude_actor;
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

    HIVE_LOG_INFO("[SUP] Startup delay complete");
#else
    // Simulation: no delay needed
    HIVE_LOG_INFO("[SUP] Simulation mode");
#endif

    // === ARM PHASE: Open log file ===
    // On STM32, this erases the flash sector (blocks 1-4 seconds)
    HIVE_LOG_INFO("[SUP] Opening log file: %s", HIVE_LOG_FILE_PATH);
    hive_status log_status = hive_log_file_open(HIVE_LOG_FILE_PATH);
    if (HIVE_FAILED(log_status)) {
        HIVE_LOG_WARN("[SUP] Failed to open log file: %s", HIVE_ERR_STR(log_status));
    } else {
        HIVE_LOG_INFO("[SUP] Log file opened");
    }

    // Start periodic log sync timer (every 4 seconds)
    timer_id sync_timer;
    hive_timer_every(LOG_SYNC_INTERVAL_US, &sync_timer);

    // === FLIGHT PHASE ===
    // Notify waypoint actor to begin flight sequence
    HIVE_LOG_INFO("[SUP] Sending START - flight authorized");
    hive_ipc_notify(s_waypoint_actor, NOTIFY_FLIGHT_START, NULL, 0);

    // Flight duration timer, then initiate controlled landing
    HIVE_LOG_INFO("[SUP] Flight duration: %.0f seconds", FLIGHT_DURATION_US / 1000000.0f);

    timer_id flight_timer;
    hive_timer_after(FLIGHT_DURATION_US, &flight_timer);

    // Event loop: handle sync timer and flight timer
    bool flight_timer_fired = false;
    while (!flight_timer_fired) {
        hive_message msg;
        hive_ipc_recv(&msg, -1);

        if (msg.class == HIVE_MSG_TIMER) {
            if (msg.tag == sync_timer) {
                // Periodic log sync
                hive_log_file_sync();
            } else if (msg.tag == flight_timer) {
                flight_timer_fired = true;
            }
        }
    }

    HIVE_LOG_INFO("[SUP] Flight duration complete - initiating landing");
    hive_ipc_notify(s_altitude_actor, NOTIFY_LANDING, NULL, 0);

    // Wait for LANDED notification (keep syncing logs while waiting)
    bool landed = false;
    while (!landed) {
        hive_message msg;
        hive_ipc_recv(&msg, -1);

        if (msg.class == HIVE_MSG_TIMER && msg.tag == sync_timer) {
            hive_log_file_sync();
        } else if (msg.class == HIVE_MSG_NOTIFY && msg.tag == NOTIFY_FLIGHT_LANDED) {
            landed = true;
        }
    }

    HIVE_LOG_INFO("[SUP] Landing confirmed - stopping motors");

    // Send STOP to motor actor
    hive_ipc_notify(s_motor_actor, NOTIFY_FLIGHT_STOP, NULL, 0);

    // === DISARM PHASE: Close log file ===
    hive_timer_cancel(sync_timer);
    HIVE_LOG_INFO("[SUP] Closing log file...");
    hive_log_file_close();
    HIVE_LOG_INFO("[SUP] Log file closed");

    hive_exit();
}

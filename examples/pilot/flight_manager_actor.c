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

#include "flight_manager_actor.h"
#include "notifications.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_log.h"
#include "hive_static_config.h"
#include <assert.h>

// Flight duration per profile (flight manager decides when to land)
#if FLIGHT_PROFILE == FLIGHT_PROFILE_FIRST_TEST
#define FLIGHT_DURATION_US (10 * 1000000) // 10 seconds
#elif FLIGHT_PROFILE == FLIGHT_PROFILE_ALTITUDE
#define FLIGHT_DURATION_US (40 * 1000000) // 40 seconds
#elif FLIGHT_PROFILE == FLIGHT_PROFILE_FULL_3D
#define FLIGHT_DURATION_US (60 * 1000000) // 60 seconds
#else
#define FLIGHT_DURATION_US (20 * 1000000) // Default: 20 seconds
#endif

// Log sync interval (4 seconds)
#define LOG_SYNC_INTERVAL_US (4 * 1000000)

void flight_manager_actor_init(void) {
    // No initialization needed - targets looked up via whereis()
}

void flight_manager_actor(void *arg) {
    (void)arg;

    // Register self with name registry
    hive_status status = hive_register("flight_manager");
    assert(HIVE_SUCCEEDED(status));

#ifndef SIMULATED_TIME
    // Real hardware: wait for startup delay before allowing flight
    HIVE_LOG_INFO("[FLM] Startup delay: 60 seconds");

    // Sleep in 10-second intervals with progress logging
    for (int i = 6; i > 0; i--) {
        hive_sleep(10 * 1000000); // 10 seconds
        if (i > 1) {
            HIVE_LOG_INFO("[FLM] Startup delay: %d seconds remaining",
                          (i - 1) * 10);
        }
    }

    HIVE_LOG_INFO("[FLM] Startup delay complete");
#else
    // Simulation: flight_manager spawns last, so all targets already registered
    HIVE_LOG_INFO("[FLM] Simulation mode");
#endif

    // === ARM PHASE: Open log file ===
    // On STM32, this erases the flash sector (blocks 1-4 seconds)
    HIVE_LOG_INFO("[FLM] Opening log file: %s", HIVE_LOG_FILE_PATH);
    hive_status log_status = hive_log_file_open(HIVE_LOG_FILE_PATH);
    if (HIVE_FAILED(log_status)) {
        HIVE_LOG_WARN("[FLM] Failed to open log file: %s",
                      HIVE_ERR_STR(log_status));
    } else {
        HIVE_LOG_INFO("[FLM] Log file opened");
    }

    // Start periodic log sync timer (every 4 seconds)
    timer_id sync_timer;
    hive_timer_every(LOG_SYNC_INTERVAL_US, &sync_timer);

    // === FLIGHT PHASE ===
    // Look up waypoint actor and notify to begin flight sequence
    actor_id waypoint;
    status = hive_whereis("waypoint", &waypoint);
    assert(HIVE_SUCCEEDED(status));

    HIVE_LOG_INFO("[FLM] Sending START - flight authorized");
    hive_ipc_notify(waypoint, NOTIFY_FLIGHT_START, NULL, 0);

    // Flight duration timer, then initiate controlled landing
    HIVE_LOG_INFO("[FLM] Flight duration: %.0f seconds",
                  FLIGHT_DURATION_US / 1000000.0f);

    timer_id flight_timer;
    hive_timer_after(FLIGHT_DURATION_US, &flight_timer);

    // Event loop: handle sync timer and flight timer
    hive_recv_filter flight_filters[] = {
        {HIVE_SENDER_ANY, HIVE_MSG_TIMER, sync_timer},
        {HIVE_SENDER_ANY, HIVE_MSG_TIMER, flight_timer},
    };

    bool flight_timer_fired = false;
    while (!flight_timer_fired) {
        hive_message msg;
        size_t matched;
        hive_ipc_recv_match_any(flight_filters, 2, &msg, -1, &matched);

        if (matched == 0) {
            // Periodic log sync
            hive_log_file_sync();
        } else {
            flight_timer_fired = true;
        }
    }

    HIVE_LOG_INFO("[FLM] Flight duration complete - initiating landing");

    // Look up altitude actor and notify to begin landing
    actor_id altitude;
    status = hive_whereis("altitude", &altitude);
    assert(HIVE_SUCCEEDED(status));
    hive_ipc_notify(altitude, NOTIFY_LANDING, NULL, 0);

    // Wait for LANDED notification (keep syncing logs while waiting)
    hive_recv_filter landing_filters[] = {
        {HIVE_SENDER_ANY, HIVE_MSG_TIMER, sync_timer},
        {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, NOTIFY_FLIGHT_LANDED},
    };

    bool landed = false;
    while (!landed) {
        hive_message msg;
        size_t matched;
        hive_ipc_recv_match_any(landing_filters, 2, &msg, -1, &matched);

        if (matched == 0) {
            hive_log_file_sync();
        } else {
            landed = true;
        }
    }

    HIVE_LOG_INFO("[FLM] Landing confirmed - stopping motors");

    // Look up motor actor and send STOP
    actor_id motor;
    status = hive_whereis("motor", &motor);
    assert(HIVE_SUCCEEDED(status));
    hive_ipc_notify(motor, NOTIFY_FLIGHT_STOP, NULL, 0);

    // === DISARM PHASE: Close log file ===
    hive_timer_cancel(sync_timer);
    HIVE_LOG_INFO("[FLM] Closing log file...");
    hive_log_file_close();
    HIVE_LOG_INFO("[FLM] Log file closed");

    hive_exit();
}

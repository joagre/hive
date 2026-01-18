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

#include "flight_manager_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_select.h"
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

void *flight_manager_actor_init(void *init_args) {
    (void)init_args;
    // No bus state needed - uses sibling info for IPC targets
    return NULL;
}

void flight_manager_actor(void *args, const hive_spawn_info *siblings,
                          size_t sibling_count) {
    (void)args;

    // Look up sibling actors
    actor_id waypoint = hive_find_sibling(siblings, sibling_count, "waypoint");
    actor_id altitude = hive_find_sibling(siblings, sibling_count, "altitude");
    actor_id motor = hive_find_sibling(siblings, sibling_count, "motor");
    assert(waypoint != ACTOR_ID_INVALID && altitude != ACTOR_ID_INVALID &&
           motor != ACTOR_ID_INVALID);

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
    // Simulation mode
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
    // Notify waypoint actor to begin flight sequence
    HIVE_LOG_INFO("[FLM] Sending START - flight authorized");
    hive_ipc_notify(waypoint, NOTIFY_FLIGHT_START, NULL, 0);

    // Flight duration timer, then initiate controlled landing
    HIVE_LOG_INFO("[FLM] Flight duration: %.0f seconds",
                  FLIGHT_DURATION_US / 1000000.0f);

    timer_id flight_timer;
    hive_timer_after(FLIGHT_DURATION_US, &flight_timer);

    // Event loop: handle sync timer and flight timer using hive_select()
    enum { SEL_SYNC_TIMER, SEL_FLIGHT_TIMER };
    hive_select_source flight_sources[] = {
        [SEL_SYNC_TIMER] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY,
                                                  HIVE_MSG_TIMER, sync_timer}},
        [SEL_FLIGHT_TIMER] = {HIVE_SEL_IPC,
                              .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER,
                                      flight_timer}},
    };

    bool flight_timer_fired = false;
    while (!flight_timer_fired) {
        hive_select_result result;
        hive_select(flight_sources, 2, &result, -1);

        if (result.index == SEL_SYNC_TIMER) {
            hive_log_file_sync();
        } else {
            flight_timer_fired = true;
        }
    }

    HIVE_LOG_INFO("[FLM] Flight duration complete - initiating landing");

    // Notify altitude actor to begin landing
    hive_ipc_notify(altitude, NOTIFY_LANDING, NULL, 0);

    // Wait for LANDED notification (keep syncing logs while waiting)
    enum { SEL_SYNC, SEL_LANDED };
    hive_select_source landing_sources[] = {
        [SEL_SYNC] = {HIVE_SEL_IPC,
                      .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, sync_timer}},
        [SEL_LANDED] = {HIVE_SEL_IPC, .ipc = {altitude, HIVE_MSG_NOTIFY,
                                              NOTIFY_FLIGHT_LANDED}},
    };

    bool landed = false;
    while (!landed) {
        hive_select_result result;
        hive_select(landing_sources, 2, &result, -1);

        if (result.index == SEL_SYNC) {
            hive_log_file_sync();
        } else {
            landed = true;
        }
    }

    HIVE_LOG_INFO("[FLM] Landing confirmed - stopping motors");

    // Send STOP to motor actor
    hive_ipc_notify(motor, NOTIFY_FLIGHT_STOP, NULL, 0);

    // === DISARM PHASE: Close log file ===
    hive_timer_cancel(sync_timer);
    HIVE_LOG_INFO("[FLM] Closing log file...");
    hive_log_file_close();
    HIVE_LOG_INFO("[FLM] Log file closed");

    hive_exit();
}

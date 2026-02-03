// Flight manager actor - Flight authority and safety monitoring
//
// Controls flight lifecycle:
// 1. Startup delay (real hardware only)
// 2. Notify START to waypoint and motor actors
// 3. Periodic log sync (every 4 seconds)
// 4. Flight duration timer
// 5. Notify LANDING to altitude actor
// 6. Wait for LANDED, then notify STOP to motor actor
//
// Log file is opened/closed in pilot.c; flight_manager just syncs.
// Uses sibling info to find waypoint, altitude, motor actors.

#include "flight_manager_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_timer.h"
#include "hive_log.h"
#include "stack_profile.h"

// Flight duration per profile (flight manager decides when to land)
#if FLIGHT_PROFILE == FLIGHT_PROFILE_FIRST_TEST
#define FLIGHT_DURATION_US (6 * 1000000) // 6 seconds - short for safety
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

void flight_manager_actor(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;

    // Look up sibling actors
    hive_actor_id_t waypoint =
        hive_find_sibling(siblings, sibling_count, "waypoint");
    hive_actor_id_t altitude =
        hive_find_sibling(siblings, sibling_count, "altitude");
    hive_actor_id_t motor = hive_find_sibling(siblings, sibling_count, "motor");
    if (waypoint == HIVE_ACTOR_ID_INVALID ||
        altitude == HIVE_ACTOR_ID_INVALID || motor == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR(
            "[FLM] Missing siblings: waypoint=%s altitude=%s motor=%s",
            waypoint == HIVE_ACTOR_ID_INVALID ? "MISSING" : "ok",
            altitude == HIVE_ACTOR_ID_INVALID ? "MISSING" : "ok",
            motor == HIVE_ACTOR_ID_INVALID ? "MISSING" : "ok");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Log flight profile and battery for post-mortem verification
    HIVE_LOG_INFO("[FLM] Flight profile=%d duration=%.0fs battery=%.2fV",
                  FLIGHT_PROFILE, FLIGHT_DURATION_US / 1000000.0f,
                  hal_power_get_battery());

#ifndef SIMULATED_TIME
#if FLIGHT_PROFILE == FLIGHT_PROFILE_GROUND_TEST
    // Ground test mode - skip arming and startup delay (motors disabled)
    HIVE_LOG_INFO("[FLM] Ground test mode - no arming required");
#else
    // === WAIT FOR GO COMMAND FROM GROUND STATION ===
    // This is a CRITICAL safety gate - flight will NOT start without explicit
    // radio command from ground station. User must run:
    //   python3 tools/ground_station.py --go
    HIVE_LOG_INFO("[FLM] *** WAITING FOR GO COMMAND ***");
    HIVE_LOG_INFO("[FLM] Run: python3 tools/ground_station.py --go");

    // Find comms actor to receive GO notification
    hive_actor_id_t comms = hive_find_sibling(siblings, sibling_count, "comms");
    if (comms == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[FLM] comms sibling not found - cannot receive GO");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Wait indefinitely for GO notification
    hive_message_t go_msg;
    hive_status_t go_status =
        hive_ipc_recv_match(comms, HIVE_MSG_NOTIFY, NOTIFY_GO, &go_msg, -1);
    if (HIVE_FAILED(go_status)) {
        HIVE_LOG_ERROR("[FLM] GO wait failed: %s", HIVE_ERR_STR(go_status));
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    HIVE_LOG_INFO("[FLM] *** GO RECEIVED - STARTING COUNTDOWN ***");

    // 60 second countdown to step back safely
    HIVE_LOG_INFO("[FLM] Startup delay: 60 seconds - STEP BACK NOW");
    for (int i = 6; i > 0; i--) {
        hive_sleep(10 * 1000000); // 10 seconds
        if (i > 1) {
            HIVE_LOG_INFO("[FLM] Startup delay: %d seconds remaining",
                          (i - 1) * 10);
        }
    }
    HIVE_LOG_INFO("[FLM] Startup delay complete - FLIGHT STARTING");
#endif
#else
    // Simulation mode - no arming required
    HIVE_LOG_INFO("[FLM] Simulation mode");
#endif

    // Start periodic log sync timer (every 4 seconds)
    // Log file is opened/closed in pilot.c; we just sync periodically
    hive_timer_id_t sync_timer;
    hive_status_t status = hive_timer_every(LOG_SYNC_INTERVAL_US, &sync_timer);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[FLM] sync timer_every failed: %s",
                       HIVE_ERR_STR(status));
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // === FLIGHT PHASE ===
    // Notify motor and waypoint actors to begin flight
    HIVE_LOG_INFO("[FLM] Notifying START - flight authorized");
    hive_ipc_notify(motor, NOTIFY_FLIGHT_START, NULL, 0);
    hive_ipc_notify(waypoint, NOTIFY_FLIGHT_START, NULL, 0);

    // Flight duration timer, then initiate controlled landing
    HIVE_LOG_INFO("[FLM] Flight duration: %.0f seconds",
                  FLIGHT_DURATION_US / 1000000.0f);

    hive_timer_id_t flight_timer;
    status = hive_timer_after(FLIGHT_DURATION_US, &flight_timer);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[FLM] flight timer_after failed: %s",
                       HIVE_ERR_STR(status));
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Event loop: handle sync timer and flight timer using hive_select()
    enum { SEL_SYNC_TIMER, SEL_FLIGHT_TIMER };
    hive_select_source_t flight_sources[] = {
        [SEL_SYNC_TIMER] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY,
                                                  HIVE_MSG_TIMER, sync_timer}},
        [SEL_FLIGHT_TIMER] = {HIVE_SEL_IPC,
                              .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER,
                                      flight_timer}},
    };

    bool flight_timer_fired = false;
    while (!flight_timer_fired) {
        hive_select_result_t result;
        status = hive_select(flight_sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[FLM] select (flight) failed: %s",
                           HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_SYNC_TIMER) {
            hive_log_file_sync();
        } else {
            flight_timer_fired = true;
        }
    }

    HIVE_LOG_INFO("[FLM] Flight duration complete - initiating landing");

    // Notify altitude actor to begin landing
    hive_ipc_notify(altitude, NOTIFY_LANDING, NULL, 0);

    // Landing timeout - don't wait forever if something goes wrong
    hive_timer_id_t landing_timeout;
    hive_timer_after(10 * 1000000, &landing_timeout); // 10 second max

    // Wait for LANDED notification (keep syncing logs while waiting)
    enum { SEL_SYNC, SEL_LANDED, SEL_TIMEOUT };
    hive_select_source_t landing_sources[] = {
        [SEL_SYNC] = {HIVE_SEL_IPC,
                      .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, sync_timer}},
        [SEL_LANDED] = {HIVE_SEL_IPC, .ipc = {altitude, HIVE_MSG_NOTIFY,
                                              NOTIFY_FLIGHT_LANDED}},
        [SEL_TIMEOUT] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER,
                                               landing_timeout}},
    };

    bool landed = false;
    while (!landed) {
        hive_select_result_t result;
        status = hive_select(landing_sources, 3, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[FLM] select (landing) failed: %s",
                           HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_SYNC) {
            hive_log_file_sync();
        } else if (result.index == SEL_TIMEOUT) {
            HIVE_LOG_WARN("[FLM] Landing timeout - forcing shutdown");
            landed = true;
        } else {
            landed = true;
        }
    }

    hive_timer_cancel(landing_timeout);
    HIVE_LOG_INFO("[FLM] Landing confirmed - stopping motors");

    // Notify STOP to motor actor
    hive_ipc_notify(motor, NOTIFY_FLIGHT_STOP, NULL, 0);

    // Cancel sync timer (log file closed in pilot.c)
    hive_timer_cancel(sync_timer);

    stack_profile_capture("flight_mgr");
    stack_profile_request();

    return;
}

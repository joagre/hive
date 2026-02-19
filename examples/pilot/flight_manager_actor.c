// Flight manager actor - Flight lifecycle state machine
//
// Looping state machine supporting multiple flights without power cycling.
// States: IDLE -> PREFLIGHT -> ARMED -> FLYING -> LANDING -> LANDED -> IDLE
//
// See docs/flight_manager_lifecycle.md for detailed specification.

#include "flight_manager_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "tunable_params.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_timer.h"
#include "hive_log.h"
#include "stack_profile.h"
#include <stdbool.h>

// Flight duration and landing timeout defined in config.h:
// FLIGHT_DURATION_US, LANDING_TIMEOUT_US

// Flight manager state
typedef enum {
    FM_STATE_IDLE = 0,
    FM_STATE_PREFLIGHT = 1,
    FM_STATE_ARMED = 2,
    FM_STATE_FLYING = 3,
    FM_STATE_LANDING = 4,
    FM_STATE_LANDED = 5,
} fm_state_t;

// Sibling actor IDs
typedef struct {
    hive_actor_id_t sensor;
    hive_actor_id_t estimator;
    hive_actor_id_t waypoint;
    hive_actor_id_t altitude;
    hive_actor_id_t position;
    hive_actor_id_t attitude;
    hive_actor_id_t rate;
    hive_actor_id_t motor;
    hive_actor_id_t logger;
    hive_actor_id_t comms;
    hive_actor_id_t battery;
} sibling_ids_t;

// Request all siblings to reset their internal state (request/reply).
// Waits for each actor to acknowledge before moving to the next.
// Order: waypoint first (stops publishing), altitude second (clears
// crash latch), then the rest. Deterministic ordering eliminates the
// race where waypoint publishes a stale target after altitude has
// already cleared its state.
static void request_reset_all(const sibling_ids_t *ids) {
    // Ordered: waypoint, altitude, then remaining actors
    hive_actor_id_t actors[] = {
        ids->waypoint, ids->altitude, ids->sensor, ids->estimator,
        ids->position, ids->attitude, ids->rate,   ids->motor,
        ids->logger,   ids->battery,  ids->comms,
    };

    for (size_t i = 0; i < sizeof(actors) / sizeof(actors[0]); i++) {
        if (actors[i] != HIVE_ACTOR_ID_INVALID) {
            hive_message_t reply;
            hive_status_t s = hive_ipc_request(actors[i], NOTIFY_RESET, NULL, 0,
                                               &reply, RESET_TIMEOUT_MS);
            if (HIVE_FAILED(s)) {
                HIVE_LOG_ERROR("[FLM] RESET request failed for actor %lu: %s",
                               (unsigned long)actors[i], HIVE_ERR_STR(s));
            }
        }
    }
    HIVE_LOG_INFO("[FLM] All actors RESET (request/reply)");
}

// Actor state
typedef struct {
    tunable_params_t *params;
} fm_state_data_t;

void *flight_manager_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static fm_state_data_t state;
    state.params = buses->params;
    return &state;
}

void flight_manager_actor(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    fm_state_data_t *data = args;
    tunable_params_t *params = data->params;

    // Look up sibling actors
    sibling_ids_t ids = {
        .sensor = hive_find_sibling(siblings, sibling_count, "sensor"),
        .estimator = hive_find_sibling(siblings, sibling_count, "estimator"),
        .waypoint = hive_find_sibling(siblings, sibling_count, "waypoint"),
        .altitude = hive_find_sibling(siblings, sibling_count, "altitude"),
        .position = hive_find_sibling(siblings, sibling_count, "position"),
        .attitude = hive_find_sibling(siblings, sibling_count, "attitude"),
        .rate = hive_find_sibling(siblings, sibling_count, "rate"),
        .motor = hive_find_sibling(siblings, sibling_count, "motor"),
        .logger = hive_find_sibling(siblings, sibling_count, "logger"),
        .comms = hive_find_sibling(siblings, sibling_count, "comms"),
        .battery = hive_find_sibling(siblings, sibling_count, "battery"),
    };

    // Required siblings
    if (ids.waypoint == HIVE_ACTOR_ID_INVALID ||
        ids.altitude == HIVE_ACTOR_ID_INVALID ||
        ids.motor == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR(
            "[FLM] Missing required siblings: waypoint=%s altitude=%s motor=%s",
            ids.waypoint == HIVE_ACTOR_ID_INVALID ? "MISSING" : "ok",
            ids.altitude == HIVE_ACTOR_ID_INVALID ? "MISSING" : "ok",
            ids.motor == HIVE_ACTOR_ID_INVALID ? "MISSING" : "ok");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Optional siblings (comms not present in Webots)
    if (ids.comms == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_INFO("[FLM] comms actor not found (expected in Webots)");
    }
    if (ids.logger == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_INFO("[FLM] logger actor not found");
    }

    // Log flight profile and battery for post-mortem verification
    HIVE_LOG_INFO("[FLM] Flight profile=%d duration=%.0fs battery=%.2fV",
                  FLIGHT_PROFILE, FLIGHT_DURATION_US / 1000000.0f,
                  hal_power_get_battery());

    // State machine
    fm_state_t state = FM_STATE_IDLE;
    uint8_t countdown_s = 0;
    hive_timer_id_t countdown_timer = HIVE_TIMER_ID_INVALID;
    hive_timer_id_t flight_timer = HIVE_TIMER_ID_INVALID;
    hive_timer_id_t liftoff_timer = HIVE_TIMER_ID_INVALID;
    hive_timer_id_t landing_timer = HIVE_TIMER_ID_INVALID;

    // Auto-GO timer for simulation
    hive_timer_id_t auto_go_timer = HIVE_TIMER_ID_INVALID;
    if (params->auto_go_delay_s > 0.0f) {
        uint64_t delay_us = (uint64_t)(params->auto_go_delay_s * 1000000.0f);
        hive_status_t ts = hive_timer_after(delay_us, &auto_go_timer);
        if (HIVE_FAILED(ts)) {
            HIVE_LOG_ERROR("[FLM] Auto-GO timer failed: %s", HIVE_ERR_STR(ts));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }
        HIVE_LOG_INFO("[FLM] Auto-GO timer set for %.1fs",
                      params->auto_go_delay_s);
    }

    HIVE_LOG_INFO("[FLM] State machine started - IDLE");

    // Main state machine loop
    while (true) {
        switch (state) {
        case FM_STATE_IDLE: {
            // Wait for GO command (from comms or auto-GO timer) or STATUS request
            HIVE_LOG_INFO("[FLM] IDLE - waiting for GO");

            enum { SEL_GO, SEL_AUTO_GO, SEL_STATUS };
            hive_select_source_t idle_sources[3];
            size_t num_idle_sources = 0;

            // GO from comms (if available)
            if (ids.comms != HIVE_ACTOR_ID_INVALID) {
                idle_sources[num_idle_sources++] = (hive_select_source_t){
                    HIVE_SEL_IPC, .ipc = {.sender = ids.comms,
                                          .class = HIVE_MSG_NOTIFY,
                                          .id = NOTIFY_GO}};
            }
            // Auto-GO timer (simulation)
            if (auto_go_timer != HIVE_TIMER_ID_INVALID) {
                idle_sources[num_idle_sources++] = (hive_select_source_t){
                    HIVE_SEL_IPC,
                    .ipc = {.class = HIVE_MSG_TIMER, .tag = auto_go_timer}};
            }
            // STATUS request (from comms or test harness)
            idle_sources[num_idle_sources++] = (hive_select_source_t){
                HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_REQUEST}};

            hive_select_result_t result;
            hive_status_t s =
                hive_select(idle_sources, num_idle_sources, &result, -1);
            if (HIVE_FAILED(s)) {
                HIVE_LOG_ERROR("[FLM] select (idle) failed: %s",
                               HIVE_ERR_STR(s));
                hive_exit(HIVE_EXIT_REASON_CRASH);
            }

            // Check if STATUS request
            if (result.ipc.class == HIVE_MSG_REQUEST) {
                // Reply with status
                uint8_t status_reply[2] = {(uint8_t)state, countdown_s};
                hive_status_t rs = hive_ipc_reply(&result.ipc, status_reply,
                                                  sizeof(status_reply));
                if (HIVE_FAILED(rs)) {
                    HIVE_LOG_ERROR("[FLM] STATUS reply failed: %s",
                                   HIVE_ERR_STR(rs));
                }
                continue;
            }

            // GO received (either from comms or auto-GO timer)
            HIVE_LOG_INFO("[FLM] GO received - transitioning to PREFLIGHT");
            auto_go_timer = HIVE_TIMER_ID_INVALID; // Consumed
            state = FM_STATE_PREFLIGHT;
            break;
        }

        case FM_STATE_PREFLIGHT: {
            // Calibrate sensors
            HIVE_LOG_INFO("[FLM] PREFLIGHT - calibrating sensors");
            hal_calibrate();

            // Reset all actors (request/reply - deterministic ordering)
            request_reset_all(&ids);

            // ARM motors
            HIVE_LOG_INFO("[FLM] Arming motors");
            hal_arm();

            // Transition to ARMED
            countdown_s = (uint8_t)params->armed_countdown_s;
            HIVE_LOG_INFO("[FLM] Preflight complete - ARMED (countdown=%us)",
                          countdown_s);
            state = FM_STATE_ARMED;

            // Start 1-second countdown tick timer
            hive_status_t ts = hive_timer_every(1 * 1000000, &countdown_timer);
            if (HIVE_FAILED(ts)) {
                HIVE_LOG_ERROR("[FLM] countdown timer failed: %s",
                               HIVE_ERR_STR(ts));
                hive_exit(HIVE_EXIT_REASON_CRASH);
            }
            HIVE_LOG_INFO("[FLM] countdown timer=%lu",
                          (unsigned long)countdown_timer);
            break;
        }

        case FM_STATE_ARMED: {
            // Countdown with ABORT/LOW_BATTERY/STATUS handling
            hive_select_source_t armed_sources[4];
            size_t num_armed_sources = 0;

            // Countdown timer (always present)
            armed_sources[num_armed_sources++] = (hive_select_source_t){
                .type = HIVE_SEL_IPC,
                .ipc = {.class = HIVE_MSG_TIMER, .tag = countdown_timer}};

            // ABORT from comms (if available)
            if (ids.comms != HIVE_ACTOR_ID_INVALID) {
                armed_sources[num_armed_sources++] =
                    (hive_select_source_t){.type = HIVE_SEL_IPC,
                                           .ipc = {.sender = ids.comms,
                                                   .class = HIVE_MSG_NOTIFY,
                                                   .id = NOTIFY_ABORT}};
            }

            // Low battery from battery actor
            if (ids.battery != HIVE_ACTOR_ID_INVALID) {
                armed_sources[num_armed_sources++] =
                    (hive_select_source_t){.type = HIVE_SEL_IPC,
                                           .ipc = {.sender = ids.battery,
                                                   .class = HIVE_MSG_NOTIFY,
                                                   .id = NOTIFY_LOW_BATTERY}};
            }

            // STATUS request (always last)
            armed_sources[num_armed_sources++] = (hive_select_source_t){
                .type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_REQUEST}};

            hive_select_result_t result;
            hive_status_t s =
                hive_select(armed_sources, num_armed_sources, &result, -1);
            if (HIVE_FAILED(s)) {
                HIVE_LOG_ERROR("[FLM] select (armed) failed: %s",
                               HIVE_ERR_STR(s));
                hive_exit(HIVE_EXIT_REASON_CRASH);
            }

            // Handle STATUS request
            if (result.ipc.class == HIVE_MSG_REQUEST) {
                uint8_t status_reply[2] = {(uint8_t)state, countdown_s};
                hive_status_t rs = hive_ipc_reply(&result.ipc, status_reply,
                                                  sizeof(status_reply));
                if (HIVE_FAILED(rs)) {
                    HIVE_LOG_ERROR("[FLM] STATUS reply failed: %s",
                                   HIVE_ERR_STR(rs));
                }
                continue;
            }

            // Handle ABORT or LOW_BATTERY - same action: cancel and disarm
            if (result.ipc.id == NOTIFY_ABORT ||
                result.ipc.id == NOTIFY_LOW_BATTERY) {
                if (result.ipc.id == NOTIFY_LOW_BATTERY) {
                    HIVE_LOG_WARN("[FLM] LOW BATTERY - cancelling countdown");
                } else {
                    HIVE_LOG_INFO("[FLM] ABORT received - returning to IDLE");
                }
                hive_timer_cancel(countdown_timer);
                hal_disarm();
                HIVE_LOG_INFO("[FLM] Motors disarmed");
                state = FM_STATE_IDLE;
                continue;
            }

            // Countdown tick
            if (countdown_s > 1) {
                countdown_s -= 1;
                HIVE_LOG_INFO("[FLM] Countdown: %us remaining", countdown_s);
            } else {
                // Countdown complete
                hive_timer_cancel(countdown_timer);
                HIVE_LOG_INFO("[FLM] Countdown complete - FLYING");
                state = FM_STATE_FLYING;

                // Reset estimator so altitude KF starts fresh.
                // On the ground the rangefinder can't measure (<40mm),
                // so the KF drifts during the countdown.
                {
                    hive_message_t reply;
                    hive_status_t rs =
                        hive_ipc_request(ids.estimator, NOTIFY_RESET, NULL, 0,
                                         &reply, RESET_TIMEOUT_MS);
                    if (HIVE_FAILED(rs)) {
                        HIVE_LOG_ERROR("[FLM] estimator RESET failed: %s",
                                       HIVE_ERR_STR(rs));
                    }
                }

                // Notify motor and waypoint to start
                hive_status_t ms =
                    hive_ipc_notify(ids.motor, NOTIFY_FLIGHT_START, NULL, 0);
                if (HIVE_FAILED(ms)) {
                    HIVE_LOG_ERROR("[FLM] motor START failed: %s",
                                   HIVE_ERR_STR(ms));
                }
                hive_status_t ws =
                    hive_ipc_notify(ids.waypoint, NOTIFY_FLIGHT_START, NULL, 0);
                if (HIVE_FAILED(ws)) {
                    HIVE_LOG_ERROR("[FLM] waypoint START failed: %s",
                                   HIVE_ERR_STR(ws));
                }

                // Start liftoff timeout (flight timer starts after
                // liftoff confirmed by altitude actor)
                hive_status_t lt =
                    hive_timer_after(LIFTOFF_TIMEOUT_US, &liftoff_timer);
                if (HIVE_FAILED(lt)) {
                    HIVE_LOG_ERROR("[FLM] liftoff timer failed: %s",
                                   HIVE_ERR_STR(lt));
                    hive_exit(HIVE_EXIT_REASON_CRASH);
                }
            }
            break;
        }

        case FM_STATE_FLYING: {
            // Wait for liftoff/flight timer, crash, abort, low battery,
            // or STATUS. Flight timer starts only after liftoff is
            // confirmed by the altitude actor. Before liftoff, we wait
            // for NOTIFY_LIFTOFF with a safety timeout.
            hive_select_source_t flying_sources[6];
            size_t num_flying_sources = 0;

            if (flight_timer != HIVE_TIMER_ID_INVALID) {
                // Liftoff confirmed - wait for flight duration timer
                flying_sources[num_flying_sources++] = (hive_select_source_t){
                    .type = HIVE_SEL_IPC,
                    .ipc = {.class = HIVE_MSG_TIMER, .tag = flight_timer}};
            } else {
                // Waiting for liftoff
                flying_sources[num_flying_sources++] =
                    (hive_select_source_t){.type = HIVE_SEL_IPC,
                                           .ipc = {.sender = ids.altitude,
                                                   .class = HIVE_MSG_NOTIFY,
                                                   .id = NOTIFY_LIFTOFF}};
                flying_sources[num_flying_sources++] = (hive_select_source_t){
                    .type = HIVE_SEL_IPC,
                    .ipc = {.class = HIVE_MSG_TIMER, .tag = liftoff_timer}};
            }

            // Crash notification from altitude actor (immediate landing)
            flying_sources[num_flying_sources++] =
                (hive_select_source_t){.type = HIVE_SEL_IPC,
                                       .ipc = {.sender = ids.altitude,
                                               .class = HIVE_MSG_NOTIFY,
                                               .id = NOTIFY_FLIGHT_LANDED}};

            // ABORT from comms (if available)
            if (ids.comms != HIVE_ACTOR_ID_INVALID) {
                flying_sources[num_flying_sources++] =
                    (hive_select_source_t){.type = HIVE_SEL_IPC,
                                           .ipc = {.sender = ids.comms,
                                                   .class = HIVE_MSG_NOTIFY,
                                                   .id = NOTIFY_ABORT}};
            }

            // Low battery from battery actor
            if (ids.battery != HIVE_ACTOR_ID_INVALID) {
                flying_sources[num_flying_sources++] =
                    (hive_select_source_t){.type = HIVE_SEL_IPC,
                                           .ipc = {.sender = ids.battery,
                                                   .class = HIVE_MSG_NOTIFY,
                                                   .id = NOTIFY_LOW_BATTERY}};
            }

            // STATUS request
            flying_sources[num_flying_sources++] = (hive_select_source_t){
                .type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_REQUEST}};

            hive_select_result_t result;
            hive_status_t s =
                hive_select(flying_sources, num_flying_sources, &result, -1);
            if (HIVE_FAILED(s)) {
                HIVE_LOG_ERROR("[FLM] select (flying) failed: %s",
                               HIVE_ERR_STR(s));
                hive_exit(HIVE_EXIT_REASON_CRASH);
            }

            // Handle STATUS request
            if (result.ipc.class == HIVE_MSG_REQUEST) {
                uint8_t status_reply[2] = {(uint8_t)state, 0};
                hive_status_t rs = hive_ipc_reply(&result.ipc, status_reply,
                                                  sizeof(status_reply));
                if (HIVE_FAILED(rs)) {
                    HIVE_LOG_ERROR("[FLM] STATUS reply failed: %s",
                                   HIVE_ERR_STR(rs));
                }
                continue;
            }

            // Liftoff confirmed - start the real flight timer
            if (result.ipc.id == NOTIFY_LIFTOFF) {
                hive_timer_cancel(liftoff_timer);
                liftoff_timer = HIVE_TIMER_ID_INVALID;
                hive_status_t ft =
                    hive_timer_after(FLIGHT_DURATION_US, &flight_timer);
                if (HIVE_FAILED(ft)) {
                    HIVE_LOG_ERROR("[FLM] flight timer failed: %s",
                                   HIVE_ERR_STR(ft));
                    hive_exit(HIVE_EXIT_REASON_CRASH);
                }
                HIVE_LOG_INFO("[FLM] Liftoff confirmed - flight timer "
                              "started (%ds)",
                              FLIGHT_DURATION_S);
                continue; // Stay in FLYING, now with flight timer
            }

            // Liftoff timeout - failed to lift off
            if (result.ipc.class == HIVE_MSG_TIMER &&
                result.ipc.tag == (uint32_t)liftoff_timer) {
                HIVE_LOG_ERROR("[FLM] Liftoff timeout (%ds) - aborting",
                               LIFTOFF_TIMEOUT_S);
                liftoff_timer = HIVE_TIMER_ID_INVALID;
                state = FM_STATE_LANDED;
                break;
            }

            // Crash - altitude actor detected emergency, drone already
            // down. Skip LANDING and go straight to LANDED.
            if (result.ipc.id == NOTIFY_FLIGHT_LANDED) {
                HIVE_LOG_WARN("[FLM] CRASH - altitude actor reports landed");
                if (flight_timer != HIVE_TIMER_ID_INVALID) {
                    hive_timer_cancel(flight_timer);
                    flight_timer = HIVE_TIMER_ID_INVALID;
                }
                if (liftoff_timer != HIVE_TIMER_ID_INVALID) {
                    hive_timer_cancel(liftoff_timer);
                    liftoff_timer = HIVE_TIMER_ID_INVALID;
                }
                state = FM_STATE_LANDED;
                break;
            }

            // ABORT from ground station - initiate immediate landing
            if (result.ipc.id == NOTIFY_ABORT) {
                HIVE_LOG_INFO("[FLM] ABORT during flight - landing");
            } else if (result.ipc.id == NOTIFY_LOW_BATTERY) {
                HIVE_LOG_WARN("[FLM] LOW BATTERY - emergency landing");
            } else {
                HIVE_LOG_INFO(
                    "[FLM] Flight duration complete - initiating LANDING");
            }

            // Cancel whichever timer is active
            if (flight_timer != HIVE_TIMER_ID_INVALID) {
                hive_timer_cancel(flight_timer);
                flight_timer = HIVE_TIMER_ID_INVALID;
            }
            if (liftoff_timer != HIVE_TIMER_ID_INVALID) {
                hive_timer_cancel(liftoff_timer);
                liftoff_timer = HIVE_TIMER_ID_INVALID;
            }

            hive_status_t ls =
                hive_ipc_notify(ids.altitude, NOTIFY_LANDING, NULL, 0);
            if (HIVE_FAILED(ls)) {
                HIVE_LOG_ERROR("[FLM] LANDING notify failed: %s",
                               HIVE_ERR_STR(ls));
            }
            hive_status_t lts =
                hive_timer_after(LANDING_TIMEOUT_US, &landing_timer);
            if (HIVE_FAILED(lts)) {
                HIVE_LOG_ERROR("[FLM] landing timer failed: %s",
                               HIVE_ERR_STR(lts));
            }
            state = FM_STATE_LANDING;
            break;
        }

        case FM_STATE_LANDING: {
            // Wait for LANDED, landing timeout, or STATUS request
            enum { SEL_LANDED, SEL_TIMEOUT, SEL_STATUS_REQ };
            hive_select_source_t landing_sources[] = {
                [SEL_LANDED] = {.type = HIVE_SEL_IPC,
                                .ipc = {.sender = ids.altitude,
                                        .class = HIVE_MSG_NOTIFY,
                                        .id = NOTIFY_FLIGHT_LANDED}},
                [SEL_TIMEOUT] = {.type = HIVE_SEL_IPC,
                                 .ipc = {.class = HIVE_MSG_TIMER,
                                         .tag = landing_timer}},
                [SEL_STATUS_REQ] = {.type = HIVE_SEL_IPC,
                                    .ipc = {.class = HIVE_MSG_REQUEST}},
            };

            hive_select_result_t result;
            hive_status_t s = hive_select(landing_sources, 3, &result, -1);
            if (HIVE_FAILED(s)) {
                HIVE_LOG_ERROR("[FLM] select (landing) failed: %s",
                               HIVE_ERR_STR(s));
                hive_exit(HIVE_EXIT_REASON_CRASH);
            }

            // Handle STATUS request
            if (result.ipc.class == HIVE_MSG_REQUEST) {
                uint8_t status_reply[2] = {(uint8_t)state, 0};
                hive_status_t rs = hive_ipc_reply(&result.ipc, status_reply,
                                                  sizeof(status_reply));
                if (HIVE_FAILED(rs)) {
                    HIVE_LOG_ERROR("[FLM] STATUS reply failed: %s",
                                   HIVE_ERR_STR(rs));
                }
                continue;
            }

            // Landing timeout
            if (result.ipc.tag == (uint32_t)landing_timer) {
                HIVE_LOG_WARN("[FLM] Landing timeout - forcing LANDED");
            } else {
                HIVE_LOG_INFO("[FLM] LANDED notification received");
            }

            hive_timer_cancel(landing_timer);
            state = FM_STATE_LANDED;
            break;
        }

        case FM_STATE_LANDED: {
            // Stop motors, disarm, return to IDLE
            HIVE_LOG_INFO("[FLM] LANDED - stopping motors");
            hive_status_t ss =
                hive_ipc_notify(ids.motor, NOTIFY_FLIGHT_STOP, NULL, 0);
            if (HIVE_FAILED(ss)) {
                HIVE_LOG_ERROR("[FLM] STOP notify failed: %s",
                               HIVE_ERR_STR(ss));
            }

            // Disarm motors
            hal_disarm();
            HIVE_LOG_INFO("[FLM] Motors disarmed");

            // Capture stack profile
            stack_profile_capture("flight_mgr");
            stack_profile_request();

            // Return to IDLE - reset timers for next flight
            HIVE_LOG_INFO("[FLM] Flight complete - returning to IDLE");
            countdown_s = 0;
            flight_timer = HIVE_TIMER_ID_INVALID;
            liftoff_timer = HIVE_TIMER_ID_INVALID;
            state = FM_STATE_IDLE;
            break;
        }
        }
    }
}

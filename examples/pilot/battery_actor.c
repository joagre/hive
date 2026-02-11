// Battery monitoring actor - In-flight voltage protection
//
// Reads hal_power_get_battery() at 2 Hz (500ms timer). Two-tier thresholds
// matching Bitcraze pm_stm32f4.c:
//   WARNING:  3.2V - log WARN once
//   CRITICAL: 3.0V - 10 consecutive readings (5s debounce), then notify
//             flight_manager for emergency landing
//
// Debouncing prevents false triggers from voltage sag under motor load.
// On simulation, hal_power_get_battery() returns 4.2V so this actor
// never triggers.

#include "battery_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_select.h"
#include "hive_log.h"
#include "stack_profile.h"
#include <stdbool.h>

// Voltage thresholds (Bitcraze values)
#define BATTERY_WARNING_V 3.2f
#define BATTERY_CRITICAL_V 3.0f

// Debounce: 10 consecutive critical readings at 2 Hz = 5 seconds
#define CRITICAL_DEBOUNCE_COUNT 10

// Sampling interval: 500ms (2 Hz)
#define BATTERY_SAMPLE_INTERVAL_US (500 * 1000)

// Actor state
typedef struct {
    tunable_params_t *params;
} battery_state_t;

void *battery_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static battery_state_t state;
    state.params = buses->params;
    return &state;
}

void battery_actor(void *args, const hive_spawn_info_t *siblings,
                   size_t sibling_count) {
    (void)args;

    // Find flight_manager sibling
    hive_actor_id_t flight_mgr =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (flight_mgr == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[BAT] flight_manager sibling not found");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Start periodic sampling timer
    hive_timer_id_t sample_timer;
    hive_timer_every(BATTERY_SAMPLE_INTERVAL_US, &sample_timer);

    // State
    uint32_t critical_count = 0;
    bool warning_logged = false;
    bool critical_notified = false;

    HIVE_LOG_INFO("[BAT] Battery monitor started (warn=%.1fV crit=%.1fV)",
                  BATTERY_WARNING_V, BATTERY_CRITICAL_V);

    while (true) {
        // Wait for timer tick or RESET notification
        enum { SEL_TIMER, SEL_RESET };
        hive_select_source_t sources[] = {
            [SEL_TIMER] = {.type = HIVE_SEL_IPC,
                           .ipc = {.class = HIVE_MSG_TIMER,
                                   .tag = sample_timer}},
            [SEL_RESET] = {.type = HIVE_SEL_IPC,
                           .ipc = {.class = HIVE_MSG_NOTIFY,
                                   .id = NOTIFY_RESET}},
        };

        hive_select_result_t result;
        hive_status_t s = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(s)) {
            HIVE_LOG_ERROR("[BAT] select failed: %s", HIVE_ERR_STR(s));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        // Handle RESET from flight_manager
        if (result.ipc.id == NOTIFY_RESET) {
            critical_count = 0;
            warning_logged = false;
            critical_notified = false;
            stack_profile_capture("battery");
            HIVE_LOG_DEBUG("[BAT] State reset");
            continue;
        }

        // Timer tick - read battery voltage
        float voltage = hal_power_get_battery();

        // Check critical threshold (with debounce)
        if (voltage < BATTERY_CRITICAL_V && voltage > 0.0f) {
            critical_count++;
            if (critical_count >= CRITICAL_DEBOUNCE_COUNT &&
                !critical_notified) {
                HIVE_LOG_WARN("[BAT] CRITICAL: %.2fV (debounced %lu readings)",
                              voltage, (unsigned long)critical_count);
                hive_ipc_notify(flight_mgr, NOTIFY_LOW_BATTERY, NULL, 0);
                critical_notified = true;
            }
        } else {
            critical_count = 0;
        }

        // Check warning threshold (log once)
        if (voltage < BATTERY_WARNING_V && voltage > 0.0f && !warning_logged) {
            HIVE_LOG_WARN("[BAT] WARNING: battery low at %.2fV", voltage);
            warning_logged = true;
        }
    }
}

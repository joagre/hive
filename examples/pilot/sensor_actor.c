// Sensor actor - Timer-driven sensor reading
//
// Periodically reads raw sensors via HAL, publishes to sensor bus.
// Sensor fusion is done by the estimator actor.

#include "sensor_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_log.h"

#define SENSOR_INTERVAL_US (TIME_STEP_MS * 1000)

// Actor state - initialized by sensor_actor_init
typedef struct {
    hive_bus_id_t sensor_bus;
    hive_actor_id_t flight_manager;
} sensor_state_t;

void *sensor_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static sensor_state_t state;
    state.sensor_bus = buses->sensor_bus;
    state.flight_manager = HIVE_ACTOR_ID_INVALID;
    return &state;
}

void sensor_actor(void *args, const hive_spawn_info_t *siblings,
                  size_t sibling_count) {
    sensor_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[SENSOR] Failed to find flight_manager sibling");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    hive_timer_id_t timer;
    hive_status_t status = hive_timer_every(SENSOR_INTERVAL_US, &timer);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[SENSOR] Failed to create periodic timer: %s",
                       HIVE_ERR_STR(status));
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Set up hive_select() sources: timer + RESET notification
    enum { SEL_TIMER, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_TIMER] = {HIVE_SEL_IPC, .ipc = {.sender = HIVE_SENDER_ANY,
                                             .class = HIVE_MSG_TIMER,
                                             .id = HIVE_ID_ANY,
                                             .tag = timer}},
        [SEL_RESET] = {HIVE_SEL_IPC, .ipc = {.sender = state->flight_manager,
                                             .class = HIVE_MSG_NOTIFY,
                                             .id = NOTIFY_RESET,
                                             .tag = HIVE_TAG_ANY}},
    };

    while (1) {
        hive_select_result_t result;
        status = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[SENSOR] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_RESET) {
            // No internal state to reset - just acknowledge
            HIVE_LOG_INFO("[SENSOR] RESET received");
            continue;
        }

        // SEL_TIMER: Read and publish sensor data
        sensor_data_t sensors;
        hal_read_sensors(&sensors);
        status = hive_bus_publish(state->sensor_bus, &sensors, sizeof(sensors));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[SENSOR] bus publish failed: %s",
                          HIVE_ERR_STR(status));
        }
    }
}

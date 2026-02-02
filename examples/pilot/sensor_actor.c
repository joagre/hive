// Sensor actor - Timer-driven sensor reading
//
// Periodically reads raw sensors via HAL, publishes to sensor bus.
// Sensor fusion is done by the estimator actor.

#include "sensor_actor.h"
#include "pilot_buses.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_log.h"

#define SENSOR_INTERVAL_US (TIME_STEP_MS * 1000)

// Actor state - initialized by sensor_actor_init
typedef struct {
    hive_bus_id_t sensor_bus;
} sensor_state_t;

void *sensor_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static sensor_state_t state;
    state.sensor_bus = buses->sensor_bus;
    return &state;
}

void sensor_actor(void *args, const hive_spawn_info_t *siblings,
                  size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;

    sensor_state_t *state = args;

    hive_timer_id_t timer;
    hive_status_t status = hive_timer_every(SENSOR_INTERVAL_US, &timer);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[SENSOR] Failed to create periodic timer: %s",
                       HIVE_ERR_STR(status));
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    while (1) {
        hive_message_t msg;
        status = hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer,
                                     &msg, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[SENSOR] recv_match failed: %s",
                           HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        sensor_data_t sensors;
        hal_read_sensors(&sensors);
        status = hive_bus_publish(state->sensor_bus, &sensors, sizeof(sensors));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[SENSOR] bus publish failed: %s",
                          HIVE_ERR_STR(status));
        }
    }
}

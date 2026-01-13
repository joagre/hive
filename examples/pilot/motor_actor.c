// Motor actor - Output layer
//
// Subscribes to torque bus, writes to hardware via HAL.
// The HAL handles mixing (converting torque to individual motor commands).
// Listens for STOP notification from supervisor for emergency cutoff.

#include "motor_actor.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include <assert.h>

// Timeout for bus read - allows periodic check for STOP notification
#define TORQUE_READ_TIMEOUT_US  (50 * 1000)  // 50ms

static bus_id s_torque_bus;

void motor_actor_init(bus_id torque_bus) {
    s_torque_bus = torque_bus;
}

void motor_actor(void *arg) {
    (void)arg;

    BUS_SUBSCRIBE(s_torque_bus);

    bool stopped = false;

    while (1) {
        // Check for STOP notification from supervisor (non-blocking)
        hive_message msg;
        if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY,
                                               NOTIFY_FLIGHT_STOP, &msg, 0))) {
            stopped = true;
        }

        if (stopped) {
            // Output zero torque and sleep before checking STOP again
            torque_cmd_t zero = TORQUE_CMD_ZERO;
            hal_write_torque(&zero);
            hive_sleep(TORQUE_READ_TIMEOUT_US);
            continue;
        }

        // Wait for torque with timeout (allows periodic STOP check)
        torque_cmd_t torque;
        if (BUS_READ_TIMEOUT(s_torque_bus, &torque, TORQUE_READ_TIMEOUT_US)) {
            hal_write_torque(&torque);
        }
    }
}

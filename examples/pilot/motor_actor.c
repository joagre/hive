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
#include <assert.h>

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

        torque_cmd_t torque;

        // Block until torque command available
        BUS_READ_WAIT(s_torque_bus, &torque);

        // If stopped, zero all output
        if (stopped) {
            torque = (torque_cmd_t)TORQUE_CMD_ZERO;
        }

        // HAL handles mixing and hardware output
        hal_write_torque(&torque);
    }
}

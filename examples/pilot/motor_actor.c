// Motor actor - Output layer
//
// Subscribes to torque bus, writes to hardware via HAL.
// The HAL handles mixing (converting torque to individual motor commands).
// Checks for STOP notification from flight manager (best-effort - only checked
// when torque commands arrive, won't interrupt blocking bus read).
//
// Uses name registry:
// - Registers self as "motor"

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

    // Register self with name registry
    hive_status status = hive_register("motor");
    assert(HIVE_SUCCEEDED(status));

    status = hive_bus_subscribe(s_torque_bus);
    assert(HIVE_SUCCEEDED(status));

    bool stopped = false;

    while (1) {
        hive_message msg;
        torque_cmd_t torque;
        size_t len;

        // Check for STOP notification (non-blocking, best-effort)
        if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY,
                                               NOTIFY_FLIGHT_STOP, &msg, 0))) {
            stopped = true;
        }

        hive_bus_read_wait(s_torque_bus, &torque, sizeof(torque), &len, -1);

        if (stopped) {
            torque = (torque_cmd_t)TORQUE_CMD_ZERO;
        }

        hal_write_torque(&torque);
    }
}

// Motor actor - Output layer
//
// Subscribes to torque bus, writes to hardware via HAL.
// The HAL handles mixing (converting torque to individual motor commands).

#include "motor_actor.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include <assert.h>

static bus_id s_torque_bus;

void motor_actor_init(bus_id torque_bus) {
    s_torque_bus = torque_bus;
}

void motor_actor(void *arg) {
    (void)arg;

    BUS_SUBSCRIBE(s_torque_bus);

    while (1) {
        torque_cmd_t torque;

        // Block until torque command available
        BUS_READ_WAIT(s_torque_bus, &torque);

        // HAL handles mixing and hardware output
        hal_write_torque(&torque);
    }
}

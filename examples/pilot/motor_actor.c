// Motor actor - Mixer and safety layer
//
// Subscribes to torque bus, applies mixer, enforces limits, implements watchdog,
// writes to hardware.

#include "motor_actor.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include <assert.h>
#include <stdbool.h>

static bus_id s_torque_bus;
static write_motors_fn s_write_motors;

// Motor mixer for Crazyflie "X" configuration (matching Bitcraze/Webots).
//
//         Front
//       M2    M3
//         \  /
//          \/
//          /\
//         /  \
//       M1    M4
//         Rear
//
// In X-config, each motor affects BOTH roll AND pitch.
static void mixer_apply(const torque_cmd_t *torque, motor_cmd_t *cmd) {
    cmd->motor[0] = torque->thrust - torque->roll + torque->pitch + torque->yaw;  // M1 (rear-left)
    cmd->motor[1] = torque->thrust - torque->roll - torque->pitch - torque->yaw;  // M2 (front-left)
    cmd->motor[2] = torque->thrust + torque->roll - torque->pitch + torque->yaw;  // M3 (front-right)
    cmd->motor[3] = torque->thrust + torque->roll + torque->pitch - torque->yaw;  // M4 (rear-right)

    for (int i = 0; i < NUM_MOTORS; i++) {
        cmd->motor[i] = CLAMPF(cmd->motor[i], 0.0f, 1.0f);
    }
}

void motor_actor_init(bus_id torque_bus, write_motors_fn write_motors) {
    assert(write_motors != NULL);
    s_torque_bus = torque_bus;
    s_write_motors = write_motors;
}

void motor_actor(void *arg) {
    (void)arg;

    assert(!HIVE_FAILED(hive_bus_subscribe(s_torque_bus)));

    int watchdog = 0;
    motor_cmd_t cmd = MOTOR_CMD_ZERO;
    bool armed = false;

    while (1) {
        torque_cmd_t torque;

        if (BUS_READ(s_torque_bus, &torque)) {
            watchdog = 0;

            mixer_apply(&torque, &cmd);
            armed = true;

            s_write_motors(&cmd);
        } else {
            watchdog++;

            if (watchdog >= MOTOR_WATCHDOG_TIMEOUT && armed) {
                HIVE_LOG_WARN("MOTOR WATCHDOG: No commands for %dms - cutting motors!",
                              MOTOR_WATCHDOG_TIMEOUT * TIME_STEP_MS);
                cmd = (motor_cmd_t)MOTOR_CMD_ZERO;
                s_write_motors(&cmd);
                armed = false;
            }
        }

        hive_yield();
    }
}

// Motor actor - Safety layer
//
// Subscribes to motor bus, enforces limits, implements watchdog, writes to hardware.

#include "motor_actor.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include <stdio.h>
#include <stdbool.h>

static bus_id s_motor_bus;
static motor_write_fn s_write_fn;

// Helper to zero all motors
static void motor_cmd_zero(motor_cmd_t *cmd) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        cmd->motor[i] = 0.0f;
    }
}

void motor_actor_init(bus_id motor_bus, motor_write_fn write_fn) {
    s_motor_bus = motor_bus;
    s_write_fn = write_fn;
}

void motor_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_motor_bus);

    int watchdog = 0;
    motor_cmd_t cmd = MOTOR_CMD_ZERO;
    bool armed = false;

    while (1) {
        motor_cmd_t new_cmd;
        size_t len;

        if (hive_bus_read(s_motor_bus, &new_cmd, sizeof(new_cmd), &len).code == HIVE_OK) {
            watchdog = 0;

            // Enforce limits
            for (int i = 0; i < NUM_MOTORS; i++) {
                new_cmd.motor[i] = CLAMPF(new_cmd.motor[i], 0.0f, 1.0f);
            }

            cmd = new_cmd;
            armed = true;

            if (s_write_fn) {
                s_write_fn(&cmd);
            }
        } else {
            watchdog++;

            if (watchdog >= MOTOR_WATCHDOG_TIMEOUT && armed) {
                printf("MOTOR WATCHDOG: No commands for %dms - cutting motors!\n",
                       MOTOR_WATCHDOG_TIMEOUT * TIME_STEP_MS);
                motor_cmd_zero(&cmd);
                if (s_write_fn) {
                    s_write_fn(&cmd);
                }
                armed = false;
            }
        }

        hive_yield();
    }
}

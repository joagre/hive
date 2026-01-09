// Attitude actor - Rate stabilization
//
// Subscribes to state, thrust, and rate setpoint buses, runs rate PIDs,
// publishes torque commands.

#include "attitude_actor.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include <stdio.h>

static bus_id s_state_bus;
static bus_id s_thrust_bus;
static bus_id s_rate_setpoint_bus;
static bus_id s_torque_bus;

void attitude_actor_init(bus_id state_bus, bus_id thrust_bus,
                         bus_id rate_setpoint_bus, bus_id torque_bus) {
    s_state_bus = state_bus;
    s_thrust_bus = thrust_bus;
    s_rate_setpoint_bus = rate_setpoint_bus;
    s_torque_bus = torque_bus;
}

void attitude_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_state_bus);
    hive_bus_subscribe(s_thrust_bus);
    hive_bus_subscribe(s_rate_setpoint_bus);

    pid_state_t roll_pid, pitch_pid, yaw_pid;

    pid_init_full(&roll_pid,  RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, 0.5f, RATE_ROLL_PID_OMAX);
    pid_init_full(&pitch_pid, RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, 0.5f, RATE_PITCH_PID_OMAX);
    pid_init_full(&yaw_pid,   RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, 0.5f, RATE_YAW_PID_OMAX);

    float thrust = 0.0f;
    rate_setpoint_t rate_sp = RATE_SETPOINT_ZERO;
    int count = 0;

    while (1) {
        state_estimate_t state;
        thrust_cmd_t thrust_cmd;
        rate_setpoint_t new_rate_sp;
        size_t len;

        if (hive_bus_read(s_thrust_bus, &thrust_cmd, sizeof(thrust_cmd), &len).code == HIVE_OK) {
            thrust = thrust_cmd.thrust;
        }

        if (hive_bus_read(s_rate_setpoint_bus, &new_rate_sp, sizeof(new_rate_sp), &len).code == HIVE_OK) {
            rate_sp = new_rate_sp;
        }

        if (hive_bus_read(s_state_bus, &state, sizeof(state), &len).code == HIVE_OK) {
            // Sign convention: Webots gyro positive = body rotating CCW when viewed from axis.
            // Our mixer expects positive torque = body rotating CW when viewed from axis.
            // Negate the measured rate to match sign conventions.
            torque_cmd_t cmd;
            cmd.thrust = thrust;
            cmd.roll   = pid_update(&roll_pid,  rate_sp.roll,  -state.roll_rate,  TIME_STEP_S);
            cmd.pitch  = pid_update(&pitch_pid, rate_sp.pitch, -state.pitch_rate, TIME_STEP_S);
            cmd.yaw    = pid_update(&yaw_pid,   rate_sp.yaw,   -state.yaw_rate,   TIME_STEP_S);

            hive_bus_publish(s_torque_bus, &cmd, sizeof(cmd));

            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                printf("[ATT] roll=%5.1f pitch=%5.1f yaw=%5.1f\n",
                       state.roll * RAD_TO_DEG, state.pitch * RAD_TO_DEG, state.yaw * RAD_TO_DEG);
            }
        }

        hive_yield();
    }
}

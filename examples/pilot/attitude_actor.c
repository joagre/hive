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
#include "hive_log.h"
#include <assert.h>

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

    assert(!HIVE_FAILED(hive_bus_subscribe(s_state_bus)));
    assert(!HIVE_FAILED(hive_bus_subscribe(s_thrust_bus)));
    assert(!HIVE_FAILED(hive_bus_subscribe(s_rate_setpoint_bus)));

    pid_state_t roll_pid, pitch_pid, yaw_pid;
    // Note: Different output limits per axis (yaw needs more authority)
    pid_init_full(&roll_pid,  RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_IMAX, RATE_PID_OMAX_ROLL);
    pid_init_full(&pitch_pid, RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_IMAX, RATE_PID_OMAX_PITCH);
    pid_init_full(&yaw_pid,   RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_IMAX, RATE_PID_OMAX_YAW);

    float thrust = 0.0f;
    rate_setpoint_t rate_sp = RATE_SETPOINT_ZERO;
    int count = 0;

    while (1) {
        state_estimate_t state;
        thrust_cmd_t thrust_cmd;
        rate_setpoint_t new_rate_sp;

        if (BUS_READ(s_thrust_bus, &thrust_cmd)) {
            thrust = thrust_cmd.thrust;
        }

        if (BUS_READ(s_rate_setpoint_bus, &new_rate_sp)) {
            rate_sp = new_rate_sp;
        }

        if (BUS_READ(s_state_bus, &state)) {
            // With correct X-config mixer (matching Bitcraze/Webots):
            // - Roll: positive output = right wing down
            // - Pitch: negated to match Webots coordinate convention
            // - Yaw: positive output = clockwise rotation
            torque_cmd_t cmd;
            cmd.thrust = thrust;
            cmd.roll   = pid_update(&roll_pid,  rate_sp.roll,  state.roll_rate, TIME_STEP_S);
            cmd.pitch  = -pid_update(&pitch_pid, rate_sp.pitch, state.pitch_rate, TIME_STEP_S);
            cmd.yaw    = pid_update(&yaw_pid,   rate_sp.yaw,   state.yaw_rate,   TIME_STEP_S);

            hive_bus_publish(s_torque_bus, &cmd, sizeof(cmd));

            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                HIVE_LOG_DEBUG("[ATT] roll=%.1f pitch=%.1f yaw=%.1f",
                               state.roll * RAD_TO_DEG, state.pitch * RAD_TO_DEG, state.yaw * RAD_TO_DEG);
            }
        }

        hive_yield();
    }
}

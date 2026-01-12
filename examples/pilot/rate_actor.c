// Rate actor - Angular rate stabilization
//
// Subscribes to state, thrust, and rate setpoint buses, runs rate PIDs,
// publishes torque commands.

#include "rate_actor.h"
#include "types.h"
#include "config.h"
#include "hal_config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include <assert.h>

static bus_id s_state_bus;
static bus_id s_thrust_bus;
static bus_id s_rate_setpoint_bus;
static bus_id s_torque_bus;

void rate_actor_init(bus_id state_bus, bus_id thrust_bus,
                     bus_id rate_setpoint_bus, bus_id torque_bus) {
    s_state_bus = state_bus;
    s_thrust_bus = thrust_bus;
    s_rate_setpoint_bus = rate_setpoint_bus;
    s_torque_bus = torque_bus;
}

void rate_actor(void *arg) {
    (void)arg;

    BUS_SUBSCRIBE(s_state_bus);
    BUS_SUBSCRIBE(s_thrust_bus);
    BUS_SUBSCRIBE(s_rate_setpoint_bus);

    pid_state_t roll_pid, pitch_pid, yaw_pid;
    // Note: Different output limits per axis (yaw needs more authority)
    pid_init_full(&roll_pid,  HAL_RATE_PID_KP, HAL_RATE_PID_KI, HAL_RATE_PID_KD, HAL_RATE_PID_IMAX, HAL_RATE_PID_OMAX_ROLL);
    pid_init_full(&pitch_pid, HAL_RATE_PID_KP, HAL_RATE_PID_KI, HAL_RATE_PID_KD, HAL_RATE_PID_IMAX, HAL_RATE_PID_OMAX_PITCH);
    pid_init_full(&yaw_pid,   HAL_RATE_PID_KP, HAL_RATE_PID_KI, HAL_RATE_PID_KD, HAL_RATE_PID_IMAX, HAL_RATE_PID_OMAX_YAW);

    float thrust = 0.0f;
    rate_setpoint_t rate_sp = RATE_SETPOINT_ZERO;
    int count = 0;

    while (1) {
        state_estimate_t state;
        thrust_cmd_t thrust_cmd;
        rate_setpoint_t new_rate_sp;

        // Block until state available
        BUS_READ_WAIT(s_state_bus, &state);

        // Read thrust and rate setpoints (non-blocking, use last known)
        if (BUS_READ(s_thrust_bus, &thrust_cmd)) {
            thrust = thrust_cmd.thrust;
        }

        if (BUS_READ(s_rate_setpoint_bus, &new_rate_sp)) {
            rate_sp = new_rate_sp;
        }

        // Torque command uses standard conventions (HAL handles platform differences)
        torque_cmd_t cmd;
        cmd.thrust = thrust;
        cmd.roll   = pid_update(&roll_pid,  rate_sp.roll,  state.roll_rate,  TIME_STEP_S);
        cmd.pitch  = pid_update(&pitch_pid, rate_sp.pitch, state.pitch_rate, TIME_STEP_S);
        cmd.yaw    = pid_update(&yaw_pid,   rate_sp.yaw,   state.yaw_rate,   TIME_STEP_S);

        hive_bus_publish(s_torque_bus, &cmd, sizeof(cmd));

        if (DEBUG_THROTTLE(count, DEBUG_PRINT_INTERVAL)) {
            HIVE_LOG_DEBUG("[RATE] roll=%.1f pitch=%.1f yaw=%.1f",
                           state.roll * RAD_TO_DEG, state.pitch * RAD_TO_DEG, state.yaw * RAD_TO_DEG);
        }
    }
}

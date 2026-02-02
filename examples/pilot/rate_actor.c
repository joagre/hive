// Rate actor - Angular rate stabilization
//
// Subscribes to state, thrust, and rate setpoint buses, runs rate PIDs,
// publishes torque commands.

#include "rate_actor.h"
#include "pilot_buses.h"
#include "types.h"
#include "config.h"
#include "hal_config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include "hive_timer.h"

// Actor state - initialized by rate_actor_init
typedef struct {
    hive_bus_id_t state_bus;
    hive_bus_id_t thrust_bus;
    hive_bus_id_t rate_setpoint_bus;
    hive_bus_id_t torque_bus;
} rate_state_t;

void *rate_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static rate_state_t state;
    state.state_bus = buses->state_bus;
    state.thrust_bus = buses->thrust_bus;
    state.rate_setpoint_bus = buses->rate_setpoint_bus;
    state.torque_bus = buses->torque_bus;
    return &state;
}

void rate_actor(void *args, const hive_spawn_info_t *siblings,
                size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;

    rate_state_t *state = args;

    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->thrust_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->rate_setpoint_bus))) {
        HIVE_LOG_ERROR("[RATE] Bus subscribe failed");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    pid_state_t roll_pid, pitch_pid, yaw_pid;
    // Note: Different output limits per axis (yaw needs more authority)
    pid_init_full(&roll_pid, HAL_RATE_PID_KP, HAL_RATE_PID_KI, HAL_RATE_PID_KD,
                  HAL_RATE_PID_IMAX, HAL_RATE_PID_OMAX_ROLL);
    pid_init_full(&pitch_pid, HAL_RATE_PID_KP, HAL_RATE_PID_KI, HAL_RATE_PID_KD,
                  HAL_RATE_PID_IMAX, HAL_RATE_PID_OMAX_PITCH);
    pid_init_full(&yaw_pid, HAL_RATE_PID_KP, HAL_RATE_PID_KI, HAL_RATE_PID_KD,
                  HAL_RATE_PID_IMAX, HAL_RATE_PID_OMAX_YAW);

    float thrust = 0.0f;
    rate_setpoint_t rate_sp = RATE_SETPOINT_ZERO;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    while (1) {
        state_estimate_t est;
        thrust_cmd_t thrust_cmd;
        rate_setpoint_t new_rate_sp;
        size_t len;
        hive_status_t status;

        // Block until state available
        status = hive_bus_read(state->state_bus, &est, sizeof(est), &len,
                               HIVE_TIMEOUT_INFINITE);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[RATE] bus read failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        // Measure actual dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Read thrust and rate setpoints (non-blocking, use last known)
        if (hive_bus_read(state->thrust_bus, &thrust_cmd, sizeof(thrust_cmd),
                          &len, HIVE_TIMEOUT_NONBLOCKING)
                .code == HIVE_OK) {
            thrust = thrust_cmd.thrust;
        }

        if (hive_bus_read(state->rate_setpoint_bus, &new_rate_sp,
                          sizeof(new_rate_sp), &len, HIVE_TIMEOUT_NONBLOCKING)
                .code == HIVE_OK) {
            rate_sp = new_rate_sp;
        }

        // Torque command uses standard conventions (HAL handles platform
        // differences)
        torque_cmd_t cmd;
        cmd.thrust = thrust;
        cmd.roll = pid_update(&roll_pid, rate_sp.roll, est.roll_rate, dt);
        cmd.pitch = pid_update(&pitch_pid, rate_sp.pitch, est.pitch_rate, dt);
        cmd.yaw = pid_update(&yaw_pid, rate_sp.yaw, est.yaw_rate, dt);

        hive_bus_publish(state->torque_bus, &cmd, sizeof(cmd));
    }
}

// Attitude actor - Attitude angle control
//
// Subscribes to state and attitude setpoint buses, runs attitude PID
// controllers, publishes rate setpoints.

#include "attitude_actor.h"
#include "pilot_buses.h"
#include "types.h"
#include "config.h"
#include "hal_config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include "hive_timer.h"

// Actor state - initialized by attitude_actor_init
typedef struct {
    bus_id state_bus;
    bus_id attitude_setpoint_bus;
    bus_id rate_setpoint_bus;
} attitude_state;

void *attitude_actor_init(void *init_args) {
    const pilot_buses *buses = init_args;
    static attitude_state state;
    state.state_bus = buses->state_bus;
    state.attitude_setpoint_bus = buses->attitude_setpoint_bus;
    state.rate_setpoint_bus = buses->rate_setpoint_bus;
    return &state;
}

void attitude_actor(void *args, const hive_spawn_info *siblings,
                    size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;

    attitude_state *state = args;

    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->attitude_setpoint_bus))) {
        HIVE_LOG_ERROR("[ATT] Bus subscribe failed");
        return;
    }

    pid_state_t roll_pid, pitch_pid, yaw_pid;
    pid_init_full(&roll_pid, HAL_ATTITUDE_PID_KP, HAL_ATTITUDE_PID_KI,
                  HAL_ATTITUDE_PID_KD, HAL_ATTITUDE_PID_IMAX,
                  HAL_ATTITUDE_PID_OMAX);
    pid_init_full(&pitch_pid, HAL_ATTITUDE_PID_KP, HAL_ATTITUDE_PID_KI,
                  HAL_ATTITUDE_PID_KD, HAL_ATTITUDE_PID_IMAX,
                  HAL_ATTITUDE_PID_OMAX);
    pid_init_full(&yaw_pid, HAL_ATTITUDE_PID_KP, HAL_ATTITUDE_PID_KI,
                  HAL_ATTITUDE_PID_KD, HAL_ATTITUDE_PID_IMAX,
                  HAL_ATTITUDE_PID_OMAX);

    // Target attitudes (updated from attitude_setpoint_bus)
    attitude_setpoint_t attitude_sp = ATTITUDE_SETPOINT_ZERO;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    while (1) {
        state_estimate_t est;
        attitude_setpoint_t new_attitude_sp;
        size_t len;
        hive_status status;

        // Block until state available
        status =
            hive_bus_read_wait(state->state_bus, &est, sizeof(est), &len, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[ATT] bus read_wait failed: %s",
                           HIVE_ERR_STR(status));
            return;
        }

        // Measure actual dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Read attitude setpoints from position controller (non-blocking, use
        // last known)
        if (hive_bus_read(state->attitude_setpoint_bus, &new_attitude_sp,
                          sizeof(new_attitude_sp), &len)
                .code == HIVE_OK) {
            attitude_sp = new_attitude_sp;
        }

        rate_setpoint_t setpoint;
        setpoint.roll = pid_update(&roll_pid, attitude_sp.roll, est.roll, dt);
        setpoint.pitch =
            pid_update(&pitch_pid, attitude_sp.pitch, est.pitch, dt);
        setpoint.yaw = pid_update_angle(&yaw_pid, attitude_sp.yaw, est.yaw, dt);

        if (HIVE_FAILED(hive_bus_publish(state->rate_setpoint_bus, &setpoint,
                                         sizeof(setpoint)))) {
            HIVE_LOG_WARN("[ATT] bus publish failed");
        }
    }
}

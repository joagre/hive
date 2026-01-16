// Attitude actor - Attitude angle control
//
// Subscribes to state and attitude setpoint buses, runs attitude PID controllers,
// publishes rate setpoints.

#include "attitude_actor.h"
#include "types.h"
#include "config.h"
#include "hal_config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include <assert.h>

static bus_id s_state_bus;
static bus_id s_attitude_setpoint_bus;
static bus_id s_rate_setpoint_bus;

void attitude_actor_init(bus_id state_bus, bus_id attitude_setpoint_bus, bus_id rate_setpoint_bus) {
    s_state_bus = state_bus;
    s_attitude_setpoint_bus = attitude_setpoint_bus;
    s_rate_setpoint_bus = rate_setpoint_bus;
}

void attitude_actor(void *arg) {
    (void)arg;

    hive_status status = hive_bus_subscribe(s_state_bus);
    assert(HIVE_SUCCEEDED(status));
    status = hive_bus_subscribe(s_attitude_setpoint_bus);
    assert(HIVE_SUCCEEDED(status));

    pid_state_t roll_pid, pitch_pid, yaw_pid;
    pid_init_full(&roll_pid,  HAL_ATTITUDE_PID_KP, HAL_ATTITUDE_PID_KI, HAL_ATTITUDE_PID_KD,
                  HAL_ATTITUDE_PID_IMAX, HAL_ATTITUDE_PID_OMAX);
    pid_init_full(&pitch_pid, HAL_ATTITUDE_PID_KP, HAL_ATTITUDE_PID_KI, HAL_ATTITUDE_PID_KD,
                  HAL_ATTITUDE_PID_IMAX, HAL_ATTITUDE_PID_OMAX);
    pid_init_full(&yaw_pid,   HAL_ATTITUDE_PID_KP, HAL_ATTITUDE_PID_KI, HAL_ATTITUDE_PID_KD,
                  HAL_ATTITUDE_PID_IMAX, HAL_ATTITUDE_PID_OMAX);

    // Target attitudes (updated from attitude_setpoint_bus)
    attitude_setpoint_t attitude_sp = ATTITUDE_SETPOINT_ZERO;

    // For measuring dt
    uint64_t prev_time = hive_get_time();

    while (1) {
        state_estimate_t state;
        attitude_setpoint_t new_attitude_sp;

        // Block until state available
        size_t len;
        hive_bus_read_wait(s_state_bus, &state, sizeof(state), &len, -1);

        // Measure actual dt
        uint64_t now = hive_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;

        // Read attitude setpoints from position controller (non-blocking, use last known)
        if (hive_bus_read(s_attitude_setpoint_bus, &new_attitude_sp, sizeof(new_attitude_sp), &len).code == HIVE_OK) {
            attitude_sp = new_attitude_sp;
        }

        rate_setpoint_t setpoint;
        setpoint.roll  = pid_update(&roll_pid,  attitude_sp.roll,  state.roll,  dt);
        setpoint.pitch = pid_update(&pitch_pid, attitude_sp.pitch, state.pitch, dt);
        setpoint.yaw   = pid_update_angle(&yaw_pid, attitude_sp.yaw, state.yaw, dt);

        hive_bus_publish(s_rate_setpoint_bus, &setpoint, sizeof(setpoint));
    }
}

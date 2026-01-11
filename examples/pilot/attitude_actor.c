// Attitude actor - Attitude angle control
//
// Subscribes to state and attitude setpoint buses, runs attitude PID controllers,
// publishes rate setpoints.

#include "attitude_actor.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
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

    BUS_SUBSCRIBE(s_state_bus);
    BUS_SUBSCRIBE(s_attitude_setpoint_bus);

    pid_state_t roll_pid, pitch_pid, yaw_pid;
    PID_INIT_RPY(roll_pid, pitch_pid, yaw_pid,
                 ATTITUDE_PID_KP, ATTITUDE_PID_KI, ATTITUDE_PID_KD, ATTITUDE_PID_IMAX, ATTITUDE_PID_OMAX);

    // Target attitudes (updated from attitude_setpoint_bus)
    attitude_setpoint_t attitude_sp = ATTITUDE_SETPOINT_ZERO;
    int count = 0;

    while (1) {
        state_estimate_t state;
        attitude_setpoint_t new_attitude_sp;

        // Block until state available
        BUS_READ_WAIT(s_state_bus, &state);

        // Read attitude setpoints from position controller (non-blocking, use last known)
        if (BUS_READ(s_attitude_setpoint_bus, &new_attitude_sp)) {
            attitude_sp = new_attitude_sp;
        }

        rate_setpoint_t setpoint;
        setpoint.roll  = pid_update(&roll_pid,  attitude_sp.roll,  state.roll,  TIME_STEP_S);
        setpoint.pitch = pid_update(&pitch_pid, attitude_sp.pitch, state.pitch, TIME_STEP_S);
        setpoint.yaw   = pid_update_angle(&yaw_pid, attitude_sp.yaw, state.yaw, TIME_STEP_S);

        hive_bus_publish(s_rate_setpoint_bus, &setpoint, sizeof(setpoint));

        if (DEBUG_THROTTLE(count, DEBUG_PRINT_INTERVAL)) {
            HIVE_LOG_DEBUG("[ATT] sp_r=%.2f st_r=%.2f rate_r=%.2f | sp_p=%.2f st_p=%.2f rate_p=%.2f",
                           attitude_sp.roll * RAD_TO_DEG, state.roll * RAD_TO_DEG, setpoint.roll * RAD_TO_DEG,
                           attitude_sp.pitch * RAD_TO_DEG, state.pitch * RAD_TO_DEG, setpoint.pitch * RAD_TO_DEG);
        }
    }
}

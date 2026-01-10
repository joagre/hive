// Angle actor - Attitude angle control
//
// Subscribes to state and angle setpoint buses, runs angle PID controllers,
// publishes rate setpoints.

#include "angle_actor.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include <assert.h>

static bus_id s_state_bus;
static bus_id s_angle_setpoint_bus;
static bus_id s_rate_setpoint_bus;

void angle_actor_init(bus_id state_bus, bus_id angle_setpoint_bus, bus_id rate_setpoint_bus) {
    s_state_bus = state_bus;
    s_angle_setpoint_bus = angle_setpoint_bus;
    s_rate_setpoint_bus = rate_setpoint_bus;
}

void angle_actor(void *arg) {
    (void)arg;

    assert(!HIVE_FAILED(hive_bus_subscribe(s_state_bus)));
    assert(!HIVE_FAILED(hive_bus_subscribe(s_angle_setpoint_bus)));

    pid_state_t roll_pid, pitch_pid, yaw_pid;
    PID_INIT_RPY(roll_pid, pitch_pid, yaw_pid,
                 ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, ANGLE_PID_IMAX, ANGLE_PID_OMAX);

    // Target angles (updated from angle_setpoint_bus)
    angle_setpoint_t angle_sp = ANGLE_SETPOINT_ZERO;
    int count = 0;

    while (1) {
        state_estimate_t state;
        angle_setpoint_t new_angle_sp;

        // Read angle setpoints from position controller
        if (BUS_READ(s_angle_setpoint_bus, &new_angle_sp)) {
            angle_sp = new_angle_sp;
        }

        if (BUS_READ(s_state_bus, &state)) {
            rate_setpoint_t setpoint;
            setpoint.roll  = pid_update(&roll_pid,  angle_sp.roll,  state.roll,  TIME_STEP_S);
            setpoint.pitch = pid_update(&pitch_pid, angle_sp.pitch, state.pitch, TIME_STEP_S);
            setpoint.yaw   = pid_update_angle(&yaw_pid, angle_sp.yaw, state.yaw, TIME_STEP_S);

            hive_bus_publish(s_rate_setpoint_bus, &setpoint, sizeof(setpoint));

            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                HIVE_LOG_DEBUG("[ANG] sp_r=%.2f st_r=%.2f rate_r=%.2f | sp_p=%.2f st_p=%.2f rate_p=%.2f",
                               angle_sp.roll * RAD_TO_DEG, state.roll * RAD_TO_DEG, setpoint.roll * RAD_TO_DEG,
                               angle_sp.pitch * RAD_TO_DEG, state.pitch * RAD_TO_DEG, setpoint.pitch * RAD_TO_DEG);
            }
        }

        hive_yield();
    }
}

// Angle actor - Attitude angle control
//
// Subscribes to state bus, runs angle PID controllers, publishes rate setpoints.

#include "angle_actor.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"

static bus_id s_state_bus;
static bus_id s_rate_setpoint_bus;

void angle_actor_init(bus_id state_bus, bus_id rate_setpoint_bus) {
    s_state_bus = state_bus;
    s_rate_setpoint_bus = rate_setpoint_bus;
}

void angle_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_state_bus);

    pid_state_t roll_pid, pitch_pid, yaw_pid;

    pid_init_full(&roll_pid,  ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, 0.5f, ANGLE_PID_OMAX);
    pid_init_full(&pitch_pid, ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, 0.5f, ANGLE_PID_OMAX);
    pid_init_full(&yaw_pid,   ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, 0.5f, ANGLE_PID_OMAX);

    // Target angles for hover (level flight)
    const float target_roll  = 0.0f;
    const float target_pitch = 0.0f;
    const float target_yaw   = 0.0f;

    while (1) {
        state_estimate_t state;
        size_t len;

        if (hive_bus_read(s_state_bus, &state, sizeof(state), &len).code == HIVE_OK) {
            rate_setpoint_t setpoint;
            setpoint.roll  = pid_update(&roll_pid,  target_roll,  state.roll,  TIME_STEP_S);
            setpoint.pitch = pid_update(&pitch_pid, target_pitch, state.pitch, TIME_STEP_S);
            setpoint.yaw   = pid_update(&yaw_pid,   target_yaw,   state.yaw,   TIME_STEP_S);

            hive_bus_publish(s_rate_setpoint_bus, &setpoint, sizeof(setpoint));
        }

        hive_yield();
    }
}

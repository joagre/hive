// Angle actor - Attitude angle control
//
// Subscribes to IMU bus, runs angle PID controllers, publishes rate setpoints.

#include "angle_actor.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"

static bus_id s_imu_bus;
static bus_id s_rate_setpoint_bus;

void angle_actor_init(bus_id imu_bus, bus_id rate_setpoint_bus) {
    s_imu_bus = imu_bus;
    s_rate_setpoint_bus = rate_setpoint_bus;
}

void angle_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_imu_bus);

    pid_state_t roll_pid, pitch_pid, yaw_pid;

    pid_init(&roll_pid,  ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD);
    pid_init(&pitch_pid, ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD);
    pid_init(&yaw_pid,   ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD);

    roll_pid.output_max  = ANGLE_PID_OMAX;
    pitch_pid.output_max = ANGLE_PID_OMAX;
    yaw_pid.output_max   = ANGLE_PID_OMAX;

    // Target angles for hover (level flight)
    const float target_roll  = 0.0f;
    const float target_pitch = 0.0f;
    const float target_yaw   = 0.0f;

    while (1) {
        imu_data_t imu;
        size_t len;

        if (hive_bus_read(s_imu_bus, &imu, sizeof(imu), &len).code == HIVE_OK) {
            rate_setpoint_t setpoint;
            setpoint.roll  = pid_update(&roll_pid,  target_roll,  imu.roll,  TIME_STEP_S);
            setpoint.pitch = pid_update(&pitch_pid, target_pitch, imu.pitch, TIME_STEP_S);
            setpoint.yaw   = pid_update(&yaw_pid,   target_yaw,   imu.yaw,   TIME_STEP_S);

            hive_bus_publish(s_rate_setpoint_bus, &setpoint, sizeof(setpoint));
        }

        hive_yield();
    }
}

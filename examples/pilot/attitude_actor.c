// Attitude actor - Inner loop rate control
//
// Subscribes to IMU and thrust buses, runs rate PIDs, publishes motor commands.

#include "attitude_actor.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include <stdio.h>

static bus_id s_imu_bus;
static bus_id s_thrust_bus;
static bus_id s_motor_bus;

// Motor mixer for Crazyflie "+" configuration.
//
//         Front
//           M1
//            |
//     M4 ----+---- M2
//            |
//           M3
//          Rear
static void mixer_update(float thrust, float roll, float pitch, float yaw, motor_cmd_t *cmd) {
    cmd->motor[0] = thrust - pitch - yaw;  // M1 (front)
    cmd->motor[1] = thrust - roll  + yaw;  // M2 (right)
    cmd->motor[2] = thrust + pitch - yaw;  // M3 (rear)
    cmd->motor[3] = thrust + roll  + yaw;  // M4 (left)

    for (int i = 0; i < NUM_MOTORS; i++) {
        cmd->motor[i] = CLAMPF(cmd->motor[i], 0.0f, 1.0f);
    }
}

void attitude_actor_init(bus_id imu_bus, bus_id thrust_bus, bus_id motor_bus) {
    s_imu_bus = imu_bus;
    s_thrust_bus = thrust_bus;
    s_motor_bus = motor_bus;
}

void attitude_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_imu_bus);
    hive_bus_subscribe(s_thrust_bus);

    pid_state_t roll_pid, pitch_pid, yaw_pid;

    pid_init(&roll_pid,  RATE_PID_KP, RATE_PID_KI, RATE_PID_KD);
    pid_init(&pitch_pid, RATE_PID_KP, RATE_PID_KI, RATE_PID_KD);
    pid_init(&yaw_pid,   RATE_PID_KP, RATE_PID_KI, RATE_PID_KD);

    roll_pid.output_max  = ROLL_PID_OMAX;
    pitch_pid.output_max = PITCH_PID_OMAX;
    yaw_pid.output_max   = YAW_PID_OMAX;

    float thrust = 0.0f;
    int count = 0;

    while (1) {
        imu_data_t imu;
        thrust_cmd_t thrust_cmd;
        size_t len;

        if (hive_bus_read(s_thrust_bus, &thrust_cmd, sizeof(thrust_cmd), &len).code == HIVE_OK) {
            thrust = thrust_cmd.thrust;
        }

        if (hive_bus_read(s_imu_bus, &imu, sizeof(imu), &len).code == HIVE_OK) {
            float roll_torque  = pid_update(&roll_pid,  0.0f, -imu.gyro_x, TIME_STEP_S);
            float pitch_torque = pid_update(&pitch_pid, 0.0f, -imu.gyro_y, TIME_STEP_S);
            float yaw_torque   = pid_update(&yaw_pid,   0.0f, -imu.gyro_z, TIME_STEP_S);

            motor_cmd_t cmd;
            mixer_update(thrust, roll_torque, pitch_torque, yaw_torque, &cmd);

            hive_bus_publish(s_motor_bus, &cmd, sizeof(cmd));

            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                printf("[ATT] roll=%5.1f pitch=%5.1f yaw=%5.1f\n",
                       imu.roll * RAD_TO_DEG, imu.pitch * RAD_TO_DEG, imu.yaw * RAD_TO_DEG);
            }
        }

        hive_yield();
    }
}

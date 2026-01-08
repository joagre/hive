// Altitude actor - Outer loop altitude control
//
// Subscribes to IMU bus, runs altitude PID, publishes thrust commands.

#include "altitude_actor.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include <stdio.h>

static bus_id s_imu_bus;
static bus_id s_thrust_bus;

void altitude_actor_init(bus_id imu_bus, bus_id thrust_bus) {
    s_imu_bus = imu_bus;
    s_thrust_bus = thrust_bus;
}

void altitude_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_imu_bus);

    pid_state_t alt_pid;
    pid_init(&alt_pid, ALT_PID_KP, ALT_PID_KI, ALT_PID_KD);
    alt_pid.integral_max = ALT_PID_IMAX;
    alt_pid.output_max = ALT_PID_OMAX;

    int count = 0;

    while (1) {
        imu_data_t imu;
        size_t len;

        if (hive_bus_read(s_imu_bus, &imu, sizeof(imu), &len).code == HIVE_OK) {
            float correction = pid_update(&alt_pid, TARGET_ALTITUDE, imu.altitude, TIME_STEP_S);
            float thrust = CLAMPF(BASE_THRUST + correction, 0.0f, 1.0f);

            thrust_cmd_t cmd = {.thrust = thrust};
            hive_bus_publish(s_thrust_bus, &cmd, sizeof(cmd));

            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                printf("[ALT] alt=%.2f target=%.1f thrust=%.3f\n",
                       imu.altitude, TARGET_ALTITUDE, thrust);
            }
        }

        hive_yield();
    }
}

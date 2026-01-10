// Sensor actor - Hardware sensor reading
//
// Reads sensors from platform layer and publishes to IMU bus.
//
// STM32: Timer-driven at 400Hz for power-efficient operation with WFI.
// Webots: Yields each step, driven by external wb_robot_step() loop.

#include "sensor_actor.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"

// 400Hz = 2500us period
#define SENSOR_INTERVAL_US  2500

static bus_id s_imu_bus;
static imu_read_fn s_read_fn;

void sensor_actor_init(bus_id imu_bus, imu_read_fn read_fn) {
    s_imu_bus = imu_bus;
    s_read_fn = read_fn;
}

void sensor_actor(void *arg) {
    (void)arg;

#ifdef PLATFORM_STEVAL_DRONE01
    // STM32: Use periodic timer for 400Hz control loop
    timer_id timer;
    hive_timer_every(SENSOR_INTERVAL_US, &timer);

    while (1) {
        hive_message msg;
        hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

        if (s_read_fn) {
            imu_data_t imu;
            s_read_fn(&imu);
            hive_bus_publish(s_imu_bus, &imu, sizeof(imu));
        }
    }
#else
    // Webots: Yield each step, external loop calls hive_step()
    while (1) {
        if (s_read_fn) {
            imu_data_t imu;
            s_read_fn(&imu);
            hive_bus_publish(s_imu_bus, &imu, sizeof(imu));
        }

        hive_yield();
    }
#endif
}

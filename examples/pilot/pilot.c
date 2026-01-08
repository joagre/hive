// Minimal pilot example - Read sensors and print values using actors
//
// Demonstrates:
// - Webots integration with hive_step()
// - Sensor data flow through bus
// - Single actor reading and printing IMU data

#include <webots/robot.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>

#include "hive_runtime.h"
#include "hive_bus.h"

#include <stdio.h>

#define TIME_STEP 4

// IMU data structure
typedef struct {
    float roll, pitch, yaw;       // radians
    float gyro_x, gyro_y, gyro_z; // rad/s
} imu_data_t;

// Globals
static WbDeviceTag gyro;
static WbDeviceTag imu;
static bus_id imu_bus;

// Sensor actor - reads IMU bus and prints values
void sensor_actor(void *arg) {
    bus_id bus = *(bus_id *)arg;
    hive_bus_subscribe(bus);

    int count = 0;
    while (1) {
        imu_data_t data;
        size_t len;

        if (hive_bus_read(bus, &data, sizeof(data), &len).code == HIVE_OK) {
            if (++count % 250 == 0) {  // Print every ~1 second
                printf("roll=%6.1f  pitch=%6.1f  yaw=%6.1f\n",
                       data.roll * 57.3f, data.pitch * 57.3f, data.yaw * 57.3f);
            }
        }
        hive_yield();
    }
}

int main(void) {
    wb_robot_init();

    // Enable sensors
    gyro = wb_robot_get_device("gyro");
    wb_gyro_enable(gyro, TIME_STEP);
    imu = wb_robot_get_device("inertial unit");
    wb_inertial_unit_enable(imu, TIME_STEP);

    // Initialize runtime and bus
    hive_init();
    hive_bus_config cfg = HIVE_BUS_CONFIG_DEFAULT;
    cfg.max_entries = 1;
    hive_bus_create(&cfg, &imu_bus);

    // Spawn actor
    actor_id sensor;
    hive_spawn(sensor_actor, &imu_bus, &sensor);

    // Main loop
    while (wb_robot_step(TIME_STEP) != -1) {
        // Read sensors
        const double *g = wb_gyro_get_values(gyro);
        const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);

        imu_data_t data = {
            .roll = rpy[0], .pitch = rpy[1], .yaw = rpy[2],
            .gyro_x = g[0], .gyro_y = g[1], .gyro_z = g[2]
        };

        // Publish and run actors
        hive_bus_publish(imu_bus, &data, sizeof(data));
        hive_step();
    }

    hive_cleanup();
    wb_robot_cleanup();
    return 0;
}

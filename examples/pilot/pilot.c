// Pilot example - Quadcopter hover using actor runtime with Webots
//
// Demonstrates altitude-hold hover control for a Crazyflie quadcopter
// using the hive actor runtime. Three actors work together:
//
//   altitude_actor - Outer loop: altitude PID → thrust command
//   attitude_actor - Inner loop: rate PIDs → motor commands
//   motor_actor    - Safety layer: watchdog, limits → hardware
//
// Data flows through buses:
//
//   IMU Bus ──► Altitude Actor ──► Thrust Bus ──► Attitude Actor ──► Motor Bus
//       │                                              │                  │
//       └──────────────────────────────────────────────┘                  │
//                                                                         ▼
//                                                                   Motor Actor
//                                                                         │
//                                                                         ▼
//                                                                    Hardware
//
// To port to real hardware, replace the platform layer functions.

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>

#include "hive_runtime.h"
#include "hive_bus.h"

#include "types.h"
#include "config.h"
#include "altitude_actor.h"
#include "attitude_actor.h"
#include "motor_actor.h"

#include <stdio.h>

// ============================================================================
// BUSES
// ============================================================================

static bus_id g_imu_bus;
static bus_id g_thrust_bus;
static bus_id g_motor_bus;

// ============================================================================
// WEBOTS PLATFORM LAYER
// ============================================================================

static WbDeviceTag motors[NUM_MOTORS];
static WbDeviceTag gyro_dev;
static WbDeviceTag imu_dev;
static WbDeviceTag gps_dev;

static int platform_init(void) {
    wb_robot_init();

    const char *motor_names[NUM_MOTORS] = {"m1_motor", "m2_motor", "m3_motor", "m4_motor"};
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i] = wb_robot_get_device(motor_names[i]);
        if (motors[i] == 0) {
            printf("Error: motor %s not found\n", motor_names[i]);
            return -1;
        }
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 0.0);
    }

    gyro_dev = wb_robot_get_device("gyro");
    imu_dev = wb_robot_get_device("inertial_unit");
    gps_dev = wb_robot_get_device("gps");

    if (gyro_dev == 0 || imu_dev == 0 || gps_dev == 0) {
        printf("Error: sensors not found\n");
        return -1;
    }

    wb_gyro_enable(gyro_dev, TIME_STEP_MS);
    wb_inertial_unit_enable(imu_dev, TIME_STEP_MS);
    wb_gps_enable(gps_dev, TIME_STEP_MS);

    return 0;
}

static void platform_read_imu(imu_data_t *imu) {
    const double *g = wb_gyro_get_values(gyro_dev);
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_dev);
    const double *pos = wb_gps_get_values(gps_dev);

    imu->roll = rpy[0];
    imu->pitch = rpy[1];
    imu->yaw = rpy[2];
    imu->gyro_x = g[0];
    imu->gyro_y = g[1];
    imu->gyro_z = g[2];
    imu->altitude = pos[2];
}

static void platform_write_motors(const motor_cmd_t *cmd) {
    static const float signs[NUM_MOTORS] = {-1.0f, 1.0f, -1.0f, 1.0f};

    for (int i = 0; i < NUM_MOTORS; i++) {
        wb_motor_set_velocity(motors[i],
            signs[i] * cmd->motor[i] * MOTOR_MAX_VELOCITY);
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    if (platform_init() < 0) return 1;

    hive_init();

    // Create buses (single entry = latest value only)
    hive_bus_config cfg = HIVE_BUS_CONFIG_DEFAULT;
    cfg.max_entries = 1;
    hive_bus_create(&cfg, &g_imu_bus);
    hive_bus_create(&cfg, &g_thrust_bus);
    hive_bus_create(&cfg, &g_motor_bus);

    // Initialize and spawn actors
    altitude_actor_init(g_imu_bus, g_thrust_bus);
    attitude_actor_init(g_imu_bus, g_thrust_bus, g_motor_bus);
    motor_actor_init(g_motor_bus, platform_write_motors);

    actor_id altitude, attitude, motor;
    hive_spawn(motor_actor, NULL, &motor);
    hive_spawn(attitude_actor, NULL, &attitude);
    hive_spawn(altitude_actor, NULL, &altitude);

    printf("Pilot: 3 actors (altitude, attitude, motor)\n");

    // Main loop: read sensors, run actors
    while (wb_robot_step(TIME_STEP_MS) != -1) {
        imu_data_t imu;
        platform_read_imu(&imu);
        hive_bus_publish(g_imu_bus, &imu, sizeof(imu));
        hive_step();
    }

    hive_cleanup();
    wb_robot_cleanup();
    return 0;
}

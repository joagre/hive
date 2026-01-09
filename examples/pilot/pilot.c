// Pilot example - Quadcopter waypoint navigation using actor runtime with Webots
//
// Demonstrates waypoint navigation for a Crazyflie quadcopter
// using the hive actor runtime. Eight actors work together:
//
//   sensor_actor    - Reads hardware sensors → IMU bus
//   estimator_actor - Sensor fusion → state bus
//   altitude_actor  - Altitude PID → thrust command
//   waypoint_actor  - Waypoint manager → target bus
//   position_actor  - Position PID → angle setpoints
//   angle_actor     - Angle PIDs → rate setpoints
//   attitude_actor  - Rate PIDs → torque commands
//   motor_actor     - Mixer + safety: torque → motors → hardware
//
// Data flows through buses:
//
//   Sensor → IMU Bus → Estimator → State Bus ─┬→ Altitude → Thrust Bus ──────────┐
//                                             ├→ Position → Angle SP Bus → Angle │
//                                             │                             ↓    │
//                                             └→ Attitude ← Rate SP Bus ←───┘    │
//                                                   ↓                             │
//                                             Torque Bus → Motor ← Thrust Bus ←───┘
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
#include "sensor_actor.h"
#include "estimator_actor.h"
#include "altitude_actor.h"
#include "waypoint_actor.h"
#include "position_actor.h"
#include "angle_actor.h"
#include "attitude_actor.h"
#include "motor_actor.h"

#include <stdio.h>

// ============================================================================
// HELPER MACROS
// ============================================================================

// Spawn an actor at CRITICAL priority. Assigns result to 'id'.
#define SPAWN_CRITICAL_ACTOR(func, name_str, id) do { \
    actor_config _cfg = HIVE_ACTOR_CONFIG_DEFAULT; \
    _cfg.priority = HIVE_PRIORITY_CRITICAL; \
    _cfg.name = name_str; \
    hive_spawn_ex(func, NULL, &_cfg, &id); \
} while (0)

// ============================================================================
// BUSES
// ============================================================================

static bus_id s_imu_bus;
static bus_id s_state_bus;
static bus_id s_thrust_bus;
static bus_id s_target_bus;
static bus_id s_angle_setpoint_bus;
static bus_id s_rate_setpoint_bus;
static bus_id s_torque_bus;

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
    imu->x = pos[0];
    imu->y = pos[1];
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
    if (HIVE_FAILED(hive_bus_create(&cfg, &s_imu_bus)) ||
        HIVE_FAILED(hive_bus_create(&cfg, &s_state_bus)) ||
        HIVE_FAILED(hive_bus_create(&cfg, &s_thrust_bus)) ||
        HIVE_FAILED(hive_bus_create(&cfg, &s_target_bus)) ||
        HIVE_FAILED(hive_bus_create(&cfg, &s_angle_setpoint_bus)) ||
        HIVE_FAILED(hive_bus_create(&cfg, &s_rate_setpoint_bus)) ||
        HIVE_FAILED(hive_bus_create(&cfg, &s_torque_bus))) {
        printf("Error: failed to create buses\n");
        return 1;
    }

    // Initialize actors
    sensor_actor_init(s_imu_bus, platform_read_imu);
    estimator_actor_init(s_imu_bus, s_state_bus);
    altitude_actor_init(s_state_bus, s_thrust_bus, s_target_bus);
    waypoint_actor_init(s_state_bus, s_target_bus);
    position_actor_init(s_state_bus, s_angle_setpoint_bus, s_target_bus);
    angle_actor_init(s_state_bus, s_angle_setpoint_bus, s_rate_setpoint_bus);
    attitude_actor_init(s_state_bus, s_thrust_bus, s_rate_setpoint_bus, s_torque_bus);
    motor_actor_init(s_torque_bus, platform_write_motors);

    // Spawn actors in data-flow order to minimize latency.
    // All CRITICAL priority, round-robin within priority follows spawn order.
    // Order: sensor → estimator → altitude → waypoint → position → angle → attitude → motor
    // This ensures each actor sees fresh data from upstream actors in same step.
    actor_id sensor, estimator, altitude, waypoint, position, angle, attitude, motor;

    SPAWN_CRITICAL_ACTOR(sensor_actor,    "sensor",    sensor);
    SPAWN_CRITICAL_ACTOR(estimator_actor, "estimator", estimator);
    SPAWN_CRITICAL_ACTOR(altitude_actor,  "altitude",  altitude);
    SPAWN_CRITICAL_ACTOR(waypoint_actor,  "waypoint",  waypoint);
    SPAWN_CRITICAL_ACTOR(position_actor,  "position",  position);
    SPAWN_CRITICAL_ACTOR(angle_actor,     "angle",     angle);
    SPAWN_CRITICAL_ACTOR(attitude_actor,  "attitude",  attitude);
    SPAWN_CRITICAL_ACTOR(motor_actor,     "motor",     motor);

    printf("Pilot: 8 actors spawned, waypoint navigation active\n");

    // Main loop: just run actors, sensor actor handles IMU reading
    while (wb_robot_step(TIME_STEP_MS) != -1) {
        hive_step();
    }

    hive_cleanup();
    wb_robot_cleanup();
    return 0;
}

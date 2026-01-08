// Pilot example - Quadcopter hover using actor runtime with Webots
//
// Architecture designed for portability to real hardware:
// - Platform layer: Webots I/O (replaceable with real hardware drivers)
// - Actor layer: Portable control logic (PID, mixer)

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>

#include "hive_runtime.h"
#include "hive_bus.h"

#include <stdio.h>

#define TIME_STEP 4
#define MOTOR_MAX_VELOCITY 100.0f  // Much lower base velocity

// ===========================================================================
// Portable types (same on Webots and real hardware)
// ===========================================================================

typedef struct {
    float roll, pitch, yaw;
    float gyro_x, gyro_y, gyro_z;
    float altitude;  // meters above ground
} imu_data_t;

typedef struct {
    float motor[4];  // 0.0-1.0 normalized
} motor_cmd_t;

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float integral_max;
    float output_max;
} pid_state_t;

// ===========================================================================
// Portable control code (same on Webots and real hardware)
// ===========================================================================

static void pid_init(pid_state_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = 0.5f;
    pid->output_max = 1.0f;
}

static float pid_update(pid_state_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    float p = pid->kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i = pid->ki * pid->integral;

    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = p + i + d;
    if (output > pid->output_max) output = pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;

    return output;
}

// Mixer for Crazyflie + configuration
static void mixer_update(float thrust, float roll, float pitch, float yaw, motor_cmd_t *cmd) {
    cmd->motor[0] = thrust - pitch - yaw;  // M1 front
    cmd->motor[1] = thrust - roll  + yaw;  // M2 right
    cmd->motor[2] = thrust + pitch - yaw;  // M3 rear
    cmd->motor[3] = thrust + roll  + yaw;  // M4 left

    for (int i = 0; i < 4; i++) {
        if (cmd->motor[i] < 0.0f) cmd->motor[i] = 0.0f;
        if (cmd->motor[i] > 1.0f) cmd->motor[i] = 1.0f;
    }
}

// ===========================================================================
// Platform abstraction (implement differently for real hardware)
// ===========================================================================

// Motor output - called by actor, implemented by platform
static motor_cmd_t g_motor_cmd;
static volatile int g_motor_cmd_ready = 0;

static void motor_output_set(const motor_cmd_t *cmd) {
    g_motor_cmd = *cmd;
    g_motor_cmd_ready = 1;
}

// ===========================================================================
// Portable attitude actor
// ===========================================================================

static bus_id g_imu_bus;

void attitude_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(g_imu_bus);

    pid_state_t roll_pid, pitch_pid, yaw_pid, alt_pid;
    // Conservative gains for attitude
    pid_init(&roll_pid,  0.02f, 0.0f, 0.001f);
    pid_init(&pitch_pid, 0.02f, 0.0f, 0.001f);
    pid_init(&yaw_pid,   0.02f, 0.0f, 0.001f);
    roll_pid.output_max = 0.1f;
    pitch_pid.output_max = 0.1f;
    yaw_pid.output_max = 0.15f;

    // Altitude controller - adjusts thrust to hold target altitude
    pid_init(&alt_pid, 0.3f, 0.05f, 0.15f);  // Reduced for smoother response
    alt_pid.output_max = 0.15f;  // Max thrust adjustment ±15%
    alt_pid.integral_max = 0.2f;

    const float dt = 0.004f;  // 4ms
    const float base_thrust = 0.553f;  // Approximate hover thrust
    const float target_altitude = 1.0f;  // Target hover height in meters

    int count = 0;

    while (1) {
        imu_data_t imu;
        size_t len;

        if (hive_bus_read(g_imu_bus, &imu, sizeof(imu), &len).code == HIVE_OK) {
            // Attitude control - negate gyro to match sign convention
            float roll_torque  = pid_update(&roll_pid,  0.0f, -imu.gyro_x, dt);
            float pitch_torque = pid_update(&pitch_pid, 0.0f, -imu.gyro_y, dt);
            float yaw_torque   = pid_update(&yaw_pid,   0.0f, -imu.gyro_z, dt);

            // Altitude control - adjust thrust to reach target altitude
            float alt_correction = pid_update(&alt_pid, target_altitude, imu.altitude, dt);
            float thrust = base_thrust + alt_correction;
            if (thrust < 0.0f) thrust = 0.0f;
            if (thrust > 1.0f) thrust = 1.0f;

            motor_cmd_t cmd;
            mixer_update(thrust, roll_torque, pitch_torque, yaw_torque, &cmd);
            motor_output_set(&cmd);

            if (++count % 250 == 0) {
                printf("alt=%.2f thrust=%.2f | roll=%5.1f pitch=%5.1f\n",
                       imu.altitude, thrust, imu.roll * 57.3f, imu.pitch * 57.3f);
            }
        }

        hive_yield();
    }
}

// ===========================================================================
// Webots platform layer (replace this for real hardware)
// ===========================================================================

static WbDeviceTag motors[4];
static WbDeviceTag gyro_dev;
static WbDeviceTag imu_dev;
static WbDeviceTag gps_dev;

static int platform_init(void) {
    wb_robot_init();

    const char *motor_names[] = {"m1_motor", "m2_motor", "m3_motor", "m4_motor"};
    for (int i = 0; i < 4; i++) {
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

    wb_gyro_enable(gyro_dev, TIME_STEP);
    wb_inertial_unit_enable(imu_dev, TIME_STEP);
    wb_gps_enable(gps_dev, TIME_STEP);

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
    imu->altitude = pos[2];  // Z is altitude in Webots
}

static void platform_write_motors(void) {
    if (g_motor_cmd_ready) {
        // Try opposite sign pattern: M1,M3 negative, M2,M4 positive
        float signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
        for (int i = 0; i < 4; i++) {
            wb_motor_set_velocity(motors[i], signs[i] * g_motor_cmd.motor[i] * MOTOR_MAX_VELOCITY);
        }
    }
}

// ===========================================================================
// Main (Webots-specific integration loop)
// ===========================================================================

int main(void) {
    if (platform_init() < 0) return 1;

    hive_init();

    hive_bus_config cfg = HIVE_BUS_CONFIG_DEFAULT;
    cfg.max_entries = 1;
    hive_bus_create(&cfg, &g_imu_bus);

    actor_id attitude;
    hive_spawn(attitude_actor, NULL, &attitude);

    printf("Pilot started - hover mode\n");

    while (wb_robot_step(TIME_STEP) != -1) {
        // Platform: read sensors → publish to bus
        imu_data_t imu;
        platform_read_imu(&imu);
        hive_bus_publish(g_imu_bus, &imu, sizeof(imu));

        // Run portable actors
        hive_step();

        // Platform: apply motor commands
        platform_write_motors();
    }

    hive_cleanup();
    wb_robot_cleanup();
    return 0;
}

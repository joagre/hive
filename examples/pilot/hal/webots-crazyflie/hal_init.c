// Webots Crazyflie HAL - Initialization
//
// Platform lifecycle: init, cleanup, self-test, calibration, arm/disarm.

#include "../hal.h"
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/gps.h>
#include <time.h>

#include "hal_internal.h"

// Hardware handles (shared with other HAL files)
WbDeviceTag g_motors[NUM_MOTORS];
WbDeviceTag g_gyro;
WbDeviceTag g_imu;
WbDeviceTag g_gps;

// PRNG state
uint32_t g_rng_state = 1;

// Sensor simulation state
float g_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
float g_prev_gps[3] = {0.0f, 0.0f, 0.0f};
bool g_prev_gps_valid = false;
float g_motor_state[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f};

int hal_init(void) {
    wb_robot_init();

    // Seed PRNG from wall clock for variety in noise patterns each run.
    g_rng_state = (uint32_t)time(NULL) ^ 0xDEADBEEF;
    if (g_rng_state == 0)
        g_rng_state = 12345; // xorshift can't have zero state

    // Initialize motors
    const char *motor_names[NUM_MOTORS] = {"m1_motor", "m2_motor", "m3_motor",
                                           "m4_motor"};

    for (int i = 0; i < NUM_MOTORS; i++) {
        g_motors[i] = wb_robot_get_device(motor_names[i]);
        if (g_motors[i] == 0) {
            return -1;
        }
        wb_motor_set_position(g_motors[i], INFINITY);
        wb_motor_set_velocity(g_motors[i], 0.0);
    }

    // Initialize sensors
    g_gyro = wb_robot_get_device("gyro");
    g_imu = wb_robot_get_device("inertial_unit");
    g_gps = wb_robot_get_device("gps");

    if (g_gyro == 0 || g_imu == 0 || g_gps == 0) {
        return -1;
    }

    wb_gyro_enable(g_gyro, TIME_STEP_MS);
    wb_inertial_unit_enable(g_imu, TIME_STEP_MS);
    wb_gps_enable(g_gps, TIME_STEP_MS);

    return 0;
}

void hal_cleanup(void) {
    wb_robot_cleanup();
}

bool hal_self_test(void) {
    // Webots sensors always work
    return true;
}

void hal_calibrate(void) {
    // No-op: Webots sensors don't need calibration
}

void hal_arm(void) {
    // No-op: Webots motors are always ready
}

void hal_disarm(void) {
    // Stop all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        wb_motor_set_velocity(g_motors[i], 0.0);
    }
}

// Simulation step
bool hal_step(void) {
    return wb_robot_step(TIME_STEP_MS) != -1;
}

// Simulated battery (always full in simulation)
float hal_power_get_battery(void) {
    return 4.2f;
}

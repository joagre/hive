// Crazyflie 2.1+ HAL - Motors
//
// Motor control with X-configuration mixer.

#include "../hal.h"
#include "platform.h"
#include "../../include/flight_profiles.h"
#include "hive_log.h"
#include <stdbool.h>

// Default MOTORS_DISABLED to 0 if not set by flight profile
#ifndef MOTORS_DISABLED
#define MOTORS_DISABLED 0
#endif

// Crazyflie 2.1+ X-configuration mixer
//
/*
 * Motor layout (viewed from above, Bitcraze standard):
 *
 *          Front
 *      M4(CW)     M1(CCW)
 *           \   /
 *            x
 *           /   \
 *      M3(CCW)   M2(CW)
 *          Back
 */
// Channel mapping:
//   M1 (front-right, CCW): TIM2_CH2 (PA1)
//   M2 (back-right, CW):   TIM2_CH4 (PB11)
//   M3 (back-left, CCW):   TIM2_CH1 (PA15)
//   M4 (front-left, CW):   TIM4_CH4 (PB9)
//
// Sign conventions (matching Bitcraze):
//   +roll  = right wing down (right motors less, left motors more)
//   +pitch = nose up (front motors more, back motors less)
//   +yaw   = CW rotation (CCW motors more, CW motors less)
//
// Mixer equations (from Bitcraze power_distribution_quadrotor.c):
//   M1 = thrust - roll + pitch + yaw  (front-right, CCW)
//   M2 = thrust - roll - pitch - yaw  (back-right, CW)
//   M3 = thrust + roll - pitch + yaw  (back-left, CCW)
//   M4 = thrust + roll + pitch - yaw  (front-left, CW)

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

void hal_write_torque(const torque_cmd_t *cmd) {
#if MOTORS_DISABLED
    (void)cmd; // Ground test mode - motors disabled
    return;
#endif

    // Apply mixer: convert torque to individual motor commands
    motor_cmd_t motors;

    // Mixer matches Bitcraze power_distribution_quadrotor.c (legacy mode)
    motors.motor[0] = cmd->thrust - cmd->roll + cmd->pitch +
                      cmd->yaw; // M1 (front-right, CCW)
    motors.motor[1] =
        cmd->thrust - cmd->roll - cmd->pitch - cmd->yaw; // M2 (back-right, CW)
    motors.motor[2] =
        cmd->thrust + cmd->roll - cmd->pitch + cmd->yaw; // M3 (back-left, CCW)
    motors.motor[3] =
        cmd->thrust + cmd->roll + cmd->pitch - cmd->yaw; // M4 (front-left, CW)

    // Attitude-priority mixing: if any motor exceeds 1.0, reduce base
    // thrust for all motors equally so attitude differentials (roll, pitch,
    // yaw) are preserved. Better to lose altitude than lose attitude control.
    float max_motor = motors.motor[0];
    for (int i = 1; i < 4; i++) {
        if (motors.motor[i] > max_motor) {
            max_motor = motors.motor[i];
        }
    }
    if (max_motor > 1.0f) {
        float reduction = max_motor - 1.0f;
        for (int i = 0; i < 4; i++) {
            motors.motor[i] -= reduction;
        }
        HIVE_LOG_DEBUG("[MIX] attitude priority: thrust reduced by %.3f",
                       reduction);
    }

    // Clamp to [0, 1] - lower bound only after attitude-priority reduction
    for (int i = 0; i < 4; i++) {
        motors.motor[i] = clampf(motors.motor[i], 0.0f, 1.0f);
    }

    // Output to hardware
    platform_write_motors(&motors);
}

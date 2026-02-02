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
// Motor layout (viewed from above):
//
//          Front
//      M1(CCW)  M2(CW)
//          +--+
//          |  |
//          +--+
//      M4(CW)  M3(CCW)
//          Rear
//
// Channel mapping (Bitcraze reference):
//   M1 (front-left, CCW):  TIM2_CH2 (PA1)
//   M2 (front-right, CW):  TIM2_CH4 (PB11)
//   M3 (rear-right, CCW):  TIM2_CH1 (PA15)
//   M4 (rear-left, CW):    TIM4_CH4 (PB9)
//
// Mixer equations (standard X-quad):
//   M1 = thrust - roll + pitch + yaw  (front-left, CCW)
//   M2 = thrust + roll + pitch - yaw  (front-right, CW)
//   M3 = thrust + roll - pitch + yaw  (rear-right, CCW)
//   M4 = thrust - roll - pitch - yaw  (rear-left, CW)

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

    motors.motor[0] =
        cmd->thrust - cmd->roll + cmd->pitch + cmd->yaw; // M1 (front-left, CCW)
    motors.motor[1] =
        cmd->thrust + cmd->roll + cmd->pitch - cmd->yaw; // M2 (front-right, CW)
    motors.motor[2] =
        cmd->thrust + cmd->roll - cmd->pitch + cmd->yaw; // M3 (rear-right, CCW)
    motors.motor[3] =
        cmd->thrust - cmd->roll - cmd->pitch - cmd->yaw; // M4 (rear-left, CW)

    // Clamp motor values and detect saturation
    bool saturated = false;
    for (int i = 0; i < 4; i++) {
        float orig = motors.motor[i];
        motors.motor[i] = clampf(motors.motor[i], 0.0f, 1.0f);
        if (orig != motors.motor[i]) {
            saturated = true;
        }
    }
    if (saturated) {
        HIVE_LOG_WARN("[MIX] motor saturation: t=%.2f r=%.2f p=%.2f y=%.2f",
                      cmd->thrust, cmd->roll, cmd->pitch, cmd->yaw);
    }

    // Output to hardware
    platform_write_motors(&motors);
}

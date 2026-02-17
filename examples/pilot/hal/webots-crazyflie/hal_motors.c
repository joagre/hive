// Webots Crazyflie HAL - Motors
//
// Motor control with X-configuration mixer and response lag simulation.

#include "../hal.h"
#include <webots/motor.h>

#include "hal_internal.h"
#include "hive_log.h"
#include <stdbool.h>

// Crazyflie X-configuration mixer
//
// Motor layout:
//         Front
//       M2    M3
//         \  /
//          \/
//          /\.
//         /  \.
//       M1    M4
//         Rear
//
// Motor rotation: M1(CCW), M2(CW), M3(CCW), M4(CW)

void hal_write_torque(const torque_cmd_t *cmd) {
    // Platform-specific sign corrections for Webots coordinate frame.
    // Pitch: Webots front/rear motor mapping is transposed vs Crazyflie.
    // Yaw: Compensates for rate_actor yaw negation (added in 3035967 for
    // Crazyflie hardware). The original Webots sim worked without any yaw
    // negation; the rate_actor change broke the sim. This restores the
    // correct net sign for Webots motor control.
    float pitch = -cmd->pitch;
    float yaw = -cmd->yaw;

    // Apply mixer: convert torque to individual motor commands
    float motors[NUM_MOTORS];
    motors[0] = cmd->thrust - cmd->roll + pitch + yaw; // M1 (rear-left)
    motors[1] = cmd->thrust - cmd->roll - pitch - yaw; // M2 (front-left)
    motors[2] = cmd->thrust + cmd->roll - pitch + yaw; // M3 (front-right)
    motors[3] = cmd->thrust + cmd->roll + pitch - yaw; // M4 (rear-right)

    // Apply motor response lag (first-order filter)
    float dt = TIME_STEP_MS;
    float tau = MOTOR_TIME_CONSTANT_MS;
    float alpha = dt / (tau + dt);

    // Clamp, filter, and output to Webots motors
    bool saturated = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        float cmd_clamped = CLAMPF(motors[i], 0.0f, 1.0f);
        if (motors[i] != cmd_clamped) {
            saturated = true;
        }

        // First-order lag filter
        g_motor_state[i] += alpha * (cmd_clamped - g_motor_state[i]);

        wb_motor_set_velocity(g_motors[i], MOTOR_SIGNS[i] * g_motor_state[i] *
                                               MOTOR_MAX_VELOCITY);
    }
    if (saturated) {
        HIVE_LOG_WARN("[MIX] motor saturation: t=%.2f r=%.2f p=%.2f y=%.2f",
                      cmd->thrust, cmd->roll, cmd->pitch, cmd->yaw);
    }
}

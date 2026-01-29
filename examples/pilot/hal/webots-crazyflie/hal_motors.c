// Webots Crazyflie HAL - Motors
//
// Motor control with X-configuration mixer and response lag simulation.

#include "../hal.h"
#include <webots/motor.h>

#include "hal_internal.h"

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
    // Platform-specific adjustment: negate pitch for Crazyflie coordinate frame
    float pitch = -cmd->pitch;

    // Apply mixer: convert torque to individual motor commands
    float motors[NUM_MOTORS];
    motors[0] = cmd->thrust - cmd->roll + pitch + cmd->yaw; // M1 (rear-left)
    motors[1] = cmd->thrust - cmd->roll - pitch - cmd->yaw; // M2 (front-left)
    motors[2] = cmd->thrust + cmd->roll - pitch + cmd->yaw; // M3 (front-right)
    motors[3] = cmd->thrust + cmd->roll + pitch - cmd->yaw; // M4 (rear-right)

    // Apply motor response lag (first-order filter)
    float dt = TIME_STEP_MS;
    float tau = MOTOR_TIME_CONSTANT_MS;
    float alpha = dt / (tau + dt);

    // Clamp, filter, and output to Webots motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        float cmd_clamped = CLAMPF(motors[i], 0.0f, 1.0f);

        // First-order lag filter
        g_motor_state[i] += alpha * (cmd_clamped - g_motor_state[i]);

        wb_motor_set_velocity(g_motors[i], MOTOR_SIGNS[i] * g_motor_state[i] *
                                               MOTOR_MAX_VELOCITY);
    }
}

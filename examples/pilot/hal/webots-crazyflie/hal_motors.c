// Webots Crazyflie HAL - Motors
//
// Motor control with X-configuration mixer and response lag simulation.

#include "../hal.h"
#include <webots/motor.h>

#include "hal_internal.h"
#include "hive_log.h"

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
    //
    // Pitch: Webots front/rear motor mapping is transposed relative to the
    // Crazyflie, requiring pitch negation for correct nose-up/down response.
    //
    // Yaw: The Webots gyro Z axis has opposite polarity from the Bitcraze
    // BMI088. The rate_actor already negates yaw for Bitcraze hardware,
    // so we negate again here to undo that and match the Webots convention.
    //
    // Roll: No negation needed - the Webots gyro X polarity difference
    // cancels the motor position transposition.
    float pitch = -cmd->pitch;
    float yaw = -cmd->yaw;

    // Apply mixer (same equation template as Crazyflie HAL)
    float motors[NUM_MOTORS];
    motors[0] = cmd->thrust - cmd->roll + pitch + yaw; // M1 (rear-left, CCW)
    motors[1] = cmd->thrust - cmd->roll - pitch - yaw; // M2 (front-left, CW)
    motors[2] = cmd->thrust + cmd->roll - pitch + yaw; // M3 (front-right, CCW)
    motors[3] = cmd->thrust + cmd->roll + pitch - yaw; // M4 (rear-right, CW)

    // Attitude-priority mixing: if any motor exceeds 1.0, reduce base
    // thrust for all motors equally so attitude differentials (roll, pitch,
    // yaw) are preserved. Better to lose altitude than lose attitude control.
    float max_motor = motors[0];
    for (int i = 1; i < NUM_MOTORS; i++) {
        if (motors[i] > max_motor) {
            max_motor = motors[i];
        }
    }
    if (max_motor > 1.0f) {
        float reduction = max_motor - 1.0f;
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i] -= reduction;
        }
        HIVE_LOG_DEBUG("[MIX] attitude priority: thrust reduced by %.3f",
                       reduction);
    }

    // Apply motor response lag (first-order filter)
    float dt = TIME_STEP_MS;
    float tau = MOTOR_TIME_CONSTANT_MS;
    float alpha = dt / (tau + dt);

    // Clamp to [0, 1], filter, and output to Webots motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        float cmd_clamped = CLAMPF(motors[i], 0.0f, 1.0f);

        // First-order lag filter
        g_motor_state[i] += alpha * (cmd_clamped - g_motor_state[i]);

        wb_motor_set_velocity(g_motors[i], MOTOR_SIGNS[i] * g_motor_state[i] *
                                               MOTOR_MAX_VELOCITY);
    }
}

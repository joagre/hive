// STEVAL-DRONE01 HAL Implementation
//
// Hardware abstraction for STM32F4 STEVAL-DRONE01 board.
// Wraps existing platform functions and adds common HAL interface.

#include "../hal.h"
#include "platform_stm32f4.h"
#include "steval_fcu001_v1.h" // For BSP_LED_Toggle

// ----------------------------------------------------------------------------
// Platform Lifecycle
// ----------------------------------------------------------------------------

int hal_init(void) {
    return platform_init();
}

void hal_cleanup(void) {
    platform_disarm();
}

void hal_calibrate(void) {
    platform_calibrate();
}

void hal_arm(void) {
    platform_arm();
}

void hal_disarm(void) {
    platform_disarm();
}

// ----------------------------------------------------------------------------
// Sensor Interface
// ----------------------------------------------------------------------------

void hal_read_sensors(sensor_data_t *sensors) {
    // Read raw sensor data from platform
    platform_read_sensors(sensors);
}

// ----------------------------------------------------------------------------
// Motor Interface
// ----------------------------------------------------------------------------

// clang-format off
// STEVAL-DRONE01 X-configuration mixer
//
// Motor layout:
//          Front
//      M2(CW)  M3(CCW)
//       P2  \  /  P4
//            \/
//            /\
//       P1  /  \  P5
//      M1(CCW) M4(CW)
//          Rear
//
// Channel to connector mapping:
//   M1 (rear-left, CCW)   -> P1 (TIM4_CH1, PB6)
//   M2 (front-left, CW)   -> P2 (TIM4_CH2, PB7)
//   M3 (front-right, CCW) -> P4 (TIM4_CH3, PB8)
//   M4 (rear-right, CW)   -> P5 (TIM4_CH4, PB9)
//
// Note: Board connectors are labeled P1, P2, P4, P5 (no P3).
// clang-format on

void hal_write_torque(const torque_cmd_t *cmd) {
    // Apply mixer: convert torque to individual motor commands
    // Signs matched to Webots simulation (validated in sim, pitch sign
    // inverted)
    motor_cmd_t motors;
    motors.motor[0] =
        cmd->thrust - cmd->roll - cmd->pitch + cmd->yaw; // M1 (rear-left)
    motors.motor[1] =
        cmd->thrust - cmd->roll + cmd->pitch - cmd->yaw; // M2 (front-left)
    motors.motor[2] =
        cmd->thrust + cmd->roll + cmd->pitch + cmd->yaw; // M3 (front-right)
    motors.motor[3] =
        cmd->thrust + cmd->roll - cmd->pitch - cmd->yaw; // M4 (rear-right)

    // Clamp motor values
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors.motor[i] = CLAMPF(motors.motor[i], 0.0f, 1.0f);
    }

    // Output to hardware
    platform_write_motors(&motors);
}

// ----------------------------------------------------------------------------
// Debug
// ----------------------------------------------------------------------------

void hal_debug_toggle_led(void) {
    BSP_LED_Toggle(LED1);
}

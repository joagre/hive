// Motor PWM driver for STEVAL-DRONE01
//
// 4 brushed DC motors via TIM4 PWM channels.
// Motor layout (X configuration, matching pilot example):
//
//         Front
//       M2    M3
//         \  /
//          \/
//          /\
//         /  \
//       M1    M4
//         Rear
//
// M1 (rear-left):   TIM4_CH1, CCW
// M2 (front-left):  TIM4_CH2, CW
// M3 (front-right): TIM4_CH3, CCW
// M4 (rear-right):  TIM4_CH4, CW

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

#define MOTORS_COUNT 4

// Motor indices (matching pilot example motor_cmd_t)
#define MOTOR_M1_REAR_LEFT   0
#define MOTOR_M2_FRONT_LEFT  1
#define MOTOR_M3_FRONT_RIGHT 2
#define MOTOR_M4_REAR_RIGHT  3

// Motor command (normalized 0.0 to 1.0)
typedef struct {
    float motor[MOTORS_COUNT];
} motors_cmd_t;

#define MOTORS_CMD_ZERO {.motor = {0.0f, 0.0f, 0.0f, 0.0f}}

// PWM configuration
typedef struct {
    uint32_t frequency_hz;  // PWM frequency (e.g., 20000 for 20kHz)
    uint16_t min_pulse;     // Minimum pulse width (motor off threshold)
    uint16_t max_pulse;     // Maximum pulse width (full throttle)
} motors_config_t;

// Default: 20kHz PWM, 0-100% range
#define MOTORS_CONFIG_DEFAULT { \
    .frequency_hz = 20000, \
    .min_pulse = 0, \
    .max_pulse = 1000 \
}

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize motor PWM outputs.
// Default configuration uses TIM4 CH3/CH4 only (PB8/PB9) to avoid I2C1 conflict.
// Motors start in stopped state.
bool motors_init(const motors_config_t *config);

// Initialize all 4 motor channels.
// use_port_d: true = use PD12-PD15, false = use PB6-PB9 (conflicts with I2C1!)
// For full quad support with I2C sensors, use port D pins.
bool motors_init_full(const motors_config_t *config, bool use_port_d);

// Arm the motors (enable PWM output).
// Must be called before motors will spin.
void motors_arm(void);

// Disarm the motors (disable PWM output, motors stop immediately).
void motors_disarm(void);

// Check if motors are armed.
bool motors_is_armed(void);

// Set motor speeds.
// Values are normalized 0.0 (stopped) to 1.0 (full throttle).
// Values outside this range are clamped.
void motors_set(const motors_cmd_t *cmd);

// Set individual motor speed (0.0 to 1.0).
void motors_set_single(uint8_t motor, float value);

// Stop all motors immediately (sets all to 0).
void motors_stop(void);

// Emergency stop - disarms and stops all motors.
void motors_emergency_stop(void);

// Get current PWM values (for debugging).
void motors_get_pwm(uint16_t pwm[MOTORS_COUNT]);

#endif // MOTORS_H

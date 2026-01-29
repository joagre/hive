// Motor PWM Driver for Crazyflie 2.1+
//
// TIM2/TIM4 PWM output for 4 brushed coreless motors.
// Pin mapping matches Bitcraze reference implementation.
//
// Motor layout (X-configuration, viewed from above):
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

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

#define MOTORS_COUNT 4

// PWM frequency options
typedef enum {
    MOTORS_PWM_328KHZ = 0, // Standard for Crazyflie
    MOTORS_PWM_50KHZ = 1,  // Lower frequency option
} motors_pwm_freq_t;

// Configuration structure
typedef struct {
    motors_pwm_freq_t frequency;
    uint16_t min_pulse; // Minimum PWM value (motor off)
    uint16_t max_pulse; // Maximum PWM value (full throttle)
} motors_config_t;

// Default configuration
#define MOTORS_CONFIG_DEFAULT \
    { .frequency = MOTORS_PWM_328KHZ, .min_pulse = 0, .max_pulse = 255 }

// ----------------------------------------------------------------------------
// Motor Command Structure
// ----------------------------------------------------------------------------

typedef struct {
    float motor[MOTORS_COUNT]; // Normalized values 0.0 to 1.0
} motors_cmd_t;

#define MOTORS_CMD_ZERO                     \
    {                                       \
        .motor = { 0.0f, 0.0f, 0.0f, 0.0f } \
    }

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

// Initialize motor PWM
// config: Configuration (or NULL for defaults)
// Returns: true on success
bool motors_init(const motors_config_t *config);

// Arm motors (enable PWM output)
void motors_arm(void);

// Disarm motors (disable PWM output)
void motors_disarm(void);

// Check if motors are armed
bool motors_is_armed(void);

// Set all motor speeds
// cmd: Normalized motor commands (0.0 to 1.0)
void motors_set(const motors_cmd_t *cmd);

// Set single motor speed
// motor: Motor index (0-3)
// value: Normalized speed (0.0 to 1.0)
void motors_set_single(uint8_t motor, float value);

// Stop all motors (set to zero)
void motors_stop(void);

// Emergency stop (immediate stop and disarm)
void motors_emergency_stop(void);

// Get current PWM values (for debugging)
void motors_get_pwm(uint16_t pwm[MOTORS_COUNT]);

// Set motor ratio directly (bypass normalization)
// For testing/calibration only
void motors_set_ratio(uint8_t motor, uint16_t ratio);

#endif // MOTORS_H

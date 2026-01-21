// Crazyflie 2.1+ Bring-Up - Motor Tests
//
// Motor PWM via TIM2 channels:
//   M1 (front-left, CCW)  - PA0 - TIM2_CH1
//   M2 (front-right, CW)  - PA1 - TIM2_CH2
//   M3 (rear-right, CCW)  - PA2 - TIM2_CH3
//   M4 (rear-left, CW)    - PA3 - TIM2_CH4

#ifndef BRINGUP_MOTORS_H
#define BRINGUP_MOTORS_H

#include <stdbool.h>
#include <stdint.h>

// Motor identifiers
typedef enum {
    MOTOR_M1 = 0, // Front-left, CCW
    MOTOR_M2 = 1, // Front-right, CW
    MOTOR_M3 = 2, // Rear-right, CCW
    MOTOR_M4 = 3, // Rear-left, CW
    MOTOR_COUNT = 4
} motor_id_t;

// Motor info
typedef struct {
    const char *name;
    const char *position;
    const char *rotation;
} motor_info_t;

// Initialize motor PWM (TIM2)
void motors_init(void);

// Set motor duty cycle (0.0 - 1.0)
void motor_set(motor_id_t motor, float duty);

// Stop all motors
void motors_stop(void);

// Get motor info
const motor_info_t *motor_get_info(motor_id_t motor);

// Run motor test sequence (spins each motor individually)
// Returns true if user confirmed test, false if skipped
// Uses UART for user interaction
bool motors_run_test(void);

// Arm/disarm motors (safety)
void motors_arm(void);
void motors_disarm(void);
bool motors_is_armed(void);

#endif // BRINGUP_MOTORS_H

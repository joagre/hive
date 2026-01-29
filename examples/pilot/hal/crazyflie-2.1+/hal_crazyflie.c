// Crazyflie 2.1+ HAL Implementation
//
// Hardware abstraction for Crazyflie 2.1+ drone.
// Wraps platform functions and adds common HAL interface.

#include "../hal.h"
#include "debug_swo.h"
#include "platform_crazyflie.h"
#include <stdarg.h>

// Crazyflie 2.1+ runs at 168 MHz
#define CPU_FREQ_HZ 168000000
#define SWO_BAUD 2000000

// ----------------------------------------------------------------------------
// Platform Lifecycle
// ----------------------------------------------------------------------------

int hal_init(void) {
    // Initialize SWO debug output first so we can log during init
    debug_swo_init(CPU_FREQ_HZ, SWO_BAUD);
    return platform_init();
}

void hal_cleanup(void) {
    platform_disarm();
}

bool hal_self_test(void) {
    return platform_self_test();
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
    platform_read_sensors(sensors);
}

// ----------------------------------------------------------------------------
// Motor Interface
// ----------------------------------------------------------------------------

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

// Clamp helper
static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

void hal_write_torque(const torque_cmd_t *cmd) {
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

    // Clamp motor values
    for (int i = 0; i < 4; i++) {
        motors.motor[i] = clampf(motors.motor[i], 0.0f, 1.0f);
    }

    // Output to hardware
    platform_write_motors(&motors);
}

// ----------------------------------------------------------------------------
// Platform Utilities
// ----------------------------------------------------------------------------

void hal_delay_ms(uint32_t ms) {
    platform_delay_ms(ms);
}

uint32_t hal_get_time_ms(void) {
    return platform_get_time_ms();
}

void hal_led_on(void) {
    platform_led_on();
}

void hal_led_off(void) {
    platform_led_off();
}

void hal_led_toggle(void) {
    platform_led_toggle();
}

// ----------------------------------------------------------------------------
// Debug Output
// ----------------------------------------------------------------------------

void hal_debug_init(void) {
    // SWO already initialized in hal_init(), nothing to do here
}

void hal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    debug_swo_vprintf(fmt, args);
    va_end(args);
}

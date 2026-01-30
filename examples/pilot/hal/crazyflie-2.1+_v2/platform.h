// Crazyflie 2.1+ Platform Layer (v2)
//
// Provides the platform interface for the Crazyflie 2.1+ hardware.
// Uses DMA/interrupt-based I2C driver from Bitcraze for reliable sensor access.

#ifndef PLATFORM_H
#define PLATFORM_H

#include "types.h"
#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Platform Interface
// ----------------------------------------------------------------------------

// Initialize all hardware: clocks, GPIO, sensors, motors.
int platform_init(void);

// Read raw sensor data from sensors.
void platform_read_sensors(sensor_data_t *sensors);

// Write motor commands to TIM2 PWM.
void platform_write_motors(const motor_cmd_t *cmd);

// ----------------------------------------------------------------------------
// Extended Platform Interface
// ----------------------------------------------------------------------------

// Self-test hardware: verify sensors respond and return sane values.
bool platform_self_test(void);

// Calibrate sensors (gyro bias, barometer reference).
int platform_calibrate(void);

// Arm/disarm motors.
void platform_arm(void);
void platform_disarm(void);

// Get timing information.
uint32_t platform_get_time_ms(void);
uint32_t platform_get_time_us(void);

// Delay functions.
void platform_delay_ms(uint32_t ms);
void platform_delay_us(uint32_t us);

// Debug output.
void platform_debug_init(void);
void platform_debug_printf(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

// Emergency stop - immediately stop all motors.
void platform_emergency_stop(void);

// ----------------------------------------------------------------------------
// Flow Deck Interface (not implemented in v2)
// ----------------------------------------------------------------------------

bool platform_has_flow_deck(void);
bool platform_read_flow(int16_t *delta_x, int16_t *delta_y);
bool platform_read_height(uint16_t *height_mm);

// ----------------------------------------------------------------------------
// LED Control
// ----------------------------------------------------------------------------

void platform_led_on(void);
void platform_led_off(void);
void platform_led_toggle(void);

#endif // PLATFORM_H

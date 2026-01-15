// Crazyflie 2.1+ Platform Layer
//
// Provides the platform interface for the Crazyflie 2.1+ hardware.
// This layer ties together all the sensor drivers and implements
// the common platform API used by hal_crazyflie.c.

#ifndef PLATFORM_CRAZYFLIE_H
#define PLATFORM_CRAZYFLIE_H

#include "../../types.h"
#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Platform Interface
// ----------------------------------------------------------------------------

// Initialize all hardware: clocks, GPIO, sensors, motors.
// Returns 0 on success, -1 on error.
int platform_init(void);

// Read raw sensor data from sensors.
// Populates accel/gyro from BMI088 (gyro is bias-corrected).
// Populates pressure from BMP388 barometer.
// Populates flow data from PMW3901 (if Flow deck present).
// Populates height from VL53L1x (if Flow deck present).
void platform_read_sensors(sensor_data_t *sensors);

// Write motor commands to TIM2 PWM.
// Values in cmd->motor[0..3] are normalized 0.0 to 1.0.
void platform_write_motors(const motor_cmd_t *cmd);

// ----------------------------------------------------------------------------
// Extended Platform Interface
// ----------------------------------------------------------------------------

// Calibrate sensors (gyro bias, barometer reference).
// Call after platform_init(), keep drone still and level.
// Returns 0 on success, -1 on error.
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

// Debug output (requires UART initialized).
void platform_debug_init(void);
void platform_debug_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Emergency stop - immediately stop all motors.
void platform_emergency_stop(void);

// ----------------------------------------------------------------------------
// Flow Deck Interface
// ----------------------------------------------------------------------------

// Check if Flow deck is present
bool platform_has_flow_deck(void);

// Read optical flow (pixels/frame)
bool platform_read_flow(int16_t *delta_x, int16_t *delta_y);

// Read height (mm)
bool platform_read_height(uint16_t *height_mm);

// ----------------------------------------------------------------------------
// LED Control
// ----------------------------------------------------------------------------

void platform_led_on(void);
void platform_led_off(void);
void platform_led_toggle(void);

#endif // PLATFORM_CRAZYFLIE_H

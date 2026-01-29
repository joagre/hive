// Hardware Abstraction Layer - Common Interface
//
// All hardware platforms implement this interface. Actors use only these
// functions, making them completely hardware-independent.
//
// Coordinate conventions (all platforms must conform):
//   Roll:  positive = right wing down
//   Pitch: positive = nose up
//   Yaw:   positive = clockwise when viewed from above
//   Torque: positive command produces positive rotation
//
// Implementations:
//   hal/crazyflie-2.1+/   - Crazyflie 2.1+ real hardware
//   hal/webots-crazyflie/ - Webots simulation

#ifndef HAL_H
#define HAL_H

#include "types.h"
#include "config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ----------------------------------------------------------------------------
// Platform Lifecycle
// ----------------------------------------------------------------------------

// Initialize hardware: clocks, GPIO, sensors, motors, communication.
// Returns 0 on success, -1 on error.
int hal_init(void);

// Cleanup and release resources.
void hal_cleanup(void);

// Self-test hardware: verify sensors respond and return sane values.
// Call after hal_init(), before hal_calibrate().
// Returns true if all required hardware is functional, false otherwise.
// Always returns true on simulation platforms.
bool hal_self_test(void);

// Calibrate sensors (gyro bias, barometer reference, etc.).
// Call after hal_self_test(), keep drone still and level.
// No-op on platforms that don't need calibration (e.g., simulation).
void hal_calibrate(void);

// Arm motors (enable output).
// No-op on platforms that don't need arming.
void hal_arm(void);

// Disarm motors (disable output).
// No-op on platforms that don't need arming.
void hal_disarm(void);

// ----------------------------------------------------------------------------
// Sensor Interface
// ----------------------------------------------------------------------------

// Read raw sensor data from hardware.
// Returns raw accel, gyro, and optionally mag/baro/GPS.
// Sensor fusion is done in the estimator actor using the portable
// complementary filter (fusion/complementary_filter.c).
void hal_read_sensors(sensor_data_t *sensors);

// ----------------------------------------------------------------------------
// Motor Interface
// ----------------------------------------------------------------------------

// Write torque command to motors.
// HAL handles mixing (converting torque to individual motor commands).
// Torque values use standard conventions (see above).
void hal_write_torque(const torque_cmd_t *cmd);

// ----------------------------------------------------------------------------
// Platform-Specific Constants
// ----------------------------------------------------------------------------

// HAL_BASE_THRUST is defined in each platform's hal_config.h:
//   hal/crazyflie-2.1+/hal_config.h
//   hal/webots-crazyflie/hal_config.h
// The appropriate file is included via the platform-specific Makefile.
// This keeps platform constants in platform directories (no #ifdefs here).

// ----------------------------------------------------------------------------
// Platform Utilities
// ----------------------------------------------------------------------------

// Delay for specified milliseconds.
// On real hardware: busy-wait or timer-based delay.
// On simulation: advances simulation time.
void hal_delay_ms(uint32_t ms);

// Get current time in milliseconds.
// On real hardware: system tick counter.
// On simulation: simulation time.
uint32_t hal_get_time_ms(void);

// Debug LED control (optional, may be no-op on some platforms).
void hal_led_on(void);
void hal_led_off(void);
void hal_led_toggle(void);

// Debug output (UART on hardware, printf on simulation).
// hal_debug_init() must be called before hal_printf().
// On Webots: No-op init, uses standard printf.
// On Crazyflie: Initializes USART1 at 115200 baud (PA9/PA10 via ST-Link VCP).
void hal_debug_init(void);
void hal_printf(const char *fmt, ...);

// ----------------------------------------------------------------------------
// ESB Radio Interface (optional, for telemetry)
// ----------------------------------------------------------------------------
// Enhanced ShockBurst (ESB) is the 2.4GHz protocol used by Crazyradio.
// On Crazyflie, implemented via syslink to nRF51 (see hal_syslink.c).

#ifdef HAL_HAS_RADIO

// Initialize ESB radio.
// Returns 0 on success, -1 on error.
int hal_esb_init(void);

// Send data over ESB.
// Returns 0 on success, -1 on error (not ready or too large).
// Max payload: 31 bytes (ESB packet limit).
int hal_esb_send(const void *data, size_t len);

// Check if ESB is ready to send.
// ESB is half-duplex: can only TX after receiving from ground.
bool hal_esb_tx_ready(void);

// Poll for incoming ESB packets.
// Call periodically to process RX data.
void hal_esb_poll(void);

// Register callback for received ESB data.
// Callback is called from hal_esb_poll() context.
// user_data is passed back to callback for context.
void hal_esb_set_rx_callback(void (*callback)(const void *data, size_t len,
                                              void *user_data),
                             void *user_data);

#endif // HAL_HAS_RADIO

// ----------------------------------------------------------------------------
// Power Interface (optional)
// ----------------------------------------------------------------------------

#ifdef HAL_HAS_RADIO

// Get battery voltage.
// On Crazyflie: received via syslink from nRF51 power management.
// Returns 0.0 if not yet available.
float hal_power_get_battery(void);

#endif // HAL_HAS_RADIO

// ----------------------------------------------------------------------------
// Simulated Time Interface (only for simulation platforms)
// ----------------------------------------------------------------------------

#ifdef SIMULATED_TIME

// Advance simulation by one time step.
// Returns true if simulation should continue, false if done.
bool hal_step(void);

// Get simulation time step in microseconds.
#define HAL_TIME_STEP_US (TIME_STEP_MS * 1000)

#endif // SIMULATED_TIME

#endif // HAL_H

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

#include "../types.h"
#include "../config.h"
#include <stdbool.h>
#include <stddef.h>

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
// Radio Interface (optional, for telemetry)
// ----------------------------------------------------------------------------

#ifdef HAL_HAS_RADIO

// Initialize radio hardware.
// Returns 0 on success, -1 on error.
int hal_radio_init(void);

// Send data over radio.
// Returns 0 on success, -1 on error (not ready or too large).
// Max payload: 31 bytes (ESB limit).
int hal_radio_send(const void *data, size_t len);

// Check if radio is ready to send.
// Flow control: can only send after receiving a packet from ground.
bool hal_radio_tx_ready(void);

// Poll for incoming radio packets.
// Call periodically to process RX data.
void hal_radio_poll(void);

// Register callback for received radio data.
// Callback is called from hal_radio_poll() context.
// user_data is passed back to callback for context.
void hal_radio_set_rx_callback(void (*callback)(const void *data, size_t len,
                                                void *user_data),
                               void *user_data);

// Get battery voltage from power management packets.
// Returns 0.0 if not yet received.
float hal_radio_get_battery(void);

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

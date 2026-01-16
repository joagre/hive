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
//   hal/STEVAL-DRONE01/   - STEVAL-DRONE01 real hardware
//   hal/webots-crazyflie/ - Webots simulation

#ifndef HAL_H
#define HAL_H

#include "../types.h"
#include "../config.h"
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Platform Lifecycle
// ----------------------------------------------------------------------------

// Initialize hardware: clocks, GPIO, sensors, motors, communication.
// Returns 0 on success, -1 on error.
int hal_init(void);

// Cleanup and release resources.
void hal_cleanup(void);

// Calibrate sensors (gyro bias, barometer reference, etc.).
// Call after hal_init(), keep drone still and level.
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
//   hal/STEVAL-DRONE01/hal_config.h
//   hal/webots-crazyflie/hal_config.h
// The appropriate file is included via the platform-specific Makefile.
// This keeps platform constants in platform directories (no #ifdefs here).

// ----------------------------------------------------------------------------
// Simulated Time Interface (only for simulation platforms)
// ----------------------------------------------------------------------------

#ifdef SIMULATED_TIME

// Advance simulation by one time step.
// Returns true if simulation should continue, false if done.
bool hal_step(void);

// Get simulation time step in microseconds.
#define HAL_TIME_STEP_US  (TIME_STEP_MS * 1000)

#endif // SIMULATED_TIME

#endif // HAL_H

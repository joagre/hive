/*
 * VL53L1X Ultra Lite Driver (C port)
 *
 * Ported from ST/mbed ULD to plain C.
 * Original: https://github.com/VRaktion/mbed-VL53L1X-ULD
 *
 * Copyright (c) 2017, STMicroelectronics - All Rights Reserved
 *
 * This file is part of VL53L1 Core and is dual licensed,
 * either 'STMicroelectronics Proprietary license'
 * or 'BSD 3-clause "New" or "Revised" License', at your option.
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VL53L1X_ULD_H
#define VL53L1X_ULD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default I2C address (7-bit)
#define VL53L1X_DEFAULT_ADDRESS 0x29

// Distance modes
typedef enum {
    VL53L1X_DISTANCE_MODE_SHORT = 1,  // Up to 1.3m
    VL53L1X_DISTANCE_MODE_LONG = 2    // Up to 4m
} vl53l1x_distance_mode_t;

// Timing budget options (in ms)
typedef enum {
    VL53L1X_TIMING_15MS = 15,   // Short mode only
    VL53L1X_TIMING_20MS = 20,
    VL53L1X_TIMING_33MS = 33,
    VL53L1X_TIMING_50MS = 50,
    VL53L1X_TIMING_100MS = 100,
    VL53L1X_TIMING_200MS = 200,
    VL53L1X_TIMING_500MS = 500
} vl53l1x_timing_budget_t;

// Range status
typedef enum {
    VL53L1X_RANGE_VALID = 0,
    VL53L1X_RANGE_SIGMA_FAIL = 1,
    VL53L1X_RANGE_SIGNAL_FAIL = 2,
    VL53L1X_RANGE_OUT_OF_BOUNDS = 4,
    VL53L1X_RANGE_HARDWARE_FAIL = 5,
    VL53L1X_RANGE_WRAP_AROUND = 7,
    VL53L1X_RANGE_NO_TARGET = 255
} vl53l1x_range_status_t;

// Measurement result
typedef struct {
    uint8_t status;        // Range status
    uint16_t distance;     // Distance in mm
    uint16_t ambient;      // Ambient rate
    uint16_t signal_per_spad;
    uint8_t num_spads;
} vl53l1x_result_t;

// Platform abstraction callbacks - must be provided by user
typedef struct {
    // I2C write: send data to register (16-bit register address)
    // Returns 0 on success, non-zero on error
    int (*i2c_write)(uint8_t dev_addr, uint16_t reg_addr, const uint8_t *data, uint16_t len);

    // I2C read: read data from register (16-bit register address)
    // Returns 0 on success, non-zero on error
    int (*i2c_read)(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t len);

    // Delay in milliseconds
    void (*delay_ms)(uint32_t ms);
} vl53l1x_platform_t;

// Device handle
typedef struct {
    const vl53l1x_platform_t *platform;
    uint8_t i2c_address;
} vl53l1x_dev_t;

// Initialize device with platform callbacks
// Returns 0 on success
int8_t vl53l1x_init(vl53l1x_dev_t *dev, const vl53l1x_platform_t *platform);

// Software reset
int8_t vl53l1x_reset(vl53l1x_dev_t *dev);

// Initialize sensor (loads default configuration)
int8_t vl53l1x_sensor_init(vl53l1x_dev_t *dev);

// Check if sensor has booted
int8_t vl53l1x_boot_state(vl53l1x_dev_t *dev, uint8_t *state);

// Get sensor ID (should return 0xEACC for VL53L1X)
int8_t vl53l1x_get_sensor_id(vl53l1x_dev_t *dev, uint16_t *sensor_id);

// Set distance mode (short or long)
int8_t vl53l1x_set_distance_mode(vl53l1x_dev_t *dev, vl53l1x_distance_mode_t mode);

// Set timing budget
int8_t vl53l1x_set_timing_budget(vl53l1x_dev_t *dev, vl53l1x_timing_budget_t budget_ms);

// Set inter-measurement period (must be >= timing budget + 4ms)
int8_t vl53l1x_set_inter_measurement(vl53l1x_dev_t *dev, uint32_t period_ms);

// Start continuous ranging
int8_t vl53l1x_start_ranging(vl53l1x_dev_t *dev);

// Stop ranging
int8_t vl53l1x_stop_ranging(vl53l1x_dev_t *dev);

// Check if new data is ready
int8_t vl53l1x_check_data_ready(vl53l1x_dev_t *dev, uint8_t *is_ready);

// Get distance measurement in mm
int8_t vl53l1x_get_distance(vl53l1x_dev_t *dev, uint16_t *distance);

// Get range status
int8_t vl53l1x_get_range_status(vl53l1x_dev_t *dev, uint8_t *status);

// Get full result (status, distance, ambient, signal)
int8_t vl53l1x_get_result(vl53l1x_dev_t *dev, vl53l1x_result_t *result);

// Clear interrupt (must be called after reading data)
int8_t vl53l1x_clear_interrupt(vl53l1x_dev_t *dev);

// Set offset calibration in mm
int8_t vl53l1x_set_offset(vl53l1x_dev_t *dev, int16_t offset_mm);

// Get offset calibration
int8_t vl53l1x_get_offset(vl53l1x_dev_t *dev, int16_t *offset_mm);

#ifdef __cplusplus
}
#endif

#endif // VL53L1X_ULD_H

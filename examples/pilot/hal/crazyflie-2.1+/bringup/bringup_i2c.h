// Crazyflie 2.1+ Bring-Up - I2C3 Bus Scan and Communication
//
// I2C3: PA8 (SCL), PC9 (SDA) at 400 kHz

#ifndef BRINGUP_I2C_H
#define BRINGUP_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Known I2C addresses on Crazyflie 2.1+
#define I2C_ADDR_BMI088_ACCEL 0x18
#define I2C_ADDR_BMI088_GYRO 0x68
#define I2C_ADDR_BMP388 0x77
#define I2C_ADDR_VL53L1X 0x29

// Initialize I2C3 peripheral
void i2c_init(void);

// Scan I2C bus for devices (7-bit addresses 0x08-0x77)
// Returns number of devices found
// found_addrs: array to store found addresses (at least 16 elements)
int i2c_scan(uint8_t *found_addrs, int max_addrs);

// Check if device at address responds
bool i2c_probe(uint8_t addr);

// Write data to device
// Returns true on success
bool i2c_write(uint8_t addr, const uint8_t *data, size_t len);

// Read data from device
// Returns true on success
bool i2c_read(uint8_t addr, uint8_t *data, size_t len);

// Write register then read (combined transaction)
// Returns true on success
bool i2c_write_read(uint8_t addr, const uint8_t *write_data, size_t write_len,
                    uint8_t *read_data, size_t read_len);

// Read single register
bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);

// Read multiple registers
bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

// Write single register
bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

// Get human-readable name for known addresses
const char *i2c_device_name(uint8_t addr);

// Bus recovery (toggle SCL to unstick SDA)
void i2c_recover(void);

#endif // BRINGUP_I2C_H

// I2C1 driver for STM32F401 (STEVAL-DRONE01)
//
// Configured for LIS2MDL magnetometer and LPS22HD barometer.
// Mode: Master, 7-bit addressing, 400kHz Fast Mode

#ifndef I2C1_H
#define I2C1_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// I2C clock speed options
typedef enum {
    I2C1_SPEED_100KHZ,    // Standard mode (100 kHz)
    I2C1_SPEED_400KHZ     // Fast mode (400 kHz)
} i2c1_speed_t;

// Default: 400 kHz (both LIS2MDL and LPS22HD support Fast Mode)
#define I2C1_DEFAULT_SPEED  I2C1_SPEED_400KHZ

// I2C status codes
typedef enum {
    I2C1_OK = 0,          // Success
    I2C1_ERR_BUSY,        // Bus is busy
    I2C1_ERR_NACK,        // No acknowledge received
    I2C1_ERR_TIMEOUT,     // Operation timed out
    I2C1_ERR_BUS          // Bus error (arbitration lost, etc.)
} i2c1_status_t;

// Timeout in milliseconds for I2C operations
#define I2C1_TIMEOUT_MS     10

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize I2C1 peripheral
// Configures for master mode, 7-bit addressing
void i2c1_init(i2c1_speed_t speed);

// Deinitialize I2C1
void i2c1_deinit(void);

// Set I2C clock speed
void i2c1_set_speed(i2c1_speed_t speed);

// Write data to device
// addr: 7-bit I2C address (not shifted)
// data: Buffer to write
// len: Number of bytes to write
// Returns: I2C1_OK on success, error code otherwise
i2c1_status_t i2c1_write(uint8_t addr, const uint8_t *data, uint16_t len);

// Read data from device
// addr: 7-bit I2C address (not shifted)
// data: Buffer to store read data
// len: Number of bytes to read
// Returns: I2C1_OK on success, error code otherwise
i2c1_status_t i2c1_read(uint8_t addr, uint8_t *data, uint16_t len);

// Write then read (combined transaction with repeated start)
// addr: 7-bit I2C address (not shifted)
// tx_data: Buffer to write
// tx_len: Number of bytes to write
// rx_data: Buffer to store read data
// rx_len: Number of bytes to read
// Returns: I2C1_OK on success, error code otherwise
i2c1_status_t i2c1_write_read(uint8_t addr,
                               const uint8_t *tx_data, uint16_t tx_len,
                               uint8_t *rx_data, uint16_t rx_len);

// Write single register
// addr: 7-bit I2C address
// reg: Register address
// value: Value to write
// Returns: I2C1_OK on success, error code otherwise
i2c1_status_t i2c1_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

// Read single register
// addr: 7-bit I2C address
// reg: Register address
// value: Pointer to store read value
// Returns: I2C1_OK on success, error code otherwise
i2c1_status_t i2c1_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);

// Read multiple registers (burst read)
// addr: 7-bit I2C address
// reg: Starting register address
// data: Buffer to store read data
// len: Number of bytes to read
// Returns: I2C1_OK on success, error code otherwise
i2c1_status_t i2c1_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

// Check if device is present on bus
// addr: 7-bit I2C address
// Returns: true if device responds with ACK
bool i2c1_probe(uint8_t addr);

// Check if I2C bus is busy
bool i2c1_is_busy(void);

// Reset I2C bus (for recovery from stuck state)
void i2c1_reset(void);

#endif // I2C1_H

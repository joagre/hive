// Crazyflie 2.1+ Bring-Up - I2C1 Bus (Expansion Connector)
//
// I2C1: PB6 (SCL), PB7 (SDA) at 400 kHz
// Used for: EEPROM, VL53L1x (Flow deck)

#ifndef BRINGUP_I2C1_H
#define BRINGUP_I2C1_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Known I2C1 addresses on expansion connector
#define I2C1_ADDR_EEPROM 0x50
#define I2C1_ADDR_VL53L1X 0x29

// Initialize I2C1 peripheral
void i2c1_init(void);

// Check if device at address responds
bool i2c1_probe(uint8_t addr);

// Write data to device
bool i2c1_write(uint8_t addr, const uint8_t *data, size_t len);

// Write then read (combined transaction)
bool i2c1_write_read(uint8_t addr, const uint8_t *write_data, size_t write_len,
                     uint8_t *read_data, size_t read_len);

// Read single register (8-bit register address)
bool i2c1_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);

// Read single register (16-bit register address, for VL53L1x)
bool i2c1_read_reg16(uint8_t addr, uint16_t reg, uint8_t *value);

// Read multiple registers (16-bit register address, for VL53L1x)
bool i2c1_read_regs16(uint8_t addr, uint16_t reg, uint8_t *data, size_t len);

#endif // BRINGUP_I2C1_H

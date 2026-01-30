// I2C Device Layer Header - Bare-metal adaptation of Bitcraze crazyflie-firmware
//
// Based on crazyflie-firmware/src/drivers/interface/i2cdev.h
// High-level I2C functions for reading/writing registers.
//
// Original copyright:
// Copyright (C) 2011-2012 Bitcraze AB
// GPL-3.0

#ifndef I2CDEV_H
#define I2CDEV_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c_drv.h"

// Device type alias
typedef I2cDrv I2C_Dev;

// Bus aliases
#define I2C3_DEV (&sensorsBus)

// Initialize I2C device
int i2cdevInit(I2C_Dev *dev);

// Read bytes from an I2C peripheral
bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data);

// Read bytes with 8-bit register address
bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint16_t len, uint8_t *data);

// Read bytes with 16-bit register address
bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                     uint16_t len, uint8_t *data);

// Read a single byte
bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data);

// Read a bit from a byte
bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                   uint8_t bitNum, uint8_t *data);

// Read bits from a byte
bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data);

// Write bytes to an I2C peripheral
bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint16_t len,
                 const uint8_t *data);

// Write bytes with 8-bit register address
bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint16_t len, const uint8_t *data);

// Write bytes with 16-bit register address
bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                      uint16_t len, const uint8_t *data);

// Write a single byte
bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data);

// Write a bit in a byte
bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data);

// Write bits in a byte
bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data);

#endif // I2CDEV_H

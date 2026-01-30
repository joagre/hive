// I2C Device Layer - Bare-metal adaptation of Bitcraze crazyflie-firmware
//
// Based on crazyflie-firmware/src/drivers/src/i2cdev.c
// High-level I2C functions for reading/writing registers.
//
// Original copyright:
// Copyright (C) 2011-2012 Bitcraze AB
// GPL-3.0

#include <stdint.h>
#include <stdbool.h>
#include "i2cdev.h"
#include "i2c_drv.h"

int i2cdevInit(I2C_Dev *dev) {
    i2cdrvInit(dev);
    return true;
}

bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data) {
    return i2cdevReadReg8(dev, devAddress, memAddress, 1, data);
}

bool i2cdevReadBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                   uint8_t bitNum, uint8_t *data) {
    uint8_t byte;
    bool status;

    status = i2cdevReadReg8(dev, devAddress, memAddress, 1, &byte);
    *data = byte & (1 << bitNum);

    return status;
}

bool i2cdevReadBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data) {
    bool status;
    uint8_t byte;

    if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        byte &= mask;
        byte >>= (bitStart - length + 1);
        *data = byte;
    }
    return status;
}

bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data) {
    I2cMessage message;

    i2cdrvCreateMessage(&message, devAddress, i2cRead, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint16_t len, uint8_t *data) {
    I2cMessage message;

    i2cdrvCreateMessageIntAddr(&message, devAddress, false, memAddress, i2cRead,
                               len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                     uint16_t len, uint8_t *data) {
    I2cMessage message;

    i2cdrvCreateMessageIntAddr(&message, devAddress, true, memAddress, i2cRead,
                               len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data) {
    return i2cdevWriteReg8(dev, devAddress, memAddress, 1, &data);
}

bool i2cdevWriteBit(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data) {
    uint8_t byte;
    i2cdevReadByte(dev, devAddress, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data) {
    bool status;
    uint8_t byte;

    if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask;                     // zero all non-important bits in data
        byte &= ~(mask); // zero all important bits in existing byte
        byte |= data;    // combine data with existing byte
        status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
    }

    return status;
}

bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint16_t len,
                 const uint8_t *data) {
    I2cMessage message;

    i2cdrvCreateMessage(&message, devAddress, i2cWrite, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
                     uint16_t len, const uint8_t *data) {
    I2cMessage message;

    i2cdrvCreateMessageIntAddr(&message, devAddress, false, memAddress,
                               i2cWrite, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
                      uint16_t len, const uint8_t *data) {
    I2cMessage message;

    i2cdrvCreateMessageIntAddr(&message, devAddress, true, memAddress, i2cWrite,
                               len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

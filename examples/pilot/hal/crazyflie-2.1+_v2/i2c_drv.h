// I2C Driver Header - Bare-metal adaptation of Bitcraze crazyflie-firmware
//
// Based on crazyflie-firmware/src/drivers/interface/i2c_drv.h
// FreeRTOS dependencies replaced with bare-metal primitives.
//
// Original copyright:
// Copyright (c) 2014, Bitcraze AB, All rights reserved.
// LGPL-3.0

#ifndef I2C_DRV_H
#define I2C_DRV_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_misc.h"

#define I2C_NO_INTERNAL_ADDRESS 0xFFFF
#define I2C_MAX_RETRIES 2
#define I2C_MESSAGE_TIMEOUT_MS 1000

typedef enum { i2cAck, i2cNack } I2cStatus;

typedef enum { i2cWrite, i2cRead } I2cDirection;

// Structure used to capture the I2C message details
typedef struct _I2cMessage {
    uint32_t messageLength;   // How many bytes to send/receive
    uint8_t slaveAddress;     // The device address on the I2C bus
    uint8_t nbrOfRetries;     // Number of retries remaining
    I2cDirection direction;   // Direction of message
    I2cStatus status;         // i2c status
    bool isInternal16bit;     // Is internal address 16 bit
    uint16_t internalAddress; // Internal address of device
    uint8_t *buffer;          // Data buffer pointer
} I2cMessage;

// I2C bus definition (static configuration)
typedef struct {
    I2C_TypeDef *i2cPort;
    uint32_t i2cPerif;
    uint32_t i2cEVIRQn;
    uint32_t i2cERIRQn;
    uint32_t i2cClockSpeed;
    uint32_t gpioSCLPerif;
    GPIO_TypeDef *gpioSCLPort;
    uint32_t gpioSCLPin;
    uint32_t gpioSCLPinSource;
    uint32_t gpioSDAPerif;
    GPIO_TypeDef *gpioSDAPort;
    uint32_t gpioSDAPin;
    uint32_t gpioSDAPinSource;
    uint32_t gpioAF;
    uint32_t dmaPerif;
    uint32_t dmaChannel;
    DMA_Stream_TypeDef *dmaRxStream;
    uint32_t dmaRxIRQ;
    uint32_t dmaRxTCFlag;
    uint32_t dmaRxTEFlag;
} I2cDef;

// I2C driver state (bare-metal version)
typedef struct {
    const I2cDef *def;              // Definition of the i2c
    I2cMessage txMessage;           // The I2C send message
    uint32_t messageIndex;          // Index of bytes sent/received
    uint32_t nbrOfretries;          // Retries done
    volatile bool transferComplete; // Set by ISR when transfer done
    volatile bool transferError;    // Set by ISR on error
    DMA_InitTypeDef DMAStruct;      // DMA configuration structure
} I2cDrv;

// Definitions of i2c busses found in c file.
extern I2cDrv sensorsBus;

// Initialize i2c peripheral
void i2cdrvInit(I2cDrv *i2c);

// Send or receive a message over the I2C bus (blocking)
// Returns true if successful, false otherwise.
bool i2cdrvMessageTransfer(I2cDrv *i2c, I2cMessage *message);

// Create a message to transfer
void i2cdrvCreateMessage(I2cMessage *message, uint8_t slaveAddress,
                         I2cDirection direction, uint32_t length,
                         const uint8_t *buffer);

// Create a message with internal register address
void i2cdrvCreateMessageIntAddr(I2cMessage *message, uint8_t slaveAddress,
                                bool IsInternal16, uint16_t intAddress,
                                I2cDirection direction, uint32_t length,
                                const uint8_t *buffer);

// ISR handlers (called from stm32f4xx_it.c)
void i2cdrvEventIsrHandler(I2cDrv *i2c);
void i2cdrvErrorIsrHandler(I2cDrv *i2c);
void i2cdrvDmaIsrHandler(I2cDrv *i2c);

#endif // I2C_DRV_H

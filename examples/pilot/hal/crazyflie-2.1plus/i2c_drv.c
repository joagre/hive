// I2C Driver - Bare-metal adaptation of Bitcraze crazyflie-firmware
//
// Based on crazyflie-firmware/src/drivers/src/i2c_drv.c
// FreeRTOS dependencies replaced with bare-metal polling.
//
// Original copyright:
// Copyright (c) 2014, Bitcraze AB, All rights reserved.
// LGPL-3.0
//
// Key features preserved from Bitcraze:
// - DMA for I2C reads
// - Interrupt-driven state machine
// - Automatic retry on NACK (up to 2 retries) in error ISR
// - Bus unlock sequence for stuck slaves
// - CR1 workaround for STM32F405 instant START+STOP bug

#include <string.h>
#include "i2c_drv.h"

// Debug tracing - enable to diagnose DMA issues
#define I2C_DEBUG_TRACE 0

#if I2C_DEBUG_TRACE
#include "debug_swo.h"
#define I2C_TRACE(fmt, ...) debug_swo_printf(fmt, ##__VA_ARGS__)
#else
#define I2C_TRACE(fmt, ...) ((void)0)
#endif

// NVIC priority for I2C interrupts (lower = higher priority)
#define NVIC_I2C_PRI 5

// Internal DMA buffer in regular RAM (not CCM)
// CCM (0x10000000) is not accessible by DMA - must use main RAM (0x20000000)
#define I2C_DMA_BUFFER_SIZE 64
static uint8_t i2c_dma_buffer[I2C_DMA_BUFFER_SIZE]
    __attribute__((section(".data")));

// I2C clock speed
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED 400000

// Misc constants
#define I2C_SLAVE_ADDRESS7 0x30

// Clock timing for bus unlock
#define I2CDEV_CLK_TS 10 // microseconds

// External timing functions (provided by platform.c)
extern uint32_t platform_get_time_us(void);
extern uint32_t platform_get_time_ms(void);

// Simple microsecond delay
static void sleepus(uint32_t us) {
    uint32_t start = platform_get_time_us();
    while ((platform_get_time_us() - start) < us) {
        // busy wait
    }
}

// Wait for GPIO pin to go high with timeout
static void gpioWaitForHigh(GPIO_TypeDef *gpio, uint16_t pin,
                            uint16_t timeout_us) {
    uint32_t start = platform_get_time_us();
    while (GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET &&
           (platform_get_time_us() - start) <= timeout_us) {
        // busy wait
    }
}

// Sensor bus definition (I2C3: PA8=SCL, PC9=SDA)
static const I2cDef sensorBusDef = {
    .i2cPort = I2C3,
    .i2cPerif = RCC_APB1Periph_I2C3,
    .i2cEVIRQn = I2C3_EV_IRQn,
    .i2cERIRQn = I2C3_ER_IRQn,
    .i2cClockSpeed = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioSCLPerif = RCC_AHB1Periph_GPIOA,
    .gpioSCLPort = GPIOA,
    .gpioSCLPin = GPIO_Pin_8,
    .gpioSCLPinSource = GPIO_PinSource8,
    .gpioSDAPerif = RCC_AHB1Periph_GPIOC,
    .gpioSDAPort = GPIOC,
    .gpioSDAPin = GPIO_Pin_9,
    .gpioSDAPinSource = GPIO_PinSource9,
    .gpioAF = GPIO_AF_I2C3,
    .dmaPerif = RCC_AHB1Periph_DMA1,
    .dmaChannel = DMA_Channel_3,
    .dmaRxStream = DMA1_Stream2,
    .dmaRxIRQ = DMA1_Stream2_IRQn,
    .dmaRxTCFlag = DMA_FLAG_TCIF2,
    .dmaRxTEFlag = DMA_FLAG_TEIF2,
};

I2cDrv sensorsBus = {
    .def = &sensorBusDef,
};

// Forward declarations
static void i2cdrvInitBus(I2cDrv *i2c);
static void i2cdrvDmaSetupBus(I2cDrv *i2c);
static void i2cdrvStartTransfer(I2cDrv *i2c);
static void i2cdrvTryToRestartBus(I2cDrv *i2c);
static void i2cdrvdevUnlockBus(GPIO_TypeDef *portSCL, GPIO_TypeDef *portSDA,
                               uint16_t pinSCL, uint16_t pinSDA);
static void i2cdrvClearDMA(I2cDrv *i2c);
static void i2cTryNextMessage(I2cDrv *i2c);
static void i2cNotifyClient(I2cDrv *i2c);

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

void i2cdrvInit(I2cDrv *i2c) {
    i2cdrvInitBus(i2c);
}

void i2cdrvCreateMessage(I2cMessage *message, uint8_t slaveAddress,
                         I2cDirection direction, uint32_t length,
                         const uint8_t *buffer) {
    message->slaveAddress = slaveAddress;
    message->direction = direction;
    message->isInternal16bit = false;
    message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
    message->messageLength = length;
    message->status = i2cAck;
    message->buffer = (uint8_t *)buffer;
    message->nbrOfRetries = I2C_MAX_RETRIES;
}

void i2cdrvCreateMessageIntAddr(I2cMessage *message, uint8_t slaveAddress,
                                bool IsInternal16, uint16_t intAddress,
                                I2cDirection direction, uint32_t length,
                                const uint8_t *buffer) {
    message->slaveAddress = slaveAddress;
    message->direction = direction;
    message->isInternal16bit = IsInternal16;
    message->internalAddress = intAddress;
    message->messageLength = length;
    message->status = i2cAck;
    message->buffer = (uint8_t *)buffer;
    message->nbrOfRetries = I2C_MAX_RETRIES;
}

bool i2cdrvMessageTransfer(I2cDrv *i2c, I2cMessage *message) {
    bool status = false;

    // Copy message to driver state
    memcpy((char *)&i2c->txMessage, (char *)message, sizeof(I2cMessage));

    // Reset transfer flags
    i2c->transferComplete = false;
    i2c->transferError = false;

    // Start the ISR-driven transfer
    i2cdrvStartTransfer(i2c);

    // Wait for transfer to complete (polling with timeout)
    uint32_t start = platform_get_time_ms();
    while (!i2c->transferComplete) {
        if ((platform_get_time_ms() - start) > I2C_MESSAGE_TIMEOUT_MS) {
            // Timeout - try to recover bus
            i2cdrvClearDMA(i2c);
            i2cdrvTryToRestartBus(i2c);
            return false;
        }
        // Could add WFI here to save power, but keep it simple
    }

    if (i2c->txMessage.status == i2cAck) {
        status = true;
    }

    return status;
}

// ----------------------------------------------------------------------------
// Private Functions
// ----------------------------------------------------------------------------

static void i2cdrvStartTransfer(I2cDrv *i2c) {
    I2C_TRACE(
        "[I2C] StartTransfer: dir=%d intAddr=0x%04X len=%d buf=0x%08X\n",
        i2c->txMessage.direction, (unsigned)i2c->txMessage.internalAddress,
        (int)i2c->txMessage.messageLength, (unsigned)i2c->txMessage.buffer);

    if (i2c->txMessage.direction == i2cRead) {
        // Prepare DMA configuration but don't enable yet
        // DMA will be enabled in ADDR handler when ready to receive
        DMA_Cmd(i2c->def->dmaRxStream, DISABLE);
        // Clear all DMA flags for stream 2 (TC, HT, TE, DME, FE)
        DMA_ClearFlag(i2c->def->dmaRxStream,
                      DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_TEIF2 |
                          DMA_FLAG_DMEIF2 | DMA_FLAG_FEIF2);

        i2c->DMAStruct.DMA_BufferSize = i2c->txMessage.messageLength;
        // Use internal DMA buffer in regular RAM (user buffer may be in CCM)
        i2c->DMAStruct.DMA_Memory0BaseAddr = (uint32_t)i2c_dma_buffer;
        I2C_TRACE("[I2C] DMA prepared: size=%d mem=0x%08X periph=0x%08X\n",
                  (int)i2c->DMAStruct.DMA_BufferSize,
                  (unsigned)i2c->DMAStruct.DMA_Memory0BaseAddr,
                  (unsigned)i2c->DMAStruct.DMA_PeripheralBaseAddr);
        // Initialize DMA but DON'T enable - wait for ADDR event
        DMA_Init(i2c->def->dmaRxStream, &i2c->DMAStruct);
    }

    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT, ENABLE);

    // Workaround for STM32F405 I2C bug:
    // Setting CR1 in sequence with I2C_AcknowledgeConfig() and then
    // I2C_GenerateSTART() sometimes creates an instant start->stop condition.
    // Setting all bits at once avoids this.
    i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
}

static void i2cTryNextMessage(I2cDrv *i2c) {
    i2c->def->i2cPort->CR1 = (I2C_CR1_STOP | I2C_CR1_PE);
    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
}

static void i2cNotifyClient(I2cDrv *i2c) {
    // Bare-metal: just set the flag
    I2C_TRACE("[I2C] NotifyClient: status=%d\n", i2c->txMessage.status);
    i2c->transferComplete = true;
}

static void i2cdrvTryToRestartBus(I2cDrv *i2c) {
    I2C_InitTypeDef I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(i2c->def->gpioSDAPerif, ENABLE);
    RCC_AHB1PeriphClockCmd(i2c->def->gpioSCLPerif, ENABLE);
    // Enable I2C clock
    RCC_APB1PeriphClockCmd(i2c->def->i2cPerif, ENABLE);

    // Configure pins as GPIO output open-drain to unlock bus
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin;
    GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSDAPin;
    GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

    // Unlock the bus if needed
    i2cdrvdevUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort,
                       i2c->def->gpioSCLPin, i2c->def->gpioSDAPin);

    // Configure pins for I2C alternate function
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin;
    GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSDAPin;
    GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

    // Map GPIOs to alternate functions
    GPIO_PinAFConfig(i2c->def->gpioSCLPort, i2c->def->gpioSCLPinSource,
                     i2c->def->gpioAF);
    GPIO_PinAFConfig(i2c->def->gpioSDAPort, i2c->def->gpioSDAPinSource,
                     i2c->def->gpioAF);

    // I2C configuration
    I2C_DeInit(i2c->def->i2cPort);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = i2c->def->i2cClockSpeed;
    I2C_Init(i2c->def->i2cPort, &I2C_InitStructure);

    // Enable I2C error interrupts
    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_ERR, ENABLE);

    // Configure NVIC for I2C event interrupt
    NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cEVIRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure NVIC for I2C error interrupt
    NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cERIRQn;
    NVIC_Init(&NVIC_InitStructure);

    // Setup DMA
    i2cdrvDmaSetupBus(i2c);
}

static void i2cdrvDmaSetupBus(I2cDrv *i2c) {
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(i2c->def->dmaPerif, ENABLE);

    // RX DMA Channel Config
    i2c->DMAStruct.DMA_Channel = i2c->def->dmaChannel;
    i2c->DMAStruct.DMA_PeripheralBaseAddr = (uint32_t)&i2c->def->i2cPort->DR;
    i2c->DMAStruct.DMA_Memory0BaseAddr = 0;
    i2c->DMAStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    i2c->DMAStruct.DMA_BufferSize = 0;
    i2c->DMAStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    i2c->DMAStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    i2c->DMAStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    i2c->DMAStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    i2c->DMAStruct.DMA_Mode = DMA_Mode_Normal;
    i2c->DMAStruct.DMA_Priority = DMA_Priority_High;
    i2c->DMAStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    i2c->DMAStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    i2c->DMAStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    i2c->DMAStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    // Configure NVIC for DMA
    NVIC_InitStructure.NVIC_IRQChannel = i2c->def->dmaRxIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void i2cdrvInitBus(I2cDrv *i2c) {
    i2c->transferComplete = false;
    i2c->transferError = false;
    i2cdrvTryToRestartBus(i2c);
}

static void i2cdrvdevUnlockBus(GPIO_TypeDef *portSCL, GPIO_TypeDef *portSDA,
                               uint16_t pinSCL, uint16_t pinSDA) {
    GPIO_SetBits(portSDA, pinSDA);

    // Check SDA line to determine if slave is asserting bus and clock out if so
    while (GPIO_ReadInputDataBit(portSDA, pinSDA) == Bit_RESET) {
        // Set clock high
        GPIO_SetBits(portSCL, pinSCL);
        // Wait for any clock stretching to finish
        gpioWaitForHigh(portSCL, pinSCL, 10 * 1000);
        sleepus(I2CDEV_CLK_TS);

        // Generate a clock cycle
        GPIO_ResetBits(portSCL, pinSCL);
        sleepus(I2CDEV_CLK_TS);
        GPIO_SetBits(portSCL, pinSCL);
        sleepus(I2CDEV_CLK_TS);
    }

    // Generate a start then stop condition
    GPIO_SetBits(portSCL, pinSCL);
    sleepus(I2CDEV_CLK_TS);
    GPIO_ResetBits(portSDA, pinSDA);
    sleepus(I2CDEV_CLK_TS);
    GPIO_ResetBits(portSCL, pinSCL);
    sleepus(I2CDEV_CLK_TS);

    // Set data and clock high and wait for any clock stretching to finish
    GPIO_SetBits(portSDA, pinSDA);
    GPIO_SetBits(portSCL, pinSCL);
    gpioWaitForHigh(portSCL, pinSCL, 10 * 1000);
    // Wait for data to be high
    gpioWaitForHigh(portSDA, pinSDA, 10 * 1000);
}

static void i2cdrvClearDMA(I2cDrv *i2c) {
    DMA_Cmd(i2c->def->dmaRxStream, DISABLE);
    // Clear all DMA flags for stream 2 (TC, HT, TE, DME, FE)
    DMA_ClearFlag(i2c->def->dmaRxStream, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2 |
                                             DMA_FLAG_TEIF2 | DMA_FLAG_DMEIF2 |
                                             DMA_FLAG_FEIF2);
    I2C_DMACmd(i2c->def->i2cPort, DISABLE);
    I2C_DMALastTransferCmd(i2c->def->i2cPort, DISABLE);
    DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, DISABLE);
}

// ----------------------------------------------------------------------------
// ISR Handlers
// ----------------------------------------------------------------------------

void i2cdrvEventIsrHandler(I2cDrv *i2c) {
    uint16_t SR1;
    uint16_t SR2;

    // Read the status register first
    SR1 = i2c->def->i2cPort->SR1;

    I2C_TRACE("[I2C-ISR] EV SR1=0x%04X\n", SR1);

    // Start bit event
    if (SR1 & I2C_SR1_SB) {
        i2c->messageIndex = 0;
        I2C_TRACE("[I2C-ISR] SB: dir=%d intAddr=0x%04X\n",
                  i2c->txMessage.direction,
                  (unsigned)i2c->txMessage.internalAddress);

        if (i2c->txMessage.direction == i2cWrite ||
            i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS) {
            I2C_TRACE("[I2C-ISR] SB: send addr 0x%02X as TX\n",
                      i2c->txMessage.slaveAddress);
            I2C_Send7bitAddress(i2c->def->i2cPort,
                                i2c->txMessage.slaveAddress << 1,
                                I2C_Direction_Transmitter);
        } else {
            I2C_TRACE("[I2C-ISR] SB: send addr 0x%02X as RX\n",
                      i2c->txMessage.slaveAddress);
            I2C_AcknowledgeConfig(i2c->def->i2cPort, ENABLE);
            I2C_Send7bitAddress(i2c->def->i2cPort,
                                i2c->txMessage.slaveAddress << 1,
                                I2C_Direction_Receiver);
        }
    }
    // Address event
    else if (SR1 & I2C_SR1_ADDR) {
        I2C_TRACE("[I2C-ISR] ADDR: dir=%d intAddr=0x%04X\n",
                  i2c->txMessage.direction,
                  (unsigned)i2c->txMessage.internalAddress);

        if (i2c->txMessage.direction == i2cWrite ||
            i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS) {
            I2C_TRACE("[I2C-ISR] ADDR: TX mode, sending intAddr\n");
            SR2 = i2c->def->i2cPort->SR2; // clear ADDR
            (void)SR2;

            // In write mode transmit is always empty so can send up to two bytes
            if (i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS) {
                if (i2c->txMessage.isInternal16bit) {
                    I2C_TRACE("[I2C-ISR] ADDR: sending 16-bit reg 0x%04X\n",
                              (unsigned)i2c->txMessage.internalAddress);
                    I2C_SendData(i2c->def->i2cPort,
                                 (i2c->txMessage.internalAddress & 0xFF00) >>
                                     8);
                    I2C_SendData(i2c->def->i2cPort,
                                 (i2c->txMessage.internalAddress & 0x00FF));
                } else {
                    I2C_TRACE(
                        "[I2C-ISR] ADDR: sending 8-bit reg 0x%02X\n",
                        (unsigned)(i2c->txMessage.internalAddress & 0xFF));
                    I2C_SendData(i2c->def->i2cPort,
                                 (i2c->txMessage.internalAddress & 0x00FF));
                }
                i2c->txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
            }
            I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF,
                         ENABLE); // allow us to have an EV7
        } else {
            // Reading, start DMA transfer
            I2C_TRACE("[I2C-ISR] ADDR: RX mode, enabling DMA, len=%d\n",
                      (int)i2c->txMessage.messageLength);
            if (i2c->txMessage.messageLength == 1) {
                I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);
                I2C_DMALastTransferCmd(i2c->def->i2cPort,
                                       ENABLE); // Required for 1-byte DMA
            } else {
                I2C_DMALastTransferCmd(i2c->def->i2cPort,
                                       ENABLE); // NACK after last byte
            }
            // Disable buffer I2C interrupts
            I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
            // Enable the Transfer Complete interrupt
            DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, ENABLE);
            I2C_DMACmd(i2c->def->i2cPort, ENABLE);  // Enable before ADDR clear
            DMA_Cmd(i2c->def->dmaRxStream, ENABLE); // Enable DMA

            I2C_TRACE("[I2C-ISR] ADDR: DMA NDTR=%d CR=0x%08X I2C_CR2=0x%04X\n",
                      (int)i2c->def->dmaRxStream->NDTR,
                      (unsigned)i2c->def->dmaRxStream->CR,
                      (unsigned)i2c->def->i2cPort->CR2);

            __DMB(); // Make sure instructions are in correct order
            SR2 = i2c->def->i2cPort->SR2; // clear ADDR by reading SR2
            I2C_TRACE("[I2C-ISR] ADDR cleared (SR2=0x%04X)\n", (unsigned)SR2);
            (void)SR2;
            // DO NOT read DR here - it would interfere with DMA!
        }
    }
    // Byte transfer finished
    else if (SR1 & I2C_SR1_BTF) {
        SR2 = i2c->def->i2cPort->SR2;
        I2C_TRACE("[I2C-ISR] BTF: SR2=0x%04X TRA=%d\n", SR2,
                  (SR2 & I2C_SR2_TRA) ? 1 : 0);
        if (SR2 & I2C_SR2_TRA) { // In write mode?
            if (i2c->txMessage.direction == i2cRead) {
                // Internal address written, switch to read
                I2C_TRACE("[I2C-ISR] BTF: switching to read, generate START\n");
                i2c->def->i2cPort->CR1 =
                    (I2C_CR1_START | I2C_CR1_PE); // Generate start
            } else {
                I2C_TRACE("[I2C-ISR] BTF: write complete, notify\n");
                i2cNotifyClient(i2c);
                i2cTryNextMessage(i2c);
            }
        } else {
            // Reading - shouldn't happen since we use DMA for reading
            I2C_TRACE("[I2C-ISR] BTF: RX fallback (no DMA?)\n");
            i2c->txMessage.buffer[i2c->messageIndex++] =
                I2C_ReceiveData(i2c->def->i2cPort);
            if (i2c->messageIndex == i2c->txMessage.messageLength) {
                i2cNotifyClient(i2c);
                i2cTryNextMessage(i2c);
            }
        }
        // A second BTF interrupt might occur if we don't wait for START to clear
        while (i2c->def->i2cPort->CR1 & I2C_CR1_START) {
            ;
        }
    }
    // Byte received
    else if (SR1 & I2C_SR1_RXNE) {
        // Should not happen when we use DMA for reception
        i2c->txMessage.buffer[i2c->messageIndex++] =
            I2C_ReceiveData(i2c->def->i2cPort);
        if (i2c->messageIndex == i2c->txMessage.messageLength) {
            I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF,
                         DISABLE); // disable RXE to get BTF
        }
    }
    // Byte ready to be transmitted
    else if (SR1 & I2C_SR1_TXE) {
        I2C_TRACE("[I2C-ISR] TXE: dir=%d\n", i2c->txMessage.direction);
        if (i2c->txMessage.direction == i2cRead) {
            // Disable TXE to flush and get BTF to switch to read
            I2C_TRACE("[I2C-ISR] TXE: disable BUF for BTF\n");
            I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
        } else {
            I2C_SendData(i2c->def->i2cPort,
                         i2c->txMessage.buffer[i2c->messageIndex++]);
            if (i2c->messageIndex == i2c->txMessage.messageLength) {
                // Disable TXE to allow the buffer to flush and get BTF
                I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
                __DMB();
            }
        }
    }
}

void i2cdrvErrorIsrHandler(I2cDrv *i2c) {
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_AF)) {
        // Acknowledge failure - retry by generating start
        if (i2c->txMessage.nbrOfRetries-- > 0) {
            // Retry by generating start
            i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
        } else {
            // Failed so notify client and try next message if any
            i2c->txMessage.status = i2cNack;
            i2cNotifyClient(i2c);
            i2cTryNextMessage(i2c);
        }
        I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_AF);
    }
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_BERR)) {
        I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_BERR);
    }
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_OVR)) {
        I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_OVR);
    }
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_ARLO)) {
        I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_ARLO);
    }
}

void i2cdrvDmaIsrHandler(I2cDrv *i2c) {
    bool has_error = false;

#if I2C_DEBUG_TRACE
    uint32_t lisr = DMA1->LISR;
    uint32_t hisr = DMA1->HISR;
    I2C_TRACE("[I2C-DMA] ISR: LISR=0x%08X HISR=0x%08X NDTR=%d\n",
              (unsigned)lisr, (unsigned)hisr, (int)i2c->def->dmaRxStream->NDTR);
    I2C_TRACE("[I2C-DMA] M0AR=0x%08X PAR=0x%08X CR=0x%08X\n",
              (unsigned)i2c->def->dmaRxStream->M0AR,
              (unsigned)i2c->def->dmaRxStream->PAR,
              (unsigned)i2c->def->dmaRxStream->CR);
#endif

    if (DMA_GetFlagStatus(i2c->def->dmaRxStream, i2c->def->dmaRxTEFlag)) {
        // Transfer error - check what went wrong
        I2C_TRACE(
            "[I2C-DMA] Transfer ERROR - FEIF=%d DMEIF=%d TEIF=%d\n",
            DMA_GetFlagStatus(i2c->def->dmaRxStream, DMA_FLAG_FEIF2) ? 1 : 0,
            DMA_GetFlagStatus(i2c->def->dmaRxStream, DMA_FLAG_DMEIF2) ? 1 : 0,
            DMA_GetFlagStatus(i2c->def->dmaRxStream, DMA_FLAG_TEIF2) ? 1 : 0);
        DMA_ClearITPendingBit(i2c->def->dmaRxStream, i2c->def->dmaRxTEFlag);
        i2c->txMessage.status = i2cNack;
        has_error = true;
    }
    if (DMA_GetFlagStatus(i2c->def->dmaRxStream, i2c->def->dmaRxTCFlag)) {
        // Transfer complete - copy from internal DMA buffer to user buffer
        I2C_TRACE("[I2C-DMA] Transfer COMPLETE, dma_buf[0]=0x%02X\n",
                  i2c_dma_buffer[0]);
        if (!has_error) {
            // Copy data from internal DMA buffer to user's buffer
            if (i2c->txMessage.buffer &&
                i2c->txMessage.messageLength <= I2C_DMA_BUFFER_SIZE) {
                memcpy(i2c->txMessage.buffer, i2c_dma_buffer,
                       i2c->txMessage.messageLength);
            }
            i2c->txMessage.status = i2cAck;
        }
    }
    i2cdrvClearDMA(i2c);
    i2cNotifyClient(i2c);
    i2cTryNextMessage(i2c);
}

// ----------------------------------------------------------------------------
// IRQ Handlers (called from vector table)
// ----------------------------------------------------------------------------

void __attribute__((used)) I2C3_EV_IRQHandler(void) {
    i2cdrvEventIsrHandler(&sensorsBus);
}

void __attribute__((used)) I2C3_ER_IRQHandler(void) {
    i2cdrvErrorIsrHandler(&sensorsBus);
}

void __attribute__((used)) DMA1_Stream2_IRQHandler(void) {
    i2cdrvDmaIsrHandler(&sensorsBus);
}

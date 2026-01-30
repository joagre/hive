// Crazyflie 2.1+ Platform Layer Implementation
//
// Implements the platform interface using direct STM32F405 peripheral access
// and vendor sensor drivers (Bosch BMI08x, Bosch BMP3, ST VL53L1x, Bitcraze PMW3901).

#include "platform.h"
#include "hal_config.h"
#include "stm32f4xx.h"
#include "motors.h"
#include "debug_swo.h"

// Vendor drivers
#include "vendor/bosch/bmi08x/bmi08x.h"
#include "vendor/bosch/bmp3/bmp3.h"
#include "vendor/st/vl53l1x/vl53l1x_uld.h"
#include "vendor/bitcraze/pmw3901/pmw3901.h"

#include <stdbool.h>
#include <stdarg.h>
#include <math.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Calibration parameters (matching Bitcraze firmware)
#define CALIBRATION_SAMPLES 512        // Gyro calibration samples
#define GYRO_VARIANCE_THRESHOLD 100.0f // Max variance for stable gyro
#define GYRO_CALIBRATION_TIMEOUT_MS \
    1000                            // Min time between calibration attempts
#define BARO_CALIBRATION_SAMPLES 50 // Barometer calibration samples
#define ACCEL_SCALE_SAMPLES 200     // Accelerometer scale calibration samples

// Startup delay (Bitcraze waits 1000ms for sensor power stabilization)
#define SENSOR_STARTUP_DELAY_MS 1000

// Conversion constants
#define GRAVITY 9.80665f
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// GPIO Pins
#define LED_PIN (1U << 2) // PD2 blue LED on Crazyflie
#define LED_PORT GPIOD

// I2C addresses (I2C3 - on-board sensors)
#define BMI08_ACCEL_I2C_ADDR 0x18 // BMI088 accelerometer
#define BMI08_GYRO_I2C_ADDR 0x69  // BMI088 gyroscope (Bitcraze uses 0x69)
#define BMP3_I2C_ADDR 0x77        // BMP388 barometer

// I2C addresses (I2C1 - expansion connector / Flow deck)
// Note: VL53L1x default is 0x29, but Bitcraze firmware may reassign it
// We scan multiple addresses to find it
#define VL53L1X_I2C_ADDR_DEFAULT 0x29
#define VL53L1X_MODEL_ID 0xEACC

// I2C pins (I2C3) - used by BMI088, BMP388, VL53L1x
#define I2C3_SCL_PIN 8 // PA8
#define I2C3_SDA_PIN 9 // PC9

// SPI1 pins - used by PMW3901 on Flow deck
#define SPI1_SCK_PIN 5  // PA5
#define SPI1_MISO_PIN 6 // PA6
#define SPI1_MOSI_PIN 7 // PA7

// SPI chip select for PMW3901 on Flow deck (DECK_GPIO_IO3 = PB4)
#define FLOW_SPI_CS_PIN 4 // PB4 (IO3 on deck connector)

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;
static bool s_calibrated = false;
static bool s_armed = false;
static bool s_flow_deck_present = false;

// Gyro bias (rad/s) - determined during calibration
static float s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// Accelerometer scale factor - determined during calibration (1.0 = no correction)
static float s_accel_scale = 1.0f;

// Barometer reference pressure (Pa)
static float s_ref_pressure = 0.0f;

// VL53L1x height offset (mm) - ground clearance when on flat surface
static uint16_t s_height_offset_mm = 0;

// Flow deck integration state (provides pseudo-GPS position)
static float s_integrated_x = 0.0f; // Integrated X position (m, world frame)
static float s_integrated_y = 0.0f; // Integrated Y position (m, world frame)
static float s_prev_yaw = 0.0f;     // Previous yaw for body-to-world rotation
static uint32_t s_prev_flow_time_us = 0;

// System tick counter
static volatile uint32_t s_sys_tick_ms = 0;

// Vendor driver device handles
static struct bmi08_dev s_bmi08_dev;
static struct bmp3_dev s_bmp3_dev;
static vl53l1x_dev_t s_vl53l1x_dev;
static pmw3901_dev_t s_pmw3901_dev;

// I2C address storage for Bosch driver callbacks
static uint8_t s_bmi08_accel_addr = BMI08_ACCEL_I2C_ADDR;
static uint8_t s_bmi08_gyro_addr = BMI08_GYRO_I2C_ADDR;
static uint8_t s_bmp3_addr = BMP3_I2C_ADDR;

// ----------------------------------------------------------------------------
// SysTick Handler
// ----------------------------------------------------------------------------

void SysTick_Handler(void) {
    s_sys_tick_ms++;
}

// ----------------------------------------------------------------------------
// Low-Level Platform Functions
// ----------------------------------------------------------------------------

// Note: System clock initialization moved to system_stm32f4xx.c (SystemInit)
// Called by startup code before main(), enabling early debug output.

static void systick_init(void) {
    // Configure SysTick for 1ms interrupts
    SysTick_Config(SystemCoreClock / 1000);
}

static void gpio_init(void) {
    // Enable GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

    // LED (PD2) as output
    LED_PORT->MODER &= ~GPIO_MODER_MODER2;
    LED_PORT->MODER |= GPIO_MODER_MODER2_0;      // Output mode
    LED_PORT->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2; // High speed
    LED_PORT->ODR &= ~LED_PIN;                   // LED off
}

// ----------------------------------------------------------------------------
// I2C3 Low-Level Interface
// ----------------------------------------------------------------------------

// I2C timeout in microseconds (10ms should be plenty for 400kHz)
#define I2C_TIMEOUT_US 10000

// I2C3 pins for bus recovery
#define I2C3_SCL_PORT GPIOA
#define I2C3_SCL_PIN 8
#define I2C3_SDA_PORT GPIOC
#define I2C3_SDA_PIN 9

// Wait for a condition with timeout, returns false on timeout
static bool i2c3_wait(volatile uint32_t *reg, uint32_t mask,
                      uint32_t expected) {
    uint32_t start = platform_get_time_us();
    while ((*reg & mask) != expected) {
        if ((platform_get_time_us() - start) > I2C_TIMEOUT_US) {
            return false;
        }
    }
    return true;
}

// Check for and clear I2C errors, returns true if error occurred
static bool i2c3_check_error(void) {
    uint32_t sr1 = I2C3->SR1;
    if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)) {
        // Clear error flags
        I2C3->SR1 =
            sr1 & ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR);
        return true;
    }
    return false;
}

// Bus recovery: toggle SCL to release stuck SDA
static void i2c3_bus_recovery(void) {
    // Disable I2C peripheral
    I2C3->CR1 &= ~I2C_CR1_PE;

    // Configure SCL as GPIO output, SDA as input
    I2C3_SCL_PORT->MODER &= ~(3U << (I2C3_SCL_PIN * 2));
    I2C3_SCL_PORT->MODER |= (1U << (I2C3_SCL_PIN * 2)); // Output
    I2C3_SCL_PORT->OTYPER |= (1U << I2C3_SCL_PIN);      // Open-drain

    I2C3_SDA_PORT->MODER &= ~(3U << (I2C3_SDA_PIN * 2)); // Input

    // Toggle SCL up to 9 times to release stuck slave
    for (int i = 0; i < 9; i++) {
        // Check if SDA is high (bus released)
        if (I2C3_SDA_PORT->IDR & (1U << I2C3_SDA_PIN)) {
            break;
        }
        // Toggle SCL
        I2C3_SCL_PORT->ODR &= ~(1U << I2C3_SCL_PIN); // SCL low
        for (volatile int d = 0; d < 100; d++)
            __NOP();
        I2C3_SCL_PORT->ODR |= (1U << I2C3_SCL_PIN); // SCL high
        for (volatile int d = 0; d < 100; d++)
            __NOP();
    }

    // Generate STOP condition manually: SDA low then high while SCL high
    I2C3_SDA_PORT->MODER |= (1U << (I2C3_SDA_PIN * 2)); // SDA output
    I2C3_SDA_PORT->OTYPER |= (1U << I2C3_SDA_PIN);      // Open-drain
    I2C3_SDA_PORT->ODR &= ~(1U << I2C3_SDA_PIN);        // SDA low
    for (volatile int d = 0; d < 100; d++)
        __NOP();
    I2C3_SDA_PORT->ODR |= (1U << I2C3_SDA_PIN); // SDA high (STOP)
    for (volatile int d = 0; d < 100; d++)
        __NOP();

    // Reconfigure pins for I2C alternate function
    I2C3_SCL_PORT->MODER &= ~(3U << (I2C3_SCL_PIN * 2));
    I2C3_SCL_PORT->MODER |= (2U << (I2C3_SCL_PIN * 2)); // AF mode
    I2C3_SDA_PORT->MODER &= ~(3U << (I2C3_SDA_PIN * 2));
    I2C3_SDA_PORT->MODER |= (2U << (I2C3_SDA_PIN * 2)); // AF mode

    // Re-enable I2C peripheral
    I2C3->CR1 |= I2C_CR1_PE;
}

// Reset I2C peripheral after error (full APB1 reset like bringup)
static void i2c3_reset(void) {
    // Disable I2C first
    I2C3->CR1 = 0;

    // Full APB1 peripheral reset
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
    for (volatile int i = 0; i < 10; i++)
        ; // Brief delay
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;

    // Reconfigure (must match i2c3_init settings)
    I2C3->CR2 = 42;              // FREQ = 42 MHz
    I2C3->CCR = 53 | I2C_CCR_FS; // CCR for ~400 kHz (matching bringup)
    I2C3->TRISE = 13;
    I2C3->CR1 = I2C_CR1_PE;
}

static void i2c3_init(void) {
    // Enable I2C3 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

    // Bus recovery: toggle SCL 9 times to release any stuck slave
    // Configure PA8 (SCL) as GPIO output first
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_0; // Output mode
    GPIOA->OTYPER |= GPIO_OTYPER_OT8;    // Open-drain
    GPIOA->BSRR = GPIO_BSRR_BS8;         // Start high
    for (int i = 0; i < 9; i++) {
        GPIOA->BSRR = GPIO_BSRR_BR8; // SCL low
        for (volatile int d = 0; d < 100; d++)
            __NOP();
        GPIOA->BSRR = GPIO_BSRR_BS8; // SCL high
        for (volatile int d = 0; d < 100; d++)
            __NOP();
    }

    // Reset I2C3 peripheral
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;

    // Configure PA8 (SCL) as AF4 open-drain with high speed
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_1;      // AF mode
    GPIOA->OTYPER |= GPIO_OTYPER_OT8;         // Open-drain
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8; // High speed
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR8;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0; // Pull-up
    GPIOA->AFR[1] &= ~(0xFU << 0);       // Clear AF bits
    GPIOA->AFR[1] |= (4U << 0);          // AF4 = I2C3

    // Configure PC9 (SDA) as AF4 open-drain with high speed
    GPIOC->MODER &= ~GPIO_MODER_MODER9;
    GPIOC->MODER |= GPIO_MODER_MODER9_1;      // AF mode
    GPIOC->OTYPER |= GPIO_OTYPER_OT9;         // Open-drain
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9; // High speed
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR9;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR9_0; // Pull-up
    GPIOC->AFR[1] &= ~(0xFU << 4);       // Clear AF bits
    GPIOC->AFR[1] |= (4U << 4);          // AF4 = I2C3

    // Configure I2C3: 400 kHz fast mode
    // APB1 = 42 MHz, CCR = 42MHz / (2 * 400kHz) = 52.5 -> 53 (matching bringup)
    I2C3->CR1 = 0;               // Disable before configuring
    I2C3->CR2 = 42;              // FREQ = 42 MHz
    I2C3->CCR = 53 | I2C_CCR_FS; // CCR for ~400 kHz, fast mode enabled
    I2C3->TRISE = 13;            // Maximum rise time
    I2C3->CR1 = I2C_CR1_PE;      // Enable I2C
}

static bool i2c3_write(uint8_t addr, uint8_t *data, uint16_t len) {
    // Generate START (no pre-checks - matches bringup)
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        I2C3->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (write)
    I2C3->DR = addr << 1;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR)) {
        I2C3->CR1 |= I2C_CR1_STOP;
        return false;
    }
    (void)I2C3->SR2; // Clear ADDR flag

    // Send data
    for (uint16_t i = 0; i < len; i++) {
        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_TXE, I2C_SR1_TXE)) {
            I2C3->CR1 |= I2C_CR1_STOP;
            return false;
        }
        I2C3->DR = data[i];
    }

    // Wait for transfer complete
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
        I2C3->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Generate STOP
    I2C3->CR1 |= I2C_CR1_STOP;

    // Wait for STOP to complete (BUSY flag cleared) - matches bringup
    i2c3_wait(&I2C3->SR2, I2C_SR2_BUSY, 0);

    return true;
}

static bool i2c3_read(uint8_t addr, uint8_t *data, uint16_t len) {
    if (len == 0)
        return false;

    // Clear any pending errors
    i2c3_check_error();

    // Wait for bus not busy
    if (!i2c3_wait(&I2C3->SR2, I2C_SR2_BUSY, 0)) {
        i2c3_bus_recovery();
        i2c3_reset();
        return false;
    }

    // Enable ACK
    I2C3->CR1 |= I2C_CR1_ACK;

    // Generate START
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        I2C3->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (read)
    I2C3->DR = (addr << 1) | 1;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR)) {
        if (i2c3_check_error()) {
            I2C3->CR1 |= I2C_CR1_STOP;
            return false; // NACK - device not responding
        }
        I2C3->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // N=1: Disable ACK before clearing ADDR, then set STOP
    if (len == 1) {
        I2C3->CR1 &= ~I2C_CR1_ACK;
        (void)I2C3->SR2; // Clear ADDR
        I2C3->CR1 |= I2C_CR1_STOP;

        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
            return false;
        }
        data[0] = I2C3->DR;
        return true;
    }

    // N=2: Use POS bit per STM32 reference manual (RM0090)
    if (len == 2) {
        I2C3->CR1 |= I2C_CR1_POS;
        (void)I2C3->SR2; // Clear ADDR
        I2C3->CR1 &= ~I2C_CR1_ACK;

        // Wait for BTF (both bytes received)
        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
            I2C3->CR1 &= ~I2C_CR1_POS;
            I2C3->CR1 |= I2C_CR1_STOP;
            return false;
        }

        // Set STOP before reading
        I2C3->CR1 |= I2C_CR1_STOP;
        data[0] = I2C3->DR;
        data[1] = I2C3->DR;

        I2C3->CR1 &= ~I2C_CR1_POS;
        return true;
    }

    // N>2: Standard sequence with BTF for last two bytes
    (void)I2C3->SR2; // Clear ADDR

    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            // Last byte: STOP already set after previous byte
            if (!i2c3_wait(&I2C3->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
                return false;
            }
            data[i] = I2C3->DR;
        } else if (i == len - 2) {
            // Second to last: wait for BTF, clear ACK, set STOP, read
            if (!i2c3_wait(&I2C3->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
                I2C3->CR1 |= I2C_CR1_STOP;
                return false;
            }
            I2C3->CR1 &= ~I2C_CR1_ACK;
            I2C3->CR1 |= I2C_CR1_STOP;
            data[i] = I2C3->DR;
        } else {
            // Normal byte
            if (!i2c3_wait(&I2C3->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
                I2C3->CR1 |= I2C_CR1_STOP;
                return false;
            }
            data[i] = I2C3->DR;
        }
    }

    return true;
}

// Combined write-then-read with repeated start (proper I2C register read)
static bool i2c3_write_read(uint8_t addr, const uint8_t *wdata, uint16_t wlen,
                            uint8_t *rdata, uint16_t rlen) {
    if (wlen == 0 || rlen == 0)
        return false;

    // Generate START (no pre-checks - matches bringup)
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_SB, I2C_SR1_SB))
        goto error;

    // Send address (write)
    I2C3->DR = addr << 1;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR))
        goto error;
    (void)I2C3->SR2; // Clear ADDR

    // Send write data
    for (uint16_t i = 0; i < wlen; i++) {
        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_TXE, I2C_SR1_TXE))
            goto error;
        I2C3->DR = wdata[i];
    }
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_BTF, I2C_SR1_BTF))
        goto error;

    // REPEATED START (not STOP)
    I2C3->CR1 |= I2C_CR1_ACK;
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_SB, I2C_SR1_SB))
        goto error;

    // Send address (read)
    I2C3->DR = (addr << 1) | 1;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR))
        goto error;

    // Handle read based on length (per RM0090)
    if (rlen == 1) {
        I2C3->CR1 &= ~I2C_CR1_ACK;
        (void)I2C3->SR2; // Clear ADDR
        I2C3->CR1 |= I2C_CR1_STOP;
        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE))
            goto error;
        rdata[0] = I2C3->DR;
    } else if (rlen == 2) {
        I2C3->CR1 |= I2C_CR1_POS;
        (void)I2C3->SR2; // Clear ADDR
        I2C3->CR1 &= ~I2C_CR1_ACK;
        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_BTF, I2C_SR1_BTF))
            goto error;
        I2C3->CR1 |= I2C_CR1_STOP;
        rdata[0] = I2C3->DR;
        rdata[1] = I2C3->DR;
        I2C3->CR1 &= ~I2C_CR1_POS;
    } else {
        // N > 2 (per RM0090 section 27.3.3)
        (void)I2C3->SR2; // Clear ADDR

        // Read bytes 0 to N-3 normally
        for (uint16_t i = 0; i < rlen - 2; i++) {
            if (!i2c3_wait(&I2C3->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE))
                goto error;
            rdata[i] = I2C3->DR;
        }

        // Wait for BTF (byte N-2 in DR, byte N-1 in shift register)
        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_BTF, I2C_SR1_BTF))
            goto error;

        // Clear ACK, set STOP
        I2C3->CR1 &= ~I2C_CR1_ACK;
        I2C3->CR1 |= I2C_CR1_STOP;

        // Read byte N-2 (shifts byte N-1 to DR)
        rdata[rlen - 2] = I2C3->DR;

        // Wait for byte N-1 to be ready
        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE))
            goto error;

        // Read byte N-1
        rdata[rlen - 1] = I2C3->DR;
    }

    // Wait for STOP to complete (hardware clears STOP bit)
    if (!i2c3_wait(&I2C3->CR1, I2C_CR1_STOP, 0)) {
        // STOP didn't clear - do full recovery (like bringup does)
        i2c3_bus_recovery();
        i2c3_reset();
    }
    // Wait for bus not busy (ignore timeout, like bringup)
    i2c3_wait(&I2C3->SR2, I2C_SR2_BUSY, 0);

    return true;

error:
    I2C3->CR1 |= I2C_CR1_STOP;
    i2c3_wait(&I2C3->SR2, I2C_SR2_BUSY, 0);
    return false;
}

// ----------------------------------------------------------------------------
// I2C1 Low-Level Interface (for VL53L1x on Flow deck)
// ----------------------------------------------------------------------------

// I2C1 pins: PB6 (SCL), PB7 (SDA)
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL_PIN_NUM 6
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA_PIN_NUM 7

static bool s_i2c1_initialized = false;

static bool i2c1_wait(volatile uint32_t *reg, uint32_t mask,
                      uint32_t expected) {
    uint32_t start = platform_get_time_us();
    while ((*reg & mask) != expected) {
        if ((platform_get_time_us() - start) > I2C_TIMEOUT_US) {
            return false;
        }
    }
    return true;
}

static bool i2c1_check_error(void) {
    uint32_t sr1 = I2C1->SR1;
    if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)) {
        I2C1->SR1 =
            sr1 & ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR);
        return true;
    }
    return false;
}

static void i2c1_bus_recovery(void) {
    I2C1->CR1 &= ~I2C_CR1_PE;

    I2C1_SCL_PORT->MODER &= ~(3U << (I2C1_SCL_PIN_NUM * 2));
    I2C1_SCL_PORT->MODER |= (1U << (I2C1_SCL_PIN_NUM * 2));
    I2C1_SCL_PORT->OTYPER |= (1U << I2C1_SCL_PIN_NUM);

    I2C1_SDA_PORT->MODER &= ~(3U << (I2C1_SDA_PIN_NUM * 2));

    for (int i = 0; i < 9; i++) {
        if (I2C1_SDA_PORT->IDR & (1U << I2C1_SDA_PIN_NUM)) {
            break;
        }
        I2C1_SCL_PORT->ODR &= ~(1U << I2C1_SCL_PIN_NUM);
        for (volatile int d = 0; d < 100; d++)
            __NOP();
        I2C1_SCL_PORT->ODR |= (1U << I2C1_SCL_PIN_NUM);
        for (volatile int d = 0; d < 100; d++)
            __NOP();
    }

    I2C1_SDA_PORT->MODER |= (1U << (I2C1_SDA_PIN_NUM * 2));
    I2C1_SDA_PORT->OTYPER |= (1U << I2C1_SDA_PIN_NUM);
    I2C1_SDA_PORT->ODR &= ~(1U << I2C1_SDA_PIN_NUM);
    for (volatile int d = 0; d < 100; d++)
        __NOP();
    I2C1_SDA_PORT->ODR |= (1U << I2C1_SDA_PIN_NUM);
    for (volatile int d = 0; d < 100; d++)
        __NOP();

    I2C1_SCL_PORT->MODER &= ~(3U << (I2C1_SCL_PIN_NUM * 2));
    I2C1_SCL_PORT->MODER |= (2U << (I2C1_SCL_PIN_NUM * 2));
    I2C1_SDA_PORT->MODER &= ~(3U << (I2C1_SDA_PIN_NUM * 2));
    I2C1_SDA_PORT->MODER |= (2U << (I2C1_SDA_PIN_NUM * 2));

    I2C1->CR1 |= I2C_CR1_PE;
}

static void i2c1_reset(void) {
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 42;
    I2C1->CCR = 35 | I2C_CCR_FS;
    I2C1->TRISE = 13;
    I2C1->CR1 = I2C_CR1_PE;
}

static void i2c1_init(void) {
    if (s_i2c1_initialized)
        return;

    // Enable I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure I2C1 GPIO (PB6=SCL, PB7=SDA)
    GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; // AF mode
    GPIOB->AFR[0] |= (4 << (6 * 4)) | (4 << (7 * 4));          // AF4 = I2C1
    GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;        // Open-drain
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0; // Pull-up

    i2c1_bus_recovery();

    // Configure I2C1: 400 kHz fast mode (APB1 = 42 MHz)
    I2C1->CR2 = 42;
    I2C1->CCR = 35 | I2C_CCR_FS;
    I2C1->TRISE = 13;
    I2C1->CR1 = I2C_CR1_PE;

    s_i2c1_initialized = true;
}

static bool i2c1_write(uint8_t addr, uint8_t *data, uint16_t len) {
    i2c1_check_error();

    if (!i2c1_wait(&I2C1->SR2, I2C_SR2_BUSY, 0)) {
        i2c1_bus_recovery();
        i2c1_reset();
        return false;
    }

    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait(&I2C1->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    I2C1->DR = addr << 1;
    if (!i2c1_wait(&I2C1->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR)) {
        if (i2c1_check_error()) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    (void)I2C1->SR2;

    for (uint16_t i = 0; i < len; i++) {
        if (!i2c1_wait(&I2C1->SR1, I2C_SR1_TXE, I2C_SR1_TXE)) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
        if (i2c1_check_error()) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
        I2C1->DR = data[i];
    }

    if (!i2c1_wait(&I2C1->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return true;
}

static bool i2c1_read(uint8_t addr, uint8_t *data, uint16_t len) {
    if (len == 0)
        return false;

    i2c1_check_error();

    if (!i2c1_wait(&I2C1->SR2, I2C_SR2_BUSY, 0)) {
        i2c1_bus_recovery();
        i2c1_reset();
        return false;
    }

    I2C1->CR1 |= I2C_CR1_ACK;

    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait(&I2C1->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    I2C1->DR = (addr << 1) | 1;
    if (!i2c1_wait(&I2C1->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR)) {
        if (i2c1_check_error()) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    if (len == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;

        if (!i2c1_wait(&I2C1->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
            return false;
        }
        data[0] = I2C1->DR;
        return true;
    }

    if (len == 2) {
        I2C1->CR1 |= I2C_CR1_POS;
        (void)I2C1->SR2;
        I2C1->CR1 &= ~I2C_CR1_ACK;

        if (!i2c1_wait(&I2C1->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
            I2C1->CR1 &= ~I2C_CR1_POS;
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }

        I2C1->CR1 |= I2C_CR1_STOP;
        data[0] = I2C1->DR;
        data[1] = I2C1->DR;

        I2C1->CR1 &= ~I2C_CR1_POS;
        return true;
    }

    (void)I2C1->SR2;

    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            if (!i2c1_wait(&I2C1->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
                return false;
            }
            data[i] = I2C1->DR;
        } else if (i == len - 2) {
            if (!i2c1_wait(&I2C1->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
                I2C1->CR1 |= I2C_CR1_STOP;
                return false;
            }
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
            data[i] = I2C1->DR;
        } else {
            if (!i2c1_wait(&I2C1->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
                I2C1->CR1 |= I2C_CR1_STOP;
                return false;
            }
            data[i] = I2C1->DR;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------
// SPI1 Low-Level Interface (for PMW3901)
// ----------------------------------------------------------------------------

static void spi1_init(void) {
    // Enable SPI1 clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure SPI1 GPIO (PA5=SCK, PA6=MISO, PA7=MOSI)
    GPIOA->MODER &=
        ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |=
        (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOA->AFR[0] |=
        (5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4)); // AF5 = SPI1
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 |
                       GPIO_OSPEEDER_OSPEEDR7);

    // Configure PMW3901 CS pin (PB12) as output
    GPIOB->MODER &= ~(3U << (FLOW_SPI_CS_PIN * 2));
    GPIOB->MODER |= (1U << (FLOW_SPI_CS_PIN * 2));
    GPIOB->OSPEEDR |= (3U << (FLOW_SPI_CS_PIN * 2));
    GPIOB->ODR |= (1 << FLOW_SPI_CS_PIN); // CS high (deselected)

    // Configure SPI1: Master, 8-bit, CPOL=1, CPHA=1 (Mode 3), ~1.3 MHz (84/64)
    // PMW3901 max SPI clock is 2 MHz per datasheet
    SPI1->CR1 = SPI_CR1_MSTR |                // Master mode
                SPI_CR1_BR_2 | SPI_CR1_BR_0 | // Baud rate = fPCLK/64 = 1.3 MHz
                SPI_CR1_CPOL |                // CPOL=1
                SPI_CR1_CPHA |                // CPHA=1
                SPI_CR1_SSM |                 // Software slave management
                SPI_CR1_SSI;                  // Internal slave select

    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}

static uint8_t spi1_transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE))
        ;
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE))
        ;
    return SPI1->DR;
}

// ----------------------------------------------------------------------------
// Bosch BMI08x Callbacks
// ----------------------------------------------------------------------------

static int8_t bmi08_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
                             void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    // Use combined write-read with repeated start (proper I2C register read)
    if (!i2c3_write_read(dev_addr, &reg_addr, 1, data, (uint16_t)len))
        return -1;
    return 0;
}

static int8_t bmi08_i2c_write(uint8_t reg_addr, const uint8_t *data,
                              uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    for (uint32_t i = 0; i < len; i++)
        buf[i + 1] = data[i];
    if (!i2c3_write(dev_addr, buf, (uint16_t)(len + 1)))
        return -1;
    return 0;
}

static void bmi08_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    platform_delay_us(period);
}

// ----------------------------------------------------------------------------
// Bosch BMP3 Callbacks
// ----------------------------------------------------------------------------

static int8_t bmp3_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
                            void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    // Use combined write-read with repeated start (proper I2C register read)
    if (!i2c3_write_read(dev_addr, &reg_addr, 1, data, (uint16_t)len))
        return -1;
    return 0;
}

static int8_t bmp3_i2c_write(uint8_t reg_addr, const uint8_t *data,
                             uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    for (uint32_t i = 0; i < len; i++)
        buf[i + 1] = data[i];
    if (!i2c3_write(dev_addr, buf, (uint16_t)(len + 1)))
        return -1;
    return 0;
}

static void bmp3_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    platform_delay_us(period);
}

// ----------------------------------------------------------------------------
// ST VL53L1x Callbacks (uses I2C1 - expansion connector)
// ----------------------------------------------------------------------------

static int vl53l1x_i2c_write(uint8_t dev_addr, uint16_t reg_addr,
                             const uint8_t *data, uint16_t len) {
    uint8_t buf[len + 2];
    buf[0] = (uint8_t)(reg_addr >> 8);
    buf[1] = (uint8_t)(reg_addr & 0xFF);
    for (uint16_t i = 0; i < len; i++)
        buf[i + 2] = data[i];
    if (!i2c1_write(dev_addr, buf, len + 2))
        return -1;
    return 0;
}

static int vl53l1x_i2c_read(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data,
                            uint16_t len) {
    uint8_t reg_buf[2] = {(uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF)};
    if (!i2c1_write(dev_addr, reg_buf, 2))
        return -1;
    if (!i2c1_read(dev_addr, data, len))
        return -1;
    return 0;
}

static void vl53l1x_delay_ms(uint32_t ms) {
    platform_delay_ms(ms);
}

static const vl53l1x_platform_t s_vl53l1x_platform = {
    .i2c_write = vl53l1x_i2c_write,
    .i2c_read = vl53l1x_i2c_read,
    .delay_ms = vl53l1x_delay_ms,
};

// Probe for VL53L1x at a specific address by reading model ID
// Returns true if VL53L1x found at this address
static bool vl53l1x_probe_address(uint8_t addr) {
    // Read model ID register (0x010F) - should return 0xEACC for VL53L1x
    uint8_t id[2];
    if (vl53l1x_i2c_read(addr, 0x010F, id, 2) != 0) {
        return false;
    }
    uint16_t model_id = (uint16_t)(id[0] << 8) | id[1];
    return (model_id == VL53L1X_MODEL_ID);
}

// ----------------------------------------------------------------------------
// Bitcraze PMW3901 Callbacks
// ----------------------------------------------------------------------------

static int pmw3901_spi_transfer(uint8_t tx_data, uint8_t *rx_data) {
    uint8_t rx = spi1_transfer(tx_data);
    if (rx_data)
        *rx_data = rx;
    return 0;
}

static void pmw3901_cs_set(int level) {
    if (level)
        GPIOB->ODR |= (1 << FLOW_SPI_CS_PIN);
    else
        GPIOB->ODR &= ~(1 << FLOW_SPI_CS_PIN);
}

static void pmw3901_delay_ms(uint32_t ms) {
    platform_delay_ms(ms);
}

static void pmw3901_delay_us(uint32_t us) {
    platform_delay_us(us);
}

static const pmw3901_platform_t s_pmw3901_platform = {
    .spi_transfer = pmw3901_spi_transfer,
    .cs_set = pmw3901_cs_set,
    .delay_ms = pmw3901_delay_ms,
    .delay_us = pmw3901_delay_us,
};

// ----------------------------------------------------------------------------
// LED Control
// ----------------------------------------------------------------------------

void platform_led_on(void) {
    LED_PORT->ODR |= LED_PIN;
}
void platform_led_off(void) {
    LED_PORT->ODR &= ~LED_PIN;
}
void platform_led_toggle(void) {
    LED_PORT->ODR ^= LED_PIN;
}

// Blink LED n times (for init feedback) with SWO message
static void init_blink(int n, int on_ms, int off_ms, const char *msg) {
    if (msg) {
        debug_swo_printf("[INIT] %s (%d blinks)\n", msg, n);
    }
    for (int i = 0; i < n; i++) {
        platform_led_on();
        platform_delay_ms(on_ms);
        platform_led_off();
        platform_delay_ms(off_ms);
    }
    platform_delay_ms(300);
}

// Slow blink forever (error indicator)
static void error_blink_forever(const char *msg) {
    debug_swo_printf("[FATAL] %s - halted\n", msg);
    while (1) {
        platform_led_toggle();
        platform_delay_ms(500);
    }
}

// ----------------------------------------------------------------------------
// Sensor Initialization
// ----------------------------------------------------------------------------

static bool init_bmi08x(void) {
    int8_t rslt;

    // Configure accelerometer device
    s_bmi08_dev.intf = BMI08_I2C_INTF;
    s_bmi08_dev.variant = BMI088_VARIANT;
    s_bmi08_dev.read = bmi08_i2c_read;
    s_bmi08_dev.write = bmi08_i2c_write;
    s_bmi08_dev.delay_us = bmi08_delay_us;
    s_bmi08_dev.intf_ptr_accel = &s_bmi08_accel_addr;
    s_bmi08_dev.intf_ptr_gyro = &s_bmi08_gyro_addr;
    s_bmi08_dev.read_write_len = 32;

    debug_swo_printf("  BMI088: accel addr=0x%02X, gyro addr=0x%02X\n",
                     s_bmi08_accel_addr, s_bmi08_gyro_addr);

    // Initialize accelerometer
    debug_swo_printf("  BMI088: accel init...\n");
    rslt = bmi08xa_init(&s_bmi08_dev);
    if (rslt != BMI08_OK) {
        debug_swo_printf("  BMI088: accel init FAILED (rslt=%d)\n", rslt);
        return false;
    }
    debug_swo_printf("  BMI088: accel chip_id=0x%02X\n",
                     s_bmi08_dev.accel_chip_id);

    // Initialize gyroscope
    debug_swo_printf("  BMI088: gyro init...\n");
    rslt = bmi08g_init(&s_bmi08_dev);
    if (rslt != BMI08_OK) {
        debug_swo_printf("  BMI088: gyro init FAILED (rslt=%d)\n", rslt);
        return false;
    }
    debug_swo_printf("  BMI088: gyro chip_id=0x%02X\n",
                     s_bmi08_dev.gyro_chip_id);

    // Configure accelerometer (matching Bitcraze):
    // +/-24g range, 1600Hz ODR, OSR4 (4x oversampling) bandwidth
    s_bmi08_dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
    s_bmi08_dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
    s_bmi08_dev.accel_cfg.bw = BMI08_ACCEL_BW_OSR4;
    s_bmi08_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

    rslt = bmi08a_set_power_mode(&s_bmi08_dev);
    if (rslt != BMI08_OK)
        return false;

    platform_delay_ms(10);

    rslt = bmi08a_set_meas_conf(&s_bmi08_dev);
    if (rslt != BMI08_OK)
        return false;

    // Configure gyroscope (matching Bitcraze):
    // +/-2000 dps, 1000Hz ODR, 116Hz bandwidth
    s_bmi08_dev.gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;
    s_bmi08_dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    s_bmi08_dev.gyro_cfg.bw = BMI08_GYRO_BW_116_ODR_1000_HZ;
    s_bmi08_dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

    rslt = bmi08g_set_power_mode(&s_bmi08_dev);
    if (rslt != BMI08_OK)
        return false;

    platform_delay_ms(30);

    rslt = bmi08g_set_meas_conf(&s_bmi08_dev);
    if (rslt != BMI08_OK)
        return false;

    return true;
}

static bool init_bmp3(void) {
    int8_t rslt;

    // Configure device
    s_bmp3_dev.intf = BMP3_I2C_INTF;
    s_bmp3_dev.read = bmp3_i2c_read;
    s_bmp3_dev.write = bmp3_i2c_write;
    s_bmp3_dev.delay_us = bmp3_delay_us;
    s_bmp3_dev.intf_ptr = &s_bmp3_addr;

    // Initialize
    rslt = bmp3_init(&s_bmp3_dev);
    if (rslt != BMP3_OK) {
        debug_swo_printf("  BMP388 init FAILED (rslt=%d)\n", rslt);
        return false;
    }
    debug_swo_printf("  BMP388: chip_id=0x%02X\n", s_bmp3_dev.chip_id);

    // Add delay after init before configuring
    platform_delay_ms(20);

    // Configure settings (matching Bitcraze):
    // pressure + temperature enabled, 50Hz ODR
    // 8x pressure oversampling, no temperature oversampling
    struct bmp3_settings settings = {0};
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    uint32_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                            BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
                            BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &s_bmp3_dev);
    if (rslt != BMP3_OK) {
        debug_swo_printf("  BMP388 settings FAILED (rslt=%d)\n", rslt);
        return false;
    }

    // Set normal mode
    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &s_bmp3_dev);
    if (rslt != BMP3_OK) {
        debug_swo_printf("  BMP388 mode FAILED (rslt=%d)\n", rslt);
        return false;
    }

    // Delay before first read (matching Bitcraze)
    platform_delay_ms(20);

    return true;
}

static bool init_vl53l1x(void) {
    int8_t rslt;

    // Initialize I2C1 for VL53L1x (on expansion connector)
    i2c1_init();

    // Scan for VL53L1x at known addresses
    // The Bitcraze firmware may reassign from default 0x29 to avoid conflicts
    static const uint8_t addrs_to_scan[] = {0x29, 0x6A, 0x30, 0x31, 0x32, 0x52};
    uint8_t found_addr = 0;

    for (size_t i = 0; i < sizeof(addrs_to_scan); i++) {
        if (vl53l1x_probe_address(addrs_to_scan[i])) {
            found_addr = addrs_to_scan[i];
            break;
        }
    }

    if (found_addr == 0) {
        return false; // VL53L1x not found at any known address
    }

    // Initialize with platform callbacks
    rslt = vl53l1x_init(&s_vl53l1x_dev, &s_vl53l1x_platform);
    if (rslt != 0)
        return false;

    // Override with discovered address (driver defaults to 0x29)
    s_vl53l1x_dev.i2c_address = found_addr;

    // Wait for boot
    uint8_t boot_state = 0;
    for (int i = 0; i < 100; i++) {
        rslt = vl53l1x_boot_state(&s_vl53l1x_dev, &boot_state);
        if (rslt == 0 && boot_state)
            break;
        platform_delay_ms(10);
    }
    if (!boot_state)
        return false;

    // Initialize sensor
    rslt = vl53l1x_sensor_init(&s_vl53l1x_dev);
    if (rslt != 0)
        return false;

    // Configure: short distance mode (good for indoor), 20ms timing budget
    // (matching Bitcraze's faster update rate)
    rslt =
        vl53l1x_set_distance_mode(&s_vl53l1x_dev, VL53L1X_DISTANCE_MODE_SHORT);
    if (rslt != 0)
        return false;

    rslt = vl53l1x_set_timing_budget(&s_vl53l1x_dev, VL53L1X_TIMING_20MS);
    if (rslt != 0)
        return false;

    rslt = vl53l1x_set_inter_measurement(&s_vl53l1x_dev, 25);
    if (rslt != 0)
        return false;

    // Start ranging
    rslt = vl53l1x_start_ranging(&s_vl53l1x_dev);
    if (rslt != 0)
        return false;

    return true;
}

static bool init_pmw3901(void) {
    return pmw3901_init(&s_pmw3901_dev, &s_pmw3901_platform);
}

// ----------------------------------------------------------------------------
// Platform Interface Implementation
// ----------------------------------------------------------------------------

int platform_init(void) {
    // Note: System clock (168 MHz) already configured by SystemInit() before main()

    // Initialize SysTick (1ms)
    systick_init();

    // Initialize GPIO (LED)
    gpio_init();

    init_blink(1, 200, 200, "Starting platform init");

    // Wait for sensor power stabilization (matching Bitcraze)
    debug_swo_printf("[INIT] Waiting %dms for sensor power...\n",
                     SENSOR_STARTUP_DELAY_MS);
    platform_delay_ms(SENSOR_STARTUP_DELAY_MS);

    // Initialize I2C3 for BMI08x and BMP3 (on-board sensors)
    debug_swo_printf("[INIT] I2C3 init (on-board sensors)...\n");
    i2c3_init();

    // Initialize SPI1 for PMW3901 (Flow deck only)
    debug_swo_printf("[INIT] SPI1 init (Flow deck)...\n");
    spi1_init();

    // Initialize BMI08x IMU
    debug_swo_printf("[INIT] BMI088 IMU init...\n");
    if (!init_bmi08x()) {
        init_blink(3, 100, 100, "BMI088 IMU init FAILED");
        error_blink_forever("BMI088 IMU init failed");
    }

    init_blink(2, 200, 200, "BMI088 IMU OK");

    // Initialize BMP3 barometer
    debug_swo_printf("[INIT] BMP388 barometer init...\n");
    if (!init_bmp3()) {
        init_blink(4, 100, 100, "BMP388 barometer init FAILED");
        error_blink_forever("BMP388 barometer init failed");
    }

    // Initialize motors
    debug_swo_printf("[INIT] Motors init...\n");
    if (!motors_init(NULL)) {
        init_blink(5, 100, 100, "Motors init FAILED");
        error_blink_forever("Motors init failed");
    }

    // Try to initialize Flow deck (optional)
    debug_swo_printf("[INIT] Checking for Flow deck...\n");
    s_flow_deck_present = false;
    if (init_pmw3901()) {
        if (init_vl53l1x()) {
            s_flow_deck_present = true;
            debug_swo_printf("[INIT] Flow deck detected (PMW3901 + VL53L1x)\n");
        }
    }
    if (!s_flow_deck_present) {
        debug_swo_printf("[INIT] No Flow deck detected (optional)\n");
    }

    s_initialized = true;
    s_calibrated = false;
    s_armed = false;

    init_blink(3, 200, 200, "Platform init complete");

    return 0;
}

bool platform_self_test(void) {
    if (!s_initialized) {
        debug_swo_printf("[TEST] Self-test called before init!\n");
        return false;
    }

    debug_swo_printf("[TEST] Starting self-test...\n");

    // Test BMI08x: verify chip IDs
    debug_swo_printf("[TEST] BMI088 accel chip ID: 0x%02X (expected 0x%02X)\n",
                     s_bmi08_dev.accel_chip_id, BMI088_ACCEL_CHIP_ID);
    if (s_bmi08_dev.accel_chip_id != BMI088_ACCEL_CHIP_ID) {
        init_blink(6, 100, 100, "BMI088 accel chip ID MISMATCH");
        return false;
    }

    debug_swo_printf("[TEST] BMI088 gyro chip ID: 0x%02X (expected 0x%02X)\n",
                     s_bmi08_dev.gyro_chip_id, BMI08_GYRO_CHIP_ID);
    if (s_bmi08_dev.gyro_chip_id != BMI08_GYRO_CHIP_ID) {
        init_blink(6, 100, 100, "BMI088 gyro chip ID MISMATCH");
        return false;
    }

    // Test BMP3: verify chip ID
    debug_swo_printf(
        "[TEST] BMP3xx chip ID: 0x%02X (expected 0x%02X or 0x%02X)\n",
        s_bmp3_dev.chip_id, BMP3_CHIP_ID, BMP390_CHIP_ID);
    if (s_bmp3_dev.chip_id != BMP3_CHIP_ID &&
        s_bmp3_dev.chip_id != BMP390_CHIP_ID) {
        init_blink(7, 100, 100, "BMP3xx chip ID MISMATCH");
        return false;
    }

    // BMI088 gyroscope built-in self-test (matching Bitcraze)
    debug_swo_printf("[TEST] BMI088 gyro built-in self-test...\n");
    int8_t rslt = bmi08g_perform_selftest(&s_bmi08_dev);
    if (rslt != BMI08_OK) {
        init_blink(8, 100, 100, "BMI088 gyro self-test FAILED");
        return false;
    }
    debug_swo_printf("[TEST] BMI088 gyro self-test OK\n");

    // Re-initialize gyro after self-test (self-test changes config)
    debug_swo_printf("[TEST] Re-initializing gyro after self-test...\n");
    platform_delay_ms(50);
    s_bmi08_dev.gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;
    s_bmi08_dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    s_bmi08_dev.gyro_cfg.bw = BMI08_GYRO_BW_116_ODR_1000_HZ;
    s_bmi08_dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    bmi08g_set_power_mode(&s_bmi08_dev);
    platform_delay_ms(30);
    bmi08g_set_meas_conf(&s_bmi08_dev);

    // BMI088 accelerometer built-in self-test (matching Bitcraze)
    debug_swo_printf("[TEST] BMI088 accel built-in self-test...\n");
    rslt = bmi08xa_perform_selftest(&s_bmi08_dev);
    if (rslt != BMI08_OK) {
        init_blink(9, 100, 100, "BMI088 accel self-test FAILED");
        return false;
    }
    debug_swo_printf("[TEST] BMI088 accel self-test OK\n");

    // Re-initialize accel after self-test (self-test changes config)
    debug_swo_printf("[TEST] Re-initializing accel after self-test...\n");
    platform_delay_ms(50);
    s_bmi08_dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
    s_bmi08_dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
    s_bmi08_dev.accel_cfg.bw = BMI08_ACCEL_BW_OSR4;
    s_bmi08_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    bmi08a_set_power_mode(&s_bmi08_dev);
    platform_delay_ms(10);
    bmi08a_set_meas_conf(&s_bmi08_dev);

    debug_swo_printf("[TEST] All self-tests passed\n");
    // Flow deck sensors are optional
    return true;
}

int platform_calibrate(void) {
    if (!s_initialized) {
        debug_swo_printf("[CAL] Calibrate called before init!\n");
        return -1;
    }

    debug_swo_printf("[CAL] Starting calibration...\n");
    debug_swo_printf("[CAL] Keep drone LEVEL and STATIONARY\n");

    // Accelerometer level check
    debug_swo_printf("[CAL] Checking level (50 samples)...\n");
    struct bmi08_sensor_data accel_data;
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    int accel_samples = 50;

    // Accel scale: 24g range (matching Bitcraze)
    float accel_scale = (24.0f * GRAVITY) / 32768.0f;

    for (int i = 0; i < accel_samples; i++) {
        if (bmi08a_get_data(&accel_data, &s_bmi08_dev) == BMI08_OK) {
            accel_sum[0] += accel_data.x * accel_scale;
            accel_sum[1] += accel_data.y * accel_scale;
            accel_sum[2] += accel_data.z * accel_scale;
        }
        platform_delay_ms(2);
    }

    float accel_avg[3] = {accel_sum[0] / accel_samples,
                          accel_sum[1] / accel_samples,
                          accel_sum[2] / accel_samples};

    debug_swo_printf("[CAL] Accel avg: X=%.2f Y=%.2f Z=%.2f m/s^2\n",
                     (double)accel_avg[0], (double)accel_avg[1],
                     (double)accel_avg[2]);

    // Check if level: X and Y should be near 0, Z should be near -9.8 m/s^2
    float xy_tolerance = 1.0f;
    float z_expected = -GRAVITY;
    float z_tolerance = 1.5f;

    if (fabsf(accel_avg[0]) > xy_tolerance ||
        fabsf(accel_avg[1]) > xy_tolerance ||
        fabsf(accel_avg[2] - z_expected) > z_tolerance) {
        init_blink(10, 50, 50, "WARNING: Drone not level!");
    } else {
        debug_swo_printf("[CAL] Level check OK\n");
    }

    // Gyro bias calibration with variance check (matching Bitcraze)
    // Retry until variance is low enough (drone is stationary)
    debug_swo_printf("[CAL] Gyro bias calibration (%d samples)...\n",
                     CALIBRATION_SAMPLES);
    float gyro_scale = (2000.0f * M_PI / 180.0f) / 32768.0f;
    bool gyro_bias_found = false;
    int max_attempts = 10;

    for (int attempt = 0; attempt < max_attempts && !gyro_bias_found;
         attempt++) {
        if (attempt > 0) {
            debug_swo_printf("[CAL] Gyro variance too high, retry %d/%d...\n",
                             attempt + 1, max_attempts);
        }

        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        float gyro_sum_sq[3] = {0.0f, 0.0f, 0.0f};
        struct bmi08_sensor_data gyro_data;

        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            if (bmi08g_get_data(&gyro_data, &s_bmi08_dev) == BMI08_OK) {
                float gx = gyro_data.x * gyro_scale;
                float gy = gyro_data.y * gyro_scale;
                float gz = gyro_data.z * gyro_scale;
                gyro_sum[0] += gx;
                gyro_sum[1] += gy;
                gyro_sum[2] += gz;
                gyro_sum_sq[0] += gx * gx;
                gyro_sum_sq[1] += gy * gy;
                gyro_sum_sq[2] += gz * gz;
            }
            if (i % 50 == 0) {
                platform_led_toggle();
            }
            platform_delay_ms(1); // 1ms for ~1kHz sample rate
        }

        // Calculate mean and variance
        float mean[3], variance[3];
        for (int j = 0; j < 3; j++) {
            mean[j] = gyro_sum[j] / CALIBRATION_SAMPLES;
            variance[j] =
                (gyro_sum_sq[j] / CALIBRATION_SAMPLES) - (mean[j] * mean[j]);
            // Convert variance to match Bitcraze units (raw LSB^2)
            variance[j] = variance[j] / (gyro_scale * gyro_scale);
        }

        // Check if variance is low enough (drone is stationary)
        if (variance[0] < GYRO_VARIANCE_THRESHOLD &&
            variance[1] < GYRO_VARIANCE_THRESHOLD &&
            variance[2] < GYRO_VARIANCE_THRESHOLD) {
            s_gyro_bias[0] = mean[0];
            s_gyro_bias[1] = mean[1];
            s_gyro_bias[2] = mean[2];
            gyro_bias_found = true;
            debug_swo_printf("[CAL] Gyro bias: X=%.4f Y=%.4f Z=%.4f rad/s\n",
                             (double)s_gyro_bias[0], (double)s_gyro_bias[1],
                             (double)s_gyro_bias[2]);
        } else {
            debug_swo_printf(
                "[CAL] Variance: X=%.1f Y=%.1f Z=%.1f (max=%.1f)\n",
                (double)variance[0], (double)variance[1], (double)variance[2],
                (double)GYRO_VARIANCE_THRESHOLD);
            // Wait before retry
            platform_delay_ms(GYRO_CALIBRATION_TIMEOUT_MS);
        }
    }

    if (!gyro_bias_found) {
        // Use last values even if variance was high
        init_blink(5, 100, 100,
                   "WARNING: Gyro not stable, using best estimate");
    }

    // Accelerometer scale calibration (matching Bitcraze)
    // Measures magnitude and computes scale factor
    debug_swo_printf("[CAL] Accel scale calibration (%d samples)...\n",
                     ACCEL_SCALE_SAMPLES);
    float accel_magnitude_sum = 0.0f;
    int scale_samples = 0;

    for (int i = 0; i < ACCEL_SCALE_SAMPLES; i++) {
        if (bmi08a_get_data(&accel_data, &s_bmi08_dev) == BMI08_OK) {
            float ax = accel_data.x * accel_scale;
            float ay = accel_data.y * accel_scale;
            float az = accel_data.z * accel_scale;
            float magnitude = sqrtf(ax * ax + ay * ay + az * az);
            accel_magnitude_sum += magnitude;
            scale_samples++;
        }
        if (i % 20 == 0) {
            platform_led_toggle();
        }
        platform_delay_ms(2);
    }

    if (scale_samples > 0) {
        float avg_magnitude = accel_magnitude_sum / scale_samples;
        // Scale factor: expected is GRAVITY, actual is avg_magnitude
        if (avg_magnitude > 0.1f) {
            s_accel_scale = GRAVITY / avg_magnitude;
        }
        debug_swo_printf("[CAL] Accel scale factor: %.4f\n",
                         (double)s_accel_scale);
    }

    // Barometer reference calibration
    debug_swo_printf("[CAL] Baro reference calibration (%d samples)...\n",
                     BARO_CALIBRATION_SAMPLES);
    float pressure_sum = 0.0f;
    struct bmp3_data baro_data;

    for (int i = 0; i < BARO_CALIBRATION_SAMPLES; i++) {
        if (bmp3_get_sensor_data(BMP3_PRESS_TEMP, &baro_data, &s_bmp3_dev) ==
            BMP3_OK) {
            pressure_sum += baro_data.pressure;
        }
        if (i % 5 == 0) {
            platform_led_toggle();
        }
        platform_delay_ms(20);
    }

    s_ref_pressure = pressure_sum / BARO_CALIBRATION_SAMPLES;
    debug_swo_printf("[CAL] Baro reference: %.1f Pa (%.1f hPa)\n",
                     (double)s_ref_pressure, (double)(s_ref_pressure / 100.0f));

    // VL53L1x height offset calibration (if Flow deck present)
    if (s_flow_deck_present) {
        debug_swo_printf("[CAL] VL53L1x height offset calibration...\n");
        uint32_t height_sum = 0;
        int height_samples = 0;
        int height_attempts = 20;

        for (int i = 0; i < height_attempts; i++) {
            if (i % 2 == 0) {
                platform_led_toggle();
            }
            platform_delay_ms(50);

            uint8_t is_ready = 0;
            if (vl53l1x_check_data_ready(&s_vl53l1x_dev, &is_ready) == 0 &&
                is_ready) {
                uint16_t distance = 0;
                if (vl53l1x_get_distance(&s_vl53l1x_dev, &distance) == 0) {
                    if (distance > 0 && distance < 200) {
                        height_sum += distance;
                        height_samples++;
                    }
                }
                vl53l1x_clear_interrupt(&s_vl53l1x_dev);
            }
        }

        if (height_samples > 0) {
            s_height_offset_mm = (uint16_t)(height_sum / height_samples);
        }
        debug_swo_printf("[CAL] VL53L1x height offset: %u mm\n",
                         s_height_offset_mm);
    }

    platform_led_off();
    s_calibrated = true;
    debug_swo_printf("[CAL] Calibration complete\n");
    return 0;
}

void platform_read_sensors(sensor_data_t *sensors) {
    // IMU - Accelerometer
    struct bmi08_sensor_data accel_data;
    if (bmi08a_get_data(&accel_data, &s_bmi08_dev) == BMI08_OK) {
        // Convert to m/s^2 (24g range, 16-bit signed) with scale correction
        float scale = (24.0f * GRAVITY) / 32768.0f;
        sensors->accel[0] = accel_data.x * scale * s_accel_scale;
        sensors->accel[1] = accel_data.y * scale * s_accel_scale;
        sensors->accel[2] = accel_data.z * scale * s_accel_scale;
    }

    // IMU - Gyroscope
    struct bmi08_sensor_data gyro_data;
    if (bmi08g_get_data(&gyro_data, &s_bmi08_dev) == BMI08_OK) {
        // Convert to rad/s (2000 dps range, 16-bit signed)
        float scale = (2000.0f * M_PI / 180.0f) / 32768.0f;
        sensors->gyro[0] = gyro_data.x * scale - s_gyro_bias[0];
        sensors->gyro[1] = gyro_data.y * scale - s_gyro_bias[1];
        sensors->gyro[2] = gyro_data.z * scale - s_gyro_bias[2];
    }

    // Barometer
    struct bmp3_data baro_data;
    if (bmp3_get_sensor_data(BMP3_PRESS_TEMP, &baro_data, &s_bmp3_dev) ==
        BMP3_OK) {
        sensors->pressure_hpa = baro_data.pressure / 100.0f;
        sensors->baro_temp_c = baro_data.temperature;
        if (s_ref_pressure > 0.0f) {
            float pressure_ratio = baro_data.pressure / s_ref_pressure;
            sensors->baro_altitude =
                44330.0f * (1.0f - powf(pressure_ratio, 0.19029f));
        } else {
            sensors->baro_altitude = 0.0f;
        }
        sensors->baro_valid = true;
    } else {
        sensors->baro_altitude = 0.0f;
        sensors->baro_valid = false;
    }

    // No magnetometer on Crazyflie 2.1+
    sensors->mag[0] = 0.0f;
    sensors->mag[1] = 0.0f;
    sensors->mag[2] = 0.0f;
    sensors->mag_valid = false;

    // Flow deck integration - provides pseudo-GPS position
    // The flow deck (PMW3901 + VL53L1x) is integrated here in the HAL to
    // provide a generic position interface matching the Webots GPS.

    // Read rangefinder height (VL53L1x)
    uint16_t height_mm = 0;
    bool range_ok = platform_read_height(&height_mm);
    float height_m = 0.0f;
    if (range_ok && height_mm > 0 && height_mm < HAL_TOF_MAX_RANGE_MM) {
        height_m = height_mm / 1000.0f;
    }

    // Read optical flow (PMW3901) and integrate to position
    int16_t delta_x = 0, delta_y = 0;
    bool flow_ok = platform_read_flow(&delta_x, &delta_y);

    // Initialize velocity as invalid
    sensors->velocity_x = 0.0f;
    sensors->velocity_y = 0.0f;
    sensors->velocity_valid = false;

    if (flow_ok && height_m > 0.05f) {
        // Compute dt from previous flow read
        uint32_t now_us = platform_get_time_us();
        float dt = 0.0f;
        if (s_prev_flow_time_us > 0) {
            dt = (now_us - s_prev_flow_time_us) / 1000000.0f;
        }
        s_prev_flow_time_us = now_us;

        if (dt > 0.0f && dt < 0.1f) { // Sanity check: dt between 0 and 100ms
            // Convert pixel deltas to velocity (m/s, body frame)
            // Formula: velocity = pixel_delta * SCALE * height
            float vx_body = delta_x * HAL_FLOW_SCALE * height_m;
            float vy_body = delta_y * HAL_FLOW_SCALE * height_m;

            // Integrate yaw from gyro (simple dead-reckoning)
            // Note: This drifts over time, but flow-based position drifts anyway
            s_prev_yaw += sensors->gyro[2] * dt;

            // Rotate body velocity to world frame
            float cos_yaw = cosf(s_prev_yaw);
            float sin_yaw = sinf(s_prev_yaw);
            float vx_world = vx_body * cos_yaw - vy_body * sin_yaw;
            float vy_world = vx_body * sin_yaw + vy_body * cos_yaw;

            // Provide direct velocity (higher quality than differentiated position)
            sensors->velocity_x = vx_world;
            sensors->velocity_y = vy_world;
            sensors->velocity_valid = true;

            // Integrate velocity to position
            s_integrated_x += vx_world * dt;
            s_integrated_y += vy_world * dt;
        }

        // Provide integrated position as pseudo-GPS
        sensors->gps_x = s_integrated_x;
        sensors->gps_y = s_integrated_y;
        sensors->gps_z = height_m;
        sensors->gps_valid = true;
    } else {
        // No valid flow/range - provide last known position with range altitude
        sensors->gps_x = s_integrated_x;
        sensors->gps_y = s_integrated_y;
        sensors->gps_z = height_m;
        sensors->gps_valid = (height_m > 0.0f);
    }
}

void platform_write_motors(const motor_cmd_t *cmd) {
    if (!s_armed) {
        return;
    }

    motors_cmd_t motor_cmd;
    for (int i = 0; i < 4; i++) {
        motor_cmd.motor[i] = cmd->motor[i];
    }
    motors_set(&motor_cmd);
}

void platform_arm(void) {
    if (s_initialized && s_calibrated) {
        debug_swo_printf("[ARM] Motors ARMED\n");
        motors_arm();
        s_armed = true;
        platform_led_on();
    } else {
        debug_swo_printf("[ARM] Cannot arm: init=%d cal=%d\n", s_initialized,
                         s_calibrated);
    }
}

void platform_disarm(void) {
    debug_swo_printf("[ARM] Motors DISARMED\n");
    motors_disarm();
    s_armed = false;
    platform_led_off();
}

uint32_t platform_get_time_ms(void) {
    return s_sys_tick_ms;
}

uint32_t platform_get_time_us(void) {
    uint32_t ms = s_sys_tick_ms;
    uint32_t ticks = SysTick->VAL;
    uint32_t load = SysTick->LOAD;
    uint32_t us_in_tick = ((load - ticks) * 1000) / (load + 1);
    return ms * 1000 + us_in_tick;
}

void platform_delay_ms(uint32_t ms) {
    uint32_t start = s_sys_tick_ms;
    while ((s_sys_tick_ms - start) < ms) {
        __WFI();
    }
}

void platform_delay_us(uint32_t us) {
    uint32_t start = platform_get_time_us();
    while ((platform_get_time_us() - start) < us) {
        __NOP();
    }
}

void platform_debug_init(void) {
    // TODO: Initialize UART for debug output
}

void platform_debug_printf(const char *fmt, ...) {
    (void)fmt;
}

void platform_emergency_stop(void) {
    motors_emergency_stop();
    s_armed = false;

    for (int i = 0; i < 10; i++) {
        platform_led_toggle();
        platform_delay_ms(50);
    }
    platform_led_off();
}

bool platform_has_flow_deck(void) {
    return s_flow_deck_present;
}

bool platform_read_flow(int16_t *delta_x, int16_t *delta_y) {
    if (!s_flow_deck_present) {
        return false;
    }
    pmw3901_read_motion(&s_pmw3901_dev, delta_x, delta_y);
    return true;
}

bool platform_read_height(uint16_t *height_mm) {
    if (!s_flow_deck_present) {
        return false;
    }

    uint8_t is_ready = 0;
    if (vl53l1x_check_data_ready(&s_vl53l1x_dev, &is_ready) != 0 || !is_ready) {
        return false;
    }

    uint16_t distance = 0;
    if (vl53l1x_get_distance(&s_vl53l1x_dev, &distance) != 0) {
        return false;
    }

    vl53l1x_clear_interrupt(&s_vl53l1x_dev);

    if (distance == 0) {
        return false;
    }

    // Subtract ground offset
    if (distance > s_height_offset_mm) {
        *height_mm = distance - s_height_offset_mm;
    } else {
        *height_mm = 0;
    }

    return true;
}

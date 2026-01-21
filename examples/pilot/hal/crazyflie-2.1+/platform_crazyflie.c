// Crazyflie 2.1+ Platform Layer Implementation
//
// Implements the platform interface using direct STM32F405 peripheral access
// and vendor sensor drivers (Bosch BMI08x, Bosch BMP3, ST VL53L1x, Bitcraze PMW3901).

#include "platform_crazyflie.h"
#include "stm32f4xx.h"
#include "motors.h"

// Vendor drivers
#include "vendor/bosch/bmi08x/bmi08x.h"
#include "vendor/bosch/bmp3/bmp3.h"
#include "vendor/st/vl53l1x/vl53l1x_uld.h"
#include "vendor/bitcraze/pmw3901/pmw3901.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

#define CALIBRATION_SAMPLES 500     // Gyro calibration samples
#define BARO_CALIBRATION_SAMPLES 50 // Barometer calibration samples

// Conversion constants
#define GRAVITY 9.80665f
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// GPIO Pins
#define LED_PIN (1U << 2) // PD2 blue LED on Crazyflie
#define LED_PORT GPIOD

// I2C addresses
#define BMI08_ACCEL_I2C_ADDR 0x18 // BMI088 accelerometer
#define BMI08_GYRO_I2C_ADDR 0x68  // BMI088 gyroscope
#define BMP3_I2C_ADDR 0x77        // BMP388 barometer
#define VL53L1X_I2C_ADDR 0x29     // VL53L1x ToF sensor

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

// Barometer reference pressure (Pa)
static float s_ref_pressure = 0.0f;

// VL53L1x height offset (mm) - ground clearance when on flat surface
static uint16_t s_height_offset_mm = 0;

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

// Reset I2C peripheral after error
static void i2c3_reset(void) {
    // Software reset
    I2C3->CR1 |= I2C_CR1_SWRST;
    I2C3->CR1 &= ~I2C_CR1_SWRST;

    // Reconfigure
    I2C3->CR2 = 42;
    I2C3->CCR = 35;
    I2C3->TRISE = 13;
    I2C3->CR1 = I2C_CR1_PE;
}

static void i2c3_init(void) {
    // Enable I2C3 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

    // Configure I2C3 GPIO (PA8=SCL, PC9=SDA)
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |= GPIO_MODER_MODER8_1; // AF mode
    GPIOA->AFR[1] |= (4 << (0 * 4));     // AF4 = I2C3
    GPIOA->OTYPER |= GPIO_OTYPER_OT8;    // Open-drain
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0; // Pull-up

    GPIOC->MODER &= ~GPIO_MODER_MODER9;
    GPIOC->MODER |= GPIO_MODER_MODER9_1;
    GPIOC->AFR[1] |= (4 << (1 * 4));
    GPIOC->OTYPER |= GPIO_OTYPER_OT9;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR9_0;

    // Bus recovery in case bus is stuck from previous run
    i2c3_bus_recovery();

    // Configure I2C3: 400 kHz
    // APB1 = 42 MHz
    I2C3->CR2 = 42;         // FREQ = 42 MHz
    I2C3->CCR = 35;         // CCR for 400 kHz
    I2C3->TRISE = 13;       // Maximum rise time
    I2C3->CR1 = I2C_CR1_PE; // Enable I2C
}

static bool i2c3_write(uint8_t addr, uint8_t *data, uint16_t len) {
    // Clear any pending errors
    i2c3_check_error();

    // Wait for bus not busy
    if (!i2c3_wait(&I2C3->SR2, I2C_SR2_BUSY, 0)) {
        i2c3_bus_recovery();
        i2c3_reset();
        return false;
    }

    // Generate START
    I2C3->CR1 |= I2C_CR1_START;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        I2C3->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (write)
    I2C3->DR = addr << 1;
    if (!i2c3_wait(&I2C3->SR1, I2C_SR1_ADDR, I2C_SR1_ADDR)) {
        if (i2c3_check_error()) {
            I2C3->CR1 |= I2C_CR1_STOP;
            return false; // NACK - device not responding
        }
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
        if (i2c3_check_error()) {
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

    // Special handling for 1-byte read
    if (len == 1) {
        I2C3->CR1 &= ~I2C_CR1_ACK; // Disable ACK before clearing ADDR
        (void)I2C3->SR2;           // Clear ADDR
        I2C3->CR1 |= I2C_CR1_STOP; // Generate STOP

        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
            return false;
        }
        data[0] = I2C3->DR;
        return true;
    }

    // Multi-byte read
    (void)I2C3->SR2; // Clear ADDR

    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            // Last byte: disable ACK and generate STOP
            I2C3->CR1 &= ~I2C_CR1_ACK;
            I2C3->CR1 |= I2C_CR1_STOP;
        }

        if (!i2c3_wait(&I2C3->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
            if (!(I2C3->CR1 & I2C_CR1_STOP)) {
                I2C3->CR1 |= I2C_CR1_STOP;
            }
            return false;
        }
        data[i] = I2C3->DR;
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
    if (!i2c3_write(dev_addr, &reg_addr, 1))
        return -1;
    if (!i2c3_read(dev_addr, data, (uint16_t)len))
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
    if (!i2c3_write(dev_addr, &reg_addr, 1))
        return -1;
    if (!i2c3_read(dev_addr, data, (uint16_t)len))
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
// ST VL53L1x Callbacks
// ----------------------------------------------------------------------------

static int vl53l1x_i2c_write(uint8_t dev_addr, uint16_t reg_addr,
                             const uint8_t *data, uint16_t len) {
    uint8_t buf[len + 2];
    buf[0] = (uint8_t)(reg_addr >> 8);
    buf[1] = (uint8_t)(reg_addr & 0xFF);
    for (uint16_t i = 0; i < len; i++)
        buf[i + 2] = data[i];
    if (!i2c3_write(dev_addr, buf, len + 2))
        return -1;
    return 0;
}

static int vl53l1x_i2c_read(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data,
                            uint16_t len) {
    uint8_t reg_buf[2] = {(uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF)};
    if (!i2c3_write(dev_addr, reg_buf, 2))
        return -1;
    if (!i2c3_read(dev_addr, data, len))
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

// Blink LED n times (for init feedback)
static void init_blink(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        platform_led_on();
        platform_delay_ms(on_ms);
        platform_led_off();
        platform_delay_ms(off_ms);
    }
    platform_delay_ms(300);
}

// Slow blink forever (error indicator)
static void error_blink_forever(void) {
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

    // Initialize accelerometer
    rslt = bmi08xa_init(&s_bmi08_dev);
    if (rslt != BMI08_OK)
        return false;

    // Initialize gyroscope
    rslt = bmi08g_init(&s_bmi08_dev);
    if (rslt != BMI08_OK)
        return false;

    // Configure accelerometer: +/-6g, 1600Hz ODR, normal mode
    s_bmi08_dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
    s_bmi08_dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
    s_bmi08_dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
    s_bmi08_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

    rslt = bmi08a_set_power_mode(&s_bmi08_dev);
    if (rslt != BMI08_OK)
        return false;

    platform_delay_ms(10);

    rslt = bmi08a_set_meas_conf(&s_bmi08_dev);
    if (rslt != BMI08_OK)
        return false;

    // Configure gyroscope: +/-2000 dps, 2000Hz ODR, normal mode
    s_bmi08_dev.gyro_cfg.odr = BMI08_GYRO_BW_230_ODR_2000_HZ;
    s_bmi08_dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    s_bmi08_dev.gyro_cfg.bw = BMI08_GYRO_BW_230_ODR_2000_HZ;
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
    if (rslt != BMP3_OK)
        return false;

    // Configure settings: pressure + temperature enabled, 50Hz ODR, 4x oversampling
    struct bmp3_settings settings = {0};
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_4X;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    uint32_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                            BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
                            BMP3_SEL_ODR | BMP3_SEL_IIR_FILTER;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &s_bmp3_dev);
    if (rslt != BMP3_OK)
        return false;

    // Set normal mode
    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &s_bmp3_dev);
    if (rslt != BMP3_OK)
        return false;

    return true;
}

static bool init_vl53l1x(void) {
    int8_t rslt;

    // Initialize with platform callbacks
    rslt = vl53l1x_init(&s_vl53l1x_dev, &s_vl53l1x_platform);
    if (rslt != 0)
        return false;

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

    // Configure: short distance mode, 50ms timing budget
    rslt =
        vl53l1x_set_distance_mode(&s_vl53l1x_dev, VL53L1X_DISTANCE_MODE_SHORT);
    if (rslt != 0)
        return false;

    rslt = vl53l1x_set_timing_budget(&s_vl53l1x_dev, VL53L1X_TIMING_50MS);
    if (rslt != 0)
        return false;

    rslt = vl53l1x_set_inter_measurement(&s_vl53l1x_dev, 55);
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

    // 1 blink = starting
    init_blink(1, 200, 200);

    // Initialize I2C3 for BMI08x, BMP3, and VL53L1x
    i2c3_init();

    // Initialize SPI1 for PMW3901 (Flow deck only)
    spi1_init();

    // Initialize BMI08x IMU
    if (!init_bmi08x()) {
        init_blink(3, 100, 100); // 3 fast blinks = IMU failed
        error_blink_forever();
    }

    // 2 blinks = IMU OK
    init_blink(2, 200, 200);

    // Initialize BMP3 barometer
    if (!init_bmp3()) {
        init_blink(4, 100, 100); // 4 fast blinks = baro failed
        error_blink_forever();
    }

    // Initialize motors
    if (!motors_init(NULL)) {
        init_blink(5, 100, 100); // 5 fast blinks = motors failed
        error_blink_forever();
    }

    // Try to initialize Flow deck (optional)
    s_flow_deck_present = false;
    if (init_pmw3901()) {
        if (init_vl53l1x()) {
            s_flow_deck_present = true;
        }
    }

    s_initialized = true;
    s_calibrated = false;
    s_armed = false;

    // 3 blinks = all init complete
    init_blink(3, 200, 200);

    return 0;
}

bool platform_self_test(void) {
    if (!s_initialized) {
        return false;
    }

    // Test BMI08x: read chip IDs
    if (s_bmi08_dev.accel_chip_id != BMI088_ACCEL_CHIP_ID) {
        init_blink(6, 100, 100);
        return false;
    }
    if (s_bmi08_dev.gyro_chip_id != BMI08_GYRO_CHIP_ID) {
        init_blink(6, 100, 100);
        return false;
    }

    // Test BMP3: read chip ID
    if (s_bmp3_dev.chip_id != BMP3_CHIP_ID &&
        s_bmp3_dev.chip_id != BMP390_CHIP_ID) {
        init_blink(7, 100, 100);
        return false;
    }

    // Flow deck sensors are optional
    return true;
}

int platform_calibrate(void) {
    if (!s_initialized) {
        return -1;
    }

    // Accelerometer level check
    struct bmi08_sensor_data accel_data;
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    int accel_samples = 50;

    for (int i = 0; i < accel_samples; i++) {
        if (bmi08a_get_data(&accel_data, &s_bmi08_dev) == BMI08_OK) {
            // Convert to m/s^2 (6g range, 16-bit signed)
            float scale = (6.0f * GRAVITY) / 32768.0f;
            accel_sum[0] += accel_data.x * scale;
            accel_sum[1] += accel_data.y * scale;
            accel_sum[2] += accel_data.z * scale;
        }
        platform_delay_ms(2);
    }

    float accel_avg[3] = {accel_sum[0] / accel_samples,
                          accel_sum[1] / accel_samples,
                          accel_sum[2] / accel_samples};

    // Check if level: X and Y should be near 0, Z should be near -9.8 m/s^2
    float xy_tolerance = 1.0f;
    float z_expected = -GRAVITY;
    float z_tolerance = 1.5f;

    if (fabsf(accel_avg[0]) > xy_tolerance ||
        fabsf(accel_avg[1]) > xy_tolerance ||
        fabsf(accel_avg[2] - z_expected) > z_tolerance) {
        init_blink(10, 50, 50); // Level warning
    }

    // Gyro bias calibration
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    struct bmi08_sensor_data gyro_data;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (bmi08g_get_data(&gyro_data, &s_bmi08_dev) == BMI08_OK) {
            // Convert to rad/s (2000 dps range, 16-bit signed)
            float scale = (2000.0f * M_PI / 180.0f) / 32768.0f;
            gyro_sum[0] += gyro_data.x * scale;
            gyro_sum[1] += gyro_data.y * scale;
            gyro_sum[2] += gyro_data.z * scale;
        }
        if (i % 50 == 0) {
            platform_led_toggle();
        }
        platform_delay_ms(2);
    }

    s_gyro_bias[0] = gyro_sum[0] / CALIBRATION_SAMPLES;
    s_gyro_bias[1] = gyro_sum[1] / CALIBRATION_SAMPLES;
    s_gyro_bias[2] = gyro_sum[2] / CALIBRATION_SAMPLES;

    // Barometer reference calibration
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

    // VL53L1x height offset calibration (if Flow deck present)
    if (s_flow_deck_present) {
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
    }

    platform_led_off();
    s_calibrated = true;
    return 0;
}

void platform_read_sensors(sensor_data_t *sensors) {
    // IMU - Accelerometer
    struct bmi08_sensor_data accel_data;
    if (bmi08a_get_data(&accel_data, &s_bmi08_dev) == BMI08_OK) {
        // Convert to m/s^2 (6g range, 16-bit signed)
        float scale = (6.0f * GRAVITY) / 32768.0f;
        sensors->accel[0] = accel_data.x * scale;
        sensors->accel[1] = accel_data.y * scale;
        sensors->accel[2] = accel_data.z * scale;
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

    // No GPS
    sensors->gps_x = 0.0f;
    sensors->gps_y = 0.0f;
    sensors->gps_z = 0.0f;
    sensors->gps_valid = false;
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
        motors_arm();
        s_armed = true;
        platform_led_on();
    }
}

void platform_disarm(void) {
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

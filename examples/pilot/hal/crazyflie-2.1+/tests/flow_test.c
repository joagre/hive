/**
 * Flow Deck Test for Crazyflie 2.1+
 *
 * Standalone test to verify Flow Deck v2 sensors:
 *   - PMW3901 optical flow sensor (SPI)
 *   - VL53L1x Time-of-Flight ranging sensor (I2C)
 *
 * Usage:
 *   1. Attach Flow Deck v2 to the Crazyflie
 *   2. Build: make flow_test
 *   3. Flash: make flash-flow
 *   4. Watch LED for feedback
 *
 * LED feedback (blue LED on PC4):
 *   2 slow blinks     = Starting
 *   3 quick blinks    = SPI initialized
 *   4 quick blinks    = PMW3901 chip ID verified
 *   5 quick blinks    = I2C initialized
 *   6 quick blinks    = VL53L1x chip ID verified
 *   Fast blink        = Reading sensors (loop)
 *   LED on 500ms      = Valid flow/distance reading
 *   7 blinks          = PMW3901 test passed
 *   8 blinks          = VL53L1x test passed
 *   9 blinks          = Both sensors passed
 *   Continuous slow   = Test complete
 *   Continuous fast   = SPI init failed
 *   Continuous med    = PMW3901 not detected
 *   Double blink      = VL53L1x not detected
 *
 * PMW3901 is on SPI1: PB4 (CS = DECK_GPIO_IO3), PA5 (SCK), PA6 (MISO), PA7 (MOSI)
 * VL53L1x is on I2C3: PA8 (SCL), PC9 (SDA)
 */

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Test Configuration
// ============================================================================

#define PMW3901_READ_DURATION_MS 3000 // How long to read flow data
#define VL53L1X_READ_DURATION_MS 3000 // How long to read distance data
#define READ_INTERVAL_MS 100          // Interval between reads

// ============================================================================
// Hardware Addresses
// ============================================================================

// Peripheral bases
#define PERIPH_BASE 0x40000000UL
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE (PERIPH_BASE + 0x00020000UL)

// GPIO
#define GPIOA_BASE (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE (AHB1PERIPH_BASE + 0x0C00UL)

#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_AFR0 (*(volatile uint32_t *)(GPIOA_BASE + 0x20))
#define GPIOA_AFR1 (*(volatile uint32_t *)(GPIOA_BASE + 0x24))

#define GPIOB_MODER (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OSPEEDR (*(volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_ODR (*(volatile uint32_t *)(GPIOB_BASE + 0x14))

#define GPIOC_MODER (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_OTYPER (*(volatile uint32_t *)(GPIOC_BASE + 0x04))
#define GPIOC_OSPEEDR (*(volatile uint32_t *)(GPIOC_BASE + 0x08))
#define GPIOC_PUPDR (*(volatile uint32_t *)(GPIOC_BASE + 0x0C))
#define GPIOC_AFR1 (*(volatile uint32_t *)(GPIOC_BASE + 0x24))

#define GPIOD_MODER (*(volatile uint32_t *)(GPIOD_BASE + 0x00))
#define GPIOD_OSPEEDR (*(volatile uint32_t *)(GPIOD_BASE + 0x08))
#define GPIOD_ODR (*(volatile uint32_t *)(GPIOD_BASE + 0x14))

// RCC
#define RCC_BASE (AHB1PERIPH_BASE + 0x3800UL)
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_APB2ENR (*(volatile uint32_t *)(RCC_BASE + 0x44))

// SPI1
#define SPI1_BASE (APB2PERIPH_BASE + 0x3000UL)
#define SPI1_CR1 (*(volatile uint32_t *)(SPI1_BASE + 0x00))
#define SPI1_CR2 (*(volatile uint32_t *)(SPI1_BASE + 0x04))
#define SPI1_SR (*(volatile uint32_t *)(SPI1_BASE + 0x08))
#define SPI1_DR (*(volatile uint32_t *)(SPI1_BASE + 0x0C))

// I2C3
#define I2C3_BASE (APB1PERIPH_BASE + 0x5C00UL)
#define I2C3_CR1 (*(volatile uint32_t *)(I2C3_BASE + 0x00))
#define I2C3_CR2 (*(volatile uint32_t *)(I2C3_BASE + 0x04))
#define I2C3_OAR1 (*(volatile uint32_t *)(I2C3_BASE + 0x08))
#define I2C3_DR (*(volatile uint32_t *)(I2C3_BASE + 0x10))
#define I2C3_SR1 (*(volatile uint32_t *)(I2C3_BASE + 0x14))
#define I2C3_SR2 (*(volatile uint32_t *)(I2C3_BASE + 0x18))
#define I2C3_CCR (*(volatile uint32_t *)(I2C3_BASE + 0x1C))
#define I2C3_TRISE (*(volatile uint32_t *)(I2C3_BASE + 0x20))

// SysTick
#define SYSTICK_BASE 0xE000E010UL
#define SYSTICK_CTRL (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))

// Flash
#define FLASH_BASE (AHB1PERIPH_BASE + 0x3C00UL)
#define FLASH_ACR (*(volatile uint32_t *)(FLASH_BASE + 0x00))

// LED Blue (PD2) - directly on STM32, active high
#define LED_PIN (1 << 2)

// PMW3901 chip select (PB4 = DECK_GPIO_IO3)
#define PMW3901_CS_PIN (1 << 4)

// PMW3901 registers
#define PMW3901_REG_PRODUCT_ID 0x00
#define PMW3901_REG_REVISION_ID 0x01
#define PMW3901_REG_MOTION 0x02
#define PMW3901_REG_DELTA_X_L 0x03
#define PMW3901_REG_DELTA_X_H 0x04
#define PMW3901_REG_DELTA_Y_L 0x05
#define PMW3901_REG_DELTA_Y_H 0x06
#define PMW3901_REG_SQUAL 0x07
#define PMW3901_REG_POWER_UP_RESET 0x3A
#define PMW3901_REG_SHUTDOWN 0x3B
#define PMW3901_REG_MOTION_BURST 0x16

// PMW3901 constants
#define PMW3901_PRODUCT_ID 0x49

// VL53L1x I2C address
#define VL53L1X_I2C_ADDR 0x29

// VL53L1x registers (16-bit addresses)
#define VL53L1X_REG_MODEL_ID 0x010F
#define VL53L1X_REG_MODULE_TYPE 0x0110
#define VL53L1X_REG_SOFT_RESET 0x0000
#define VL53L1X_REG_SYSTEM_MODE_START 0x0087
#define VL53L1X_REG_SYSTEM_INT_CLEAR 0x0086
#define VL53L1X_REG_RESULT_RANGE_STATUS 0x0089
#define VL53L1X_REG_RESULT_FINAL_RANGE_MM_SD0 0x0096

// VL53L1x constants
#define VL53L1X_MODEL_ID 0xEA
#define VL53L1X_MODULE_TYPE 0xCC

// ============================================================================
// Global State
// ============================================================================

static volatile uint32_t g_ticks = 0;

// ============================================================================
// SysTick Handler
// ============================================================================

void SysTick_Handler(void) {
    g_ticks++;
}

// ============================================================================
// Helper Functions
// ============================================================================

static void delay_ms(uint32_t ms) {
    uint32_t start = g_ticks;
    while ((g_ticks - start) < ms)
        ;
}

static void delay_us(uint32_t us) {
    // Rough delay for microseconds at 168 MHz
    for (volatile uint32_t i = 0; i < us * 42; i++)
        ;
}

static void led_on(void) {
    GPIOD_ODR |= LED_PIN;
}
static void led_off(void) {
    GPIOD_ODR &= ~LED_PIN;
}
static void led_toggle(void) {
    GPIOD_ODR ^= LED_PIN;
}

static void blink_n(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        led_on();
        delay_ms(on_ms);
        led_off();
        delay_ms(off_ms);
    }
    delay_ms(300);
}

// Double blink pattern for VL53L1x not detected
static void blink_double(void) {
    while (1) {
        led_on();
        delay_ms(100);
        led_off();
        delay_ms(100);
        led_on();
        delay_ms(100);
        led_off();
        delay_ms(500);
    }
}

// ============================================================================
// System Initialization
// ============================================================================

static void clock_init(void) {
    // Configure flash latency for 168 MHz (5 wait states)
    FLASH_ACR = (5 << 0) | (1 << 8) | (1 << 9) | (1 << 10);

    volatile uint32_t *RCC_CR = (volatile uint32_t *)(RCC_BASE + 0x00);
    volatile uint32_t *RCC_PLLCFGR = (volatile uint32_t *)(RCC_BASE + 0x04);
    volatile uint32_t *RCC_CFGR = (volatile uint32_t *)(RCC_BASE + 0x08);

    // Enable HSE
    *RCC_CR |= (1 << 16); // HSEON
    while (!(*RCC_CR & (1 << 17)))
        ; // Wait for HSERDY

    // Configure PLL: HSE=8MHz, PLLM=4, PLLN=168, PLLP=2, PLLQ=7
    *RCC_PLLCFGR = (4 << 0) | (168 << 6) | (0 << 16) | (1 << 22) | (7 << 24);

    // Enable PLL
    *RCC_CR |= (1 << 24);
    while (!(*RCC_CR & (1 << 25)))
        ;

    // Configure prescalers: AHB=1, APB1=4, APB2=2
    *RCC_CFGR = (0 << 4) | (5 << 10) | (4 << 13);

    // Switch to PLL
    *RCC_CFGR |= (2 << 0);
    while (((*RCC_CFGR >> 2) & 0x3) != 2)
        ;
}

static void systick_init(void) {
    SYSTICK_LOAD = 168000 - 1; // 1ms ticks at 168 MHz
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = (1 << 2) | (1 << 1) | (1 << 0);
}

static void gpio_init(void) {
    // Enable GPIO clocks (GPIOA for SPI/I2C, GPIOB for CS, GPIOC for I2C SDA, GPIOD for LED)
    RCC_AHB1ENR |=
        (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3); // GPIOA, GPIOB, GPIOC, GPIOD
    for (volatile int i = 0; i < 100; i++)
        ;

    // Configure PD2 as output (Blue LED)
    GPIOD_MODER &= ~(3 << 4);  // Clear bits for pin 2
    GPIOD_MODER |= (1 << 4);   // Output mode
    GPIOD_OSPEEDR |= (3 << 4); // High speed
    GPIOD_ODR &= ~LED_PIN;     // LED off

    // Configure PB4 as output (PMW3901 CS = DECK_GPIO_IO3)
    GPIOB_MODER &= ~(3 << 8);    // Clear bits for pin 4
    GPIOB_MODER |= (1 << 8);     // Output mode
    GPIOB_OSPEEDR |= (3 << 8);   // High speed
    GPIOB_ODR |= PMW3901_CS_PIN; // CS high (inactive)
}

// ============================================================================
// SPI Functions (for PMW3901)
// ============================================================================

static void spi_init(void) {
    // Enable SPI1 clock
    RCC_APB2ENR |= (1 << 12);
    for (volatile int i = 0; i < 100; i++)
        ;

    // Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as AF5 (SPI1)
    GPIOA_MODER &= ~((3 << 10) | (3 << 12) | (3 << 14));
    GPIOA_MODER |= ((2 << 10) | (2 << 12) | (2 << 14));
    GPIOA_OSPEEDR |= ((3 << 10) | (3 << 12) | (3 << 14));

    // Set AF5 for PA5, PA6, PA7 (bits 20-31 of AFRL)
    GPIOA_AFR0 &= ~((0xF << 20) | (0xF << 24) | (0xF << 28));
    GPIOA_AFR0 |= ((5 << 20) | (5 << 24) | (5 << 28));

    // Configure SPI1: Master, 8-bit, CPOL=1, CPHA=1 (SPI mode 3 for PMW3901)
    // BR = 84MHz/64 = 1.3 MHz (PMW3901 max is 2 MHz)
    SPI1_CR1 = 0;
    SPI1_CR1 = (1 << 2)    // MSTR (Master)
               | (5 << 3)  // BR = /64 (84MHz APB2 / 64 = 1.3 MHz)
               | (1 << 1)  // CPOL = 1 (clock idle high)
               | (1 << 0)  // CPHA = 1 (data sampled on trailing edge)
               | (1 << 9)  // SSM (Software slave management)
               | (1 << 8); // SSI (Internal slave select high)

    // Enable SPI
    SPI1_CR1 |= (1 << 6); // SPE
}

static uint8_t spi_transfer(uint8_t data) {
    // Wait for TXE
    while (!(SPI1_SR & (1 << 1)))
        ;
    SPI1_DR = data;
    // Wait for RXNE
    while (!(SPI1_SR & (1 << 0)))
        ;
    return (uint8_t)SPI1_DR;
}

static void pmw3901_cs_low(void) {
    GPIOB_ODR &= ~PMW3901_CS_PIN;
}
static void pmw3901_cs_high(void) {
    GPIOB_ODR |= PMW3901_CS_PIN;
}

static uint8_t pmw3901_read_reg(uint8_t reg) {
    pmw3901_cs_low();
    delay_us(1);
    spi_transfer(reg & 0x7F); // Read: MSB = 0
    delay_us(35);             // tSRAD
    uint8_t val = spi_transfer(0x00);
    delay_us(1);
    pmw3901_cs_high();
    delay_us(20); // tSRW/tSRR
    return val;
}

static void pmw3901_write_reg(uint8_t reg, uint8_t val) {
    pmw3901_cs_low();
    delay_us(1);
    spi_transfer(reg | 0x80); // Write: MSB = 1
    spi_transfer(val);
    delay_us(1);
    pmw3901_cs_high();
    delay_us(20); // tSWW/tSWR
}

static bool pmw3901_init(void) {
    spi_init();

    // Power-on delay
    delay_ms(50);

    // Reset
    pmw3901_write_reg(PMW3901_REG_POWER_UP_RESET, 0x5A);
    delay_ms(50);

    // Read registers to clear motion data
    pmw3901_read_reg(PMW3901_REG_MOTION);
    pmw3901_read_reg(PMW3901_REG_DELTA_X_L);
    pmw3901_read_reg(PMW3901_REG_DELTA_X_H);
    pmw3901_read_reg(PMW3901_REG_DELTA_Y_L);
    pmw3901_read_reg(PMW3901_REG_DELTA_Y_H);

    // Check product ID
    uint8_t product_id = pmw3901_read_reg(PMW3901_REG_PRODUCT_ID);
    if (product_id != PMW3901_PRODUCT_ID) {
        return false;
    }

    return true;
}

static bool pmw3901_read_motion(int16_t *delta_x, int16_t *delta_y,
                                uint8_t *squal) {
    uint8_t motion = pmw3901_read_reg(PMW3901_REG_MOTION);

    if (motion & 0x80) { // Motion detected
        uint8_t xl = pmw3901_read_reg(PMW3901_REG_DELTA_X_L);
        uint8_t xh = pmw3901_read_reg(PMW3901_REG_DELTA_X_H);
        uint8_t yl = pmw3901_read_reg(PMW3901_REG_DELTA_Y_L);
        uint8_t yh = pmw3901_read_reg(PMW3901_REG_DELTA_Y_H);
        *squal = pmw3901_read_reg(PMW3901_REG_SQUAL);

        *delta_x = (int16_t)((xh << 8) | xl);
        *delta_y = (int16_t)((yh << 8) | yl);
        return true;
    }

    *delta_x = 0;
    *delta_y = 0;
    *squal = pmw3901_read_reg(PMW3901_REG_SQUAL);
    return true;
}

// ============================================================================
// I2C Functions (for VL53L1x)
// ============================================================================

static bool i2c_init(void) {
    // Enable I2C3 clock
    RCC_APB1ENR |= (1 << 23);
    for (volatile int i = 0; i < 100; i++)
        ;

    // Configure PA8 (SCL) as AF4 (I2C3)
    GPIOA_MODER &= ~(3 << 16);
    GPIOA_MODER |= (2 << 16);   // Alternate function
    GPIOA_OTYPER |= (1 << 8);   // Open-drain
    GPIOA_OSPEEDR |= (3 << 16); // High speed
    GPIOA_PUPDR &= ~(3 << 16);
    GPIOA_PUPDR |= (1 << 16); // Pull-up
    GPIOA_AFR1 &= ~(0xF << 0);
    GPIOA_AFR1 |= (4 << 0); // AF4

    // Configure PC9 (SDA) as AF4 (I2C3)
    GPIOC_MODER &= ~(3 << 18);
    GPIOC_MODER |= (2 << 18);   // Alternate function
    GPIOC_OTYPER |= (1 << 9);   // Open-drain
    GPIOC_OSPEEDR |= (3 << 18); // High speed
    GPIOC_PUPDR &= ~(3 << 18);
    GPIOC_PUPDR |= (1 << 18); // Pull-up
    GPIOC_AFR1 &= ~(0xF << 4);
    GPIOC_AFR1 |= (4 << 4); // AF4

    // Reset I2C3
    I2C3_CR1 = (1 << 15); // SWRST
    I2C3_CR1 = 0;

    // Configure I2C3
    // APB1 clock = 42 MHz, I2C clock = 100 kHz (standard mode)
    I2C3_CR2 = 42;   // APB1 frequency in MHz
    I2C3_CCR = 210;  // CCR = APB1/(2*I2C) = 42MHz/(2*100kHz) = 210
    I2C3_TRISE = 43; // Rise time

    // Enable I2C3
    I2C3_CR1 = (1 << 0); // PE

    return true;
}

static bool i2c_wait_flag(volatile uint32_t *reg, uint32_t flag, bool set) {
    uint32_t timeout = 100000;
    while (timeout--) {
        if (set) {
            if (*reg & flag)
                return true;
        } else {
            if (!(*reg & flag))
                return true;
        }
    }
    return false;
}

static bool i2c_start(void) {
    I2C3_CR1 |= (1 << 8);                            // START
    return i2c_wait_flag(&I2C3_SR1, (1 << 0), true); // Wait for SB
}

static bool i2c_stop(void) {
    I2C3_CR1 |= (1 << 9); // STOP
    return true;
}

static bool i2c_send_addr(uint8_t addr, bool read) {
    I2C3_DR = (addr << 1) | (read ? 1 : 0);
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 1), true))
        return false; // Wait for ADDR
    (void)I2C3_SR1;
    (void)I2C3_SR2; // Clear ADDR
    return true;
}

static bool i2c_read_bytes(uint8_t *buf, uint8_t len) {
    if (len == 0)
        return true;

    if (len == 1) {
        I2C3_CR1 &= ~(1 << 10); // ACK = 0
        (void)I2C3_SR1;
        (void)I2C3_SR2;
        i2c_stop();
        if (!i2c_wait_flag(&I2C3_SR1, (1 << 6), true))
            return false; // RXNE
        *buf = I2C3_DR;
    } else {
        I2C3_CR1 |= (1 << 10); // ACK = 1
        for (uint8_t i = 0; i < len; i++) {
            if (i == len - 1) {
                I2C3_CR1 &= ~(1 << 10); // ACK = 0 for last byte
                i2c_stop();
            }
            if (!i2c_wait_flag(&I2C3_SR1, (1 << 6), true))
                return false; // RXNE
            buf[i] = I2C3_DR;
        }
    }

    return true;
}

// VL53L1x uses 16-bit register addresses
static bool vl53l1x_read_reg(uint16_t reg, uint8_t *data, uint8_t len) {
    // Send register address (16-bit, MSB first)
    if (!i2c_start())
        return false;
    if (!i2c_send_addr(VL53L1X_I2C_ADDR, false)) {
        i2c_stop();
        return false;
    }

    if (!i2c_wait_flag(&I2C3_SR1, (1 << 7), true)) {
        i2c_stop();
        return false;
    }
    I2C3_DR = (reg >> 8) & 0xFF; // MSB
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 7), true)) {
        i2c_stop();
        return false;
    }
    I2C3_DR = reg & 0xFF; // LSB
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 2), true)) {
        i2c_stop();
        return false;
    }

    // Read data
    if (!i2c_start())
        return false;
    if (!i2c_send_addr(VL53L1X_I2C_ADDR, true)) {
        i2c_stop();
        return false;
    }
    if (!i2c_read_bytes(data, len))
        return false;

    return true;
}

static bool vl53l1x_write_reg(uint16_t reg, uint8_t data) {
    if (!i2c_start())
        return false;
    if (!i2c_send_addr(VL53L1X_I2C_ADDR, false)) {
        i2c_stop();
        return false;
    }

    if (!i2c_wait_flag(&I2C3_SR1, (1 << 7), true)) {
        i2c_stop();
        return false;
    }
    I2C3_DR = (reg >> 8) & 0xFF; // MSB
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 7), true)) {
        i2c_stop();
        return false;
    }
    I2C3_DR = reg & 0xFF; // LSB
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 7), true)) {
        i2c_stop();
        return false;
    }
    I2C3_DR = data;
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 2), true)) {
        i2c_stop();
        return false;
    }

    i2c_stop();
    return true;
}

static bool vl53l1x_init(void) {
    // Soft reset
    vl53l1x_write_reg(VL53L1X_REG_SOFT_RESET, 0x00);
    delay_ms(1);
    vl53l1x_write_reg(VL53L1X_REG_SOFT_RESET, 0x01);
    delay_ms(100);

    // Check model ID
    uint8_t model_id;
    if (!vl53l1x_read_reg(VL53L1X_REG_MODEL_ID, &model_id, 1)) {
        return false;
    }

    if (model_id != VL53L1X_MODEL_ID) {
        return false;
    }

    // Check module type
    uint8_t module_type;
    if (!vl53l1x_read_reg(VL53L1X_REG_MODULE_TYPE, &module_type, 1)) {
        return false;
    }

    if (module_type != VL53L1X_MODULE_TYPE) {
        return false;
    }

    return true;
}

static uint16_t vl53l1x_read_distance(void) {
    // Start measurement
    vl53l1x_write_reg(VL53L1X_REG_SYSTEM_MODE_START, 0x40);
    delay_ms(50);

    // Read result
    uint8_t buf[2];
    if (!vl53l1x_read_reg(VL53L1X_REG_RESULT_FINAL_RANGE_MM_SD0, buf, 2)) {
        return 0;
    }

    // Clear interrupt
    vl53l1x_write_reg(VL53L1X_REG_SYSTEM_INT_CLEAR, 0x01);

    // Stop measurement
    vl53l1x_write_reg(VL53L1X_REG_SYSTEM_MODE_START, 0x00);

    return (uint16_t)((buf[0] << 8) | buf[1]);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    bool pmw3901_ok = false;
    bool vl53l1x_ok = false;

    // Initialize system
    clock_init();
    systick_init();
    gpio_init();

    // 2 slow blinks = Starting
    blink_n(2, 300, 300);
    delay_ms(500);

    // Test PMW3901 (SPI)
    spi_init();
    blink_n(3, 100, 100); // 3 quick blinks = SPI initialized
    delay_ms(500);

    if (pmw3901_init()) {
        blink_n(4, 100, 100); // 4 quick blinks = PMW3901 chip ID verified
        delay_ms(500);

        // Read flow data
        uint32_t start_time = g_ticks;
        int valid_reads = 0;

        while ((g_ticks - start_time) < PMW3901_READ_DURATION_MS) {
            int16_t delta_x, delta_y;
            uint8_t squal;

            if (pmw3901_read_motion(&delta_x, &delta_y, &squal)) {
                // Check if surface quality is reasonable (> 20)
                if (squal > 20) {
                    valid_reads++;
                    led_on();
                    delay_ms(30);
                    led_off();
                }
            }

            led_toggle();
            delay_ms(READ_INTERVAL_MS - 30);
        }

        if (valid_reads > 10) {
            pmw3901_ok = true;
        }
    } else {
        // PMW3901 not detected - continuous medium blink
        while (1) {
            led_toggle();
            delay_ms(250);
        }
    }

    delay_ms(500);

    // Test VL53L1x (I2C)
    if (!i2c_init()) {
        // I2C init failed - continuous fast blink
        while (1) {
            led_toggle();
            delay_ms(100);
        }
    }

    blink_n(5, 100, 100); // 5 quick blinks = I2C initialized
    delay_ms(500);

    if (vl53l1x_init()) {
        blink_n(6, 100, 100); // 6 quick blinks = VL53L1x chip ID verified
        delay_ms(500);

        // Read distance data
        uint32_t start_time = g_ticks;
        int valid_reads = 0;

        while ((g_ticks - start_time) < VL53L1X_READ_DURATION_MS) {
            uint16_t distance_mm = vl53l1x_read_distance();

            // Check if distance is reasonable (10mm to 4000mm)
            if (distance_mm > 10 && distance_mm < 4000) {
                valid_reads++;
                led_on();
                delay_ms(100);
                led_off();
            }

            led_toggle();
            delay_ms(READ_INTERVAL_MS);
        }

        if (valid_reads > 5) {
            vl53l1x_ok = true;
        }
    } else {
        // VL53L1x not detected - double blink pattern
        blink_double();
    }

    delay_ms(500);

    // Report results
    if (pmw3901_ok) {
        blink_n(7, 200, 200); // 7 blinks = PMW3901 passed
        delay_ms(500);
    }

    if (vl53l1x_ok) {
        blink_n(8, 200, 200); // 8 blinks = VL53L1x passed
        delay_ms(500);
    }

    if (pmw3901_ok && vl53l1x_ok) {
        blink_n(9, 200, 200); // 9 blinks = Both sensors passed
        delay_ms(500);

        // Success: LED on for 1 second
        led_on();
        delay_ms(1000);
        led_off();
    }

    // Test complete - slow continuous blink
    while (1) {
        led_toggle();
        delay_ms(1000);
    }
}

// ============================================================================
// Startup and Vector Table
// ============================================================================

extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata;
extern uint32_t _sbss, _ebss;

void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss)
        *dst++ = 0;
    main();
    while (1)
        ;
}

void Default_Handler(void) {
    while (1)
        ;
}

void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector"))) const uint32_t g_vectors[] = {
    (uint32_t)&_estack,
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    (uint32_t)MemManage_Handler,
    (uint32_t)BusFault_Handler,
    (uint32_t)UsageFault_Handler,
    0,
    0,
    0,
    0,
    (uint32_t)SVC_Handler,
    (uint32_t)DebugMon_Handler,
    0,
    (uint32_t)PendSV_Handler,
    (uint32_t)SysTick_Handler,
};

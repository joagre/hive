/**
 * BMP388 Barometer Test for Crazyflie 2.1+
 *
 * Standalone test to verify BMP388 barometer communication and readings.
 * Uses direct I2C register access (no HAL).
 *
 * Usage:
 *   1. Build: make baro_test
 *   2. Flash: make flash-baro
 *   3. Watch LED for feedback
 *
 * LED feedback (blue LED on PC4):
 *   2 slow blinks    = Starting
 *   3 quick blinks   = I2C initialized
 *   4 quick blinks   = BMP388 chip ID verified
 *   5 quick blinks   = Calibration data read
 *   Fast blink       = Reading sensor data (loop)
 *   LED on 1s        = Valid pressure reading
 *   Continuous slow  = Test complete
 *   Continuous fast  = I2C init failed
 *   Continuous med   = Chip ID mismatch
 *
 * BMP388 is on I2C3: PA8 (SCL), PC9 (SDA)
 */

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Test Configuration
// ============================================================================

#define BMP388_I2C_ADDR 0x77  // Default address (can also be 0x76)
#define READ_DURATION_MS 5000 // How long to read sensor data
#define READ_INTERVAL_MS 100  // Interval between reads

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
#define GPIOC_BASE (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE (AHB1PERIPH_BASE + 0x0C00UL)

#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_AFR1 (*(volatile uint32_t *)(GPIOA_BASE + 0x24))

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

// BMP388 registers
#define BMP388_REG_CHIP_ID 0x00
#define BMP388_REG_STATUS 0x03
#define BMP388_REG_DATA_0 0x04     // Pressure XLSB
#define BMP388_REG_CALIB_DATA 0x31 // Start of calibration data (21 bytes)
#define BMP388_REG_PWR_CTRL 0x1B
#define BMP388_REG_OSR 0x1C
#define BMP388_REG_ODR 0x1D
#define BMP388_REG_CONFIG 0x1F
#define BMP388_REG_CMD 0x7E

// BMP388 constants
#define BMP388_CHIP_ID_VALUE 0x50
#define BMP388_CMD_SOFTRESET 0xB6

// ============================================================================
// Global State
// ============================================================================

static volatile uint32_t g_ticks = 0;

// Calibration coefficients
typedef struct {
    uint16_t t1;
    uint16_t t2;
    int8_t t3;
    int16_t p1;
    int16_t p2;
    int8_t p3;
    int8_t p4;
    uint16_t p5;
    uint16_t p6;
    int8_t p7;
    int8_t p8;
    int16_t p9;
    int8_t p10;
    int8_t p11;
} bmp388_calib_t;

static bmp388_calib_t g_calib;
static float g_t_lin = 0.0f;

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
    // Enable GPIO clocks (GPIOA for I2C, GPIOC for I2C SDA, GPIOD for LED)
    RCC_AHB1ENR |= (1 << 0) | (1 << 2) | (1 << 3); // GPIOA, GPIOC, GPIOD
    for (volatile int i = 0; i < 100; i++)
        ;

    // Configure PD2 as output (Blue LED)
    GPIOD_MODER &= ~(3 << 4);  // Clear bits for pin 2
    GPIOD_MODER |= (1 << 4);   // Output mode
    GPIOD_OSPEEDR |= (3 << 4); // High speed
    GPIOD_ODR &= ~LED_PIN;     // LED off
}

// ============================================================================
// I2C Functions
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
    I2C3_TRISE = 43; // Rise time = (1000ns / (1/42MHz)) + 1 = 43

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
    (void)I2C3_SR2; // Clear ADDR by reading SR1 and SR2
    return true;
}

static bool i2c_write_byte(uint8_t data) {
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 7), true))
        return false; // Wait for TXE
    I2C3_DR = data;
    return i2c_wait_flag(&I2C3_SR1, (1 << 2), true); // Wait for BTF
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

static bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data,
                         uint8_t len) {
    // Send register address
    if (!i2c_start())
        return false;
    if (!i2c_send_addr(addr, false)) {
        i2c_stop();
        return false;
    }
    if (!i2c_write_byte(reg)) {
        i2c_stop();
        return false;
    }

    // Read data
    if (!i2c_start())
        return false;
    if (!i2c_send_addr(addr, true)) {
        i2c_stop();
        return false;
    }
    if (!i2c_read_bytes(data, len))
        return false;

    return true;
}

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data) {
    if (!i2c_start())
        return false;
    if (!i2c_send_addr(addr, false)) {
        i2c_stop();
        return false;
    }

    if (!i2c_wait_flag(&I2C3_SR1, (1 << 7), true)) {
        i2c_stop();
        return false;
    } // TXE
    I2C3_DR = reg;
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 7), true)) {
        i2c_stop();
        return false;
    } // TXE
    I2C3_DR = data;
    if (!i2c_wait_flag(&I2C3_SR1, (1 << 2), true)) {
        i2c_stop();
        return false;
    } // BTF

    i2c_stop();
    return true;
}

// ============================================================================
// BMP388 Functions
// ============================================================================

static bool bmp388_read_calibration(void) {
    uint8_t buf[21];
    if (!i2c_read_reg(BMP388_I2C_ADDR, BMP388_REG_CALIB_DATA, buf, 21)) {
        return false;
    }

    // Parse calibration coefficients (little-endian)
    g_calib.t1 = (uint16_t)(buf[1] << 8) | buf[0];
    g_calib.t2 = (uint16_t)(buf[3] << 8) | buf[2];
    g_calib.t3 = (int8_t)buf[4];

    g_calib.p1 = (int16_t)((buf[6] << 8) | buf[5]);
    g_calib.p2 = (int16_t)((buf[8] << 8) | buf[7]);
    g_calib.p3 = (int8_t)buf[9];
    g_calib.p4 = (int8_t)buf[10];
    g_calib.p5 = (uint16_t)(buf[12] << 8) | buf[11];
    g_calib.p6 = (uint16_t)(buf[14] << 8) | buf[13];
    g_calib.p7 = (int8_t)buf[15];
    g_calib.p8 = (int8_t)buf[16];
    g_calib.p9 = (int16_t)((buf[18] << 8) | buf[17]);
    g_calib.p10 = (int8_t)buf[19];
    g_calib.p11 = (int8_t)buf[20];

    return true;
}

static float compensate_temperature(uint32_t raw_temp) {
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(raw_temp - (256.0f * g_calib.t1));
    partial_data2 = g_calib.t2 * (1.0f / 1073741824.0f);

    g_t_lin = partial_data1 * partial_data2 + partial_data1 * partial_data1 *
                                                  g_calib.t3 *
                                                  (1.0f / 281474976710656.0f);

    return g_t_lin;
}

static float compensate_pressure(uint32_t raw_press) {
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    partial_data1 = g_calib.p6 * g_t_lin;
    partial_data2 = g_calib.p7 * (g_t_lin * g_t_lin);
    partial_data3 = g_calib.p8 * (g_t_lin * g_t_lin * g_t_lin);
    partial_out1 = g_calib.p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = g_calib.p2 * g_t_lin;
    partial_data2 = g_calib.p3 * (g_t_lin * g_t_lin);
    partial_data3 = g_calib.p4 * (g_t_lin * g_t_lin * g_t_lin);
    partial_out2 = (float)raw_press *
                   (g_calib.p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = g_calib.p9 + g_calib.p10 * g_t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 =
        partial_data3 +
        ((float)raw_press * (float)raw_press * (float)raw_press) * g_calib.p11;

    return partial_out1 + partial_out2 + partial_data4;
}

static bool bmp388_init(void) {
    // Soft reset
    i2c_write_reg(BMP388_I2C_ADDR, BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
    delay_ms(10);

    // Verify chip ID
    uint8_t chip_id;
    if (!i2c_read_reg(BMP388_I2C_ADDR, BMP388_REG_CHIP_ID, &chip_id, 1)) {
        return false;
    }

    if (chip_id != BMP388_CHIP_ID_VALUE) {
        return false;
    }

    // 4 quick blinks = chip ID verified
    blink_n(4, 100, 100);

    // Read calibration data
    if (!bmp388_read_calibration()) {
        return false;
    }

    // 5 quick blinks = calibration data read
    blink_n(5, 100, 100);

    // Configure: OSR x8 for pressure, x1 for temp
    i2c_write_reg(BMP388_I2C_ADDR, BMP388_REG_OSR, (0x00 << 3) | 0x03);

    // ODR = 50 Hz
    i2c_write_reg(BMP388_I2C_ADDR, BMP388_REG_ODR, 0x02);

    // IIR filter coef = 3
    i2c_write_reg(BMP388_I2C_ADDR, BMP388_REG_CONFIG, 0x02 << 1);

    // Enable pressure and temperature, normal mode
    i2c_write_reg(BMP388_I2C_ADDR, BMP388_REG_PWR_CTRL, 0x33);

    delay_ms(10);

    return true;
}

static bool bmp388_read(float *pressure_pa, float *temp_c) {
    uint8_t buf[6];
    if (!i2c_read_reg(BMP388_I2C_ADDR, BMP388_REG_DATA_0, buf, 6)) {
        return false;
    }

    // Assemble 24-bit raw values (LSB first)
    uint32_t raw_press = (uint32_t)(buf[2] << 16) | (buf[1] << 8) | buf[0];
    uint32_t raw_temp = (uint32_t)(buf[5] << 16) | (buf[4] << 8) | buf[3];

    // Compensate (temperature must be done first)
    *temp_c = compensate_temperature(raw_temp);
    *pressure_pa = compensate_pressure(raw_press);

    return true;
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    // Initialize system
    clock_init();
    systick_init();
    gpio_init();

    // 2 slow blinks = Starting
    blink_n(2, 300, 300);
    delay_ms(500);

    // Initialize I2C
    if (!i2c_init()) {
        // Error: continuous fast blink
        while (1) {
            led_toggle();
            delay_ms(100);
        }
    }

    // 3 quick blinks = I2C initialized
    blink_n(3, 100, 100);
    delay_ms(500);

    // Initialize BMP388
    if (!bmp388_init()) {
        // Error: continuous medium blink
        while (1) {
            led_toggle();
            delay_ms(250);
        }
    }

    delay_ms(500);

    // Read sensor data loop
    uint32_t start_time = g_ticks;
    int valid_reads = 0;

    while ((g_ticks - start_time) < READ_DURATION_MS) {
        float pressure_pa, temp_c;

        if (bmp388_read(&pressure_pa, &temp_c)) {
            // Check if pressure is in reasonable range (80000-120000 Pa)
            if (pressure_pa > 80000.0f && pressure_pa < 120000.0f) {
                valid_reads++;
                // LED on briefly for valid reading
                led_on();
                delay_ms(50);
                led_off();
            }
        }

        led_toggle();
        delay_ms(READ_INTERVAL_MS - 50);
    }

    // Indicate result
    delay_ms(500);

    if (valid_reads > 20) {
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

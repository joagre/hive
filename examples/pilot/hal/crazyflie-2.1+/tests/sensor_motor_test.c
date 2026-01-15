/**
 * Motor and Sensor Diagnostic for Crazyflie 2.1+
 *
 * Comprehensive test to identify motor wiring and verify sensors:
 *   1. Tests each motor individually (count LED blinks to identify)
 *   2. Detects motor rotation direction using BMI088 gyroscope
 *   3. Reads and displays sensor data via LED patterns
 *
 * Usage:
 *   1. REMOVE PROPELLERS or use a test rig!
 *   2. Build: make -C tests sensor_motor_test
 *   3. Flash: make -C tests flash-sensor
 *   4. Count LED blinks to identify which motor is being tested
 *   5. Observe motor position and rotation
 *
 * LED feedback (blue LED on PC4):
 *   N blinks = Testing motor N (1-4)
 *   Fast blink during motor spin = Motor running
 *   Slow blinks after each motor = Rotation detected:
 *     1 slow blink = CCW detected
 *     2 slow blinks = CW detected
 *     3 slow blinks = Unclear/no rotation
 *   10 fast blinks = All motors test starting
 *   Continuous slow blink = Test complete
 *   Continuous fast blink = Error
 *
 * Motor layout (X-configuration, viewed from above):
 *          Front
 *      M1(CCW)  M2(CW)
 *          +--+
 *          |  |
 *          +--+
 *      M4(CW)  M3(CCW)
 *          Rear
 *
 * TIM2 PWM: PA0=M1, PA1=M2, PA2=M3, PA3=M4
 * BMI088: SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI), PB4=Gyro CS, PB5=Accel CS
 */

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Test Configuration
// ============================================================================

#define TEST_SPEED          40      // ~16% duty cycle - enough to spin but safe
#define SPIN_DURATION_MS    2000    // How long to spin each motor
#define GYRO_SAMPLES        50      // Gyro samples during spin

// ============================================================================
// Hardware Addresses
// ============================================================================

// Peripheral bases
#define PERIPH_BASE       0x40000000UL
#define APB1PERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE   (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE   (PERIPH_BASE + 0x00020000UL)

// GPIO
#define GPIOA_BASE        (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE        (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE        (AHB1PERIPH_BASE + 0x0800UL)

#define GPIOA_MODER       (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OSPEEDR     (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR       (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_AFR0        (*(volatile uint32_t *)(GPIOA_BASE + 0x20))

#define GPIOB_MODER       (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OSPEEDR     (*(volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR       (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_ODR         (*(volatile uint32_t *)(GPIOB_BASE + 0x14))
#define GPIOB_BSRR        (*(volatile uint32_t *)(GPIOB_BASE + 0x18))

#define GPIOC_MODER       (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_OSPEEDR     (*(volatile uint32_t *)(GPIOC_BASE + 0x08))
#define GPIOC_ODR         (*(volatile uint32_t *)(GPIOC_BASE + 0x14))

// RCC
#define RCC_BASE          (AHB1PERIPH_BASE + 0x3800UL)
#define RCC_AHB1ENR       (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR       (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_APB2ENR       (*(volatile uint32_t *)(RCC_BASE + 0x44))

// TIM2
#define TIM2_BASE         (APB1PERIPH_BASE + 0x0000UL)
#define TIM2_CR1          (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_CCMR1        (*(volatile uint32_t *)(TIM2_BASE + 0x18))
#define TIM2_CCMR2        (*(volatile uint32_t *)(TIM2_BASE + 0x1C))
#define TIM2_CCER         (*(volatile uint32_t *)(TIM2_BASE + 0x20))
#define TIM2_PSC          (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR          (*(volatile uint32_t *)(TIM2_BASE + 0x2C))
#define TIM2_CCR1         (*(volatile uint32_t *)(TIM2_BASE + 0x34))
#define TIM2_CCR2         (*(volatile uint32_t *)(TIM2_BASE + 0x38))
#define TIM2_CCR3         (*(volatile uint32_t *)(TIM2_BASE + 0x3C))
#define TIM2_CCR4         (*(volatile uint32_t *)(TIM2_BASE + 0x40))
#define TIM2_EGR          (*(volatile uint32_t *)(TIM2_BASE + 0x14))

// SPI1
#define SPI1_BASE         (APB2PERIPH_BASE + 0x3000UL)
#define SPI1_CR1          (*(volatile uint32_t *)(SPI1_BASE + 0x00))
#define SPI1_CR2          (*(volatile uint32_t *)(SPI1_BASE + 0x04))
#define SPI1_SR           (*(volatile uint32_t *)(SPI1_BASE + 0x08))
#define SPI1_DR           (*(volatile uint32_t *)(SPI1_BASE + 0x0C))

// SysTick
#define SYSTICK_BASE      0xE000E010UL
#define SYSTICK_CTRL      (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD      (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL       (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))

// Flash
#define FLASH_BASE        (AHB1PERIPH_BASE + 0x3C00UL)
#define FLASH_ACR         (*(volatile uint32_t *)(FLASH_BASE + 0x00))

// PWM configuration
#define PWM_PRESCALER     0
#define PWM_PERIOD        255

// LED (PC4)
#define LED_PIN           (1 << 4)

// BMI088 chip selects (directly on STM32)
#define BMI088_GYRO_CS_PIN   (1 << 4)   // PB4
#define BMI088_ACCEL_CS_PIN  (1 << 5)   // PB5

// BMI088 registers
#define BMI088_GYRO_CHIP_ID      0x00
#define BMI088_GYRO_RATE_Z_LSB   0x06
#define BMI088_GYRO_RANGE        0x0F
#define BMI088_GYRO_BANDWIDTH    0x10
#define BMI088_GYRO_SOFTRESET    0x14

// ============================================================================
// Global State
// ============================================================================

static volatile uint32_t g_ticks = 0;

typedef struct {
    int channel;
    int rotation;       // +1=CW, -1=CCW, 0=unknown
    int gyro_z_sum;
} motor_info_t;

static motor_info_t g_motors[4];

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
    while ((g_ticks - start) < ms);
}

static void led_on(void)     { GPIOC_ODR |= LED_PIN; }
static void led_off(void)    { GPIOC_ODR &= ~LED_PIN; }
static void led_toggle(void) { GPIOC_ODR ^= LED_PIN; }

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
    *RCC_CR |= (1 << 16);  // HSEON
    while (!(*RCC_CR & (1 << 17)));  // Wait for HSERDY

    // Configure PLL: HSE=8MHz, PLLM=4, PLLN=168, PLLP=2, PLLQ=7
    *RCC_PLLCFGR = (4 << 0) | (168 << 6) | (0 << 16) | (1 << 22) | (7 << 24);

    // Enable PLL
    *RCC_CR |= (1 << 24);
    while (!(*RCC_CR & (1 << 25)));

    // Configure prescalers: AHB=1, APB1=4, APB2=2
    *RCC_CFGR = (0 << 4) | (5 << 10) | (4 << 13);

    // Switch to PLL
    *RCC_CFGR |= (2 << 0);
    while (((*RCC_CFGR >> 2) & 0x3) != 2);
}

static void systick_init(void) {
    SYSTICK_LOAD = 168000 - 1;  // 1ms ticks at 168 MHz
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = (1 << 2) | (1 << 1) | (1 << 0);
}

static void gpio_init(void) {
    // Enable GPIO clocks
    RCC_AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);  // GPIOA, GPIOB, GPIOC
    for (volatile int i = 0; i < 100; i++);

    // Configure PC4 as output (LED)
    GPIOC_MODER &= ~(3 << 8);
    GPIOC_MODER |= (1 << 8);
    GPIOC_OSPEEDR |= (3 << 8);
    GPIOC_ODR &= ~LED_PIN;

    // Configure PB4, PB5 as output (BMI088 CS pins)
    GPIOB_MODER &= ~((3 << 8) | (3 << 10));
    GPIOB_MODER |= ((1 << 8) | (1 << 10));
    GPIOB_OSPEEDR |= ((3 << 8) | (3 << 10));
    // Set CS high (inactive)
    GPIOB_ODR |= BMI088_GYRO_CS_PIN | BMI088_ACCEL_CS_PIN;
}

static bool motors_init(void) {
    // Enable TIM2 clock
    RCC_APB1ENR |= (1 << 0);
    for (volatile int i = 0; i < 100; i++);

    // Configure PA0-PA3 as alternate function (AF1 = TIM2)
    GPIOA_MODER &= ~((3 << 0) | (3 << 2) | (3 << 4) | (3 << 6));
    GPIOA_MODER |= ((2 << 0) | (2 << 2) | (2 << 4) | (2 << 6));
    GPIOA_OSPEEDR |= ((3 << 0) | (3 << 2) | (3 << 4) | (3 << 6));
    GPIOA_PUPDR &= ~((3 << 0) | (3 << 2) | (3 << 4) | (3 << 6));
    GPIOA_AFR0 &= ~0xFFFF;
    GPIOA_AFR0 |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);

    // Configure TIM2
    TIM2_CR1 = 0;
    TIM2_PSC = PWM_PRESCALER;
    TIM2_ARR = PWM_PERIOD;
    TIM2_CCMR1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM2_CCMR2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM2_CCER = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);
    TIM2_CCR1 = 0;
    TIM2_CCR2 = 0;
    TIM2_CCR3 = 0;
    TIM2_CCR4 = 0;
    TIM2_EGR = 1;
    TIM2_CR1 = (1 << 7) | (1 << 0);

    return true;
}

static void motor_set(int channel, uint16_t speed) {
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    switch (channel) {
        case 0: TIM2_CCR1 = speed; break;
        case 1: TIM2_CCR2 = speed; break;
        case 2: TIM2_CCR3 = speed; break;
        case 3: TIM2_CCR4 = speed; break;
    }
}

static void motors_stop_all(void) {
    TIM2_CCR1 = 0;
    TIM2_CCR2 = 0;
    TIM2_CCR3 = 0;
    TIM2_CCR4 = 0;
}

// ============================================================================
// SPI Functions
// ============================================================================

static void spi_init(void) {
    // Enable SPI1 clock
    RCC_APB2ENR |= (1 << 12);
    for (volatile int i = 0; i < 100; i++);

    // Configure PA5 (SCK), PA6 (MISO), PA7 (MOSI) as AF5 (SPI1)
    GPIOA_MODER &= ~((3 << 10) | (3 << 12) | (3 << 14));
    GPIOA_MODER |= ((2 << 10) | (2 << 12) | (2 << 14));
    GPIOA_OSPEEDR |= ((3 << 10) | (3 << 12) | (3 << 14));

    // Set AF5 for PA5, PA6, PA7 (bits 20-31 of AFRL)
    GPIOA_AFR0 &= ~((0xF << 20) | (0xF << 24) | (0xF << 28));
    GPIOA_AFR0 |= ((5 << 20) | (5 << 24) | (5 << 28));

    // Configure SPI1: Master, 8-bit, CPOL=1, CPHA=1, BR=84MHz/16=5.25MHz
    // For BMI088: CPOL=1, CPHA=1 (SPI mode 3)
    SPI1_CR1 = 0;
    SPI1_CR1 = (1 << 2)    // MSTR (Master)
             | (3 << 3)    // BR = /16 (84MHz APB2 / 16 = 5.25 MHz)
             | (1 << 1)    // CPOL = 1
             | (1 << 0)    // CPHA = 1
             | (1 << 9)    // SSM (Software slave management)
             | (1 << 8);   // SSI (Internal slave select high)

    // Enable SPI
    SPI1_CR1 |= (1 << 6);  // SPE
}

static uint8_t spi_transfer(uint8_t data) {
    // Wait for TXE
    while (!(SPI1_SR & (1 << 1)));
    SPI1_DR = data;
    // Wait for RXNE
    while (!(SPI1_SR & (1 << 0)));
    return (uint8_t)SPI1_DR;
}

static void gyro_cs_low(void)  { GPIOB_ODR &= ~BMI088_GYRO_CS_PIN; }
static void gyro_cs_high(void) { GPIOB_ODR |= BMI088_GYRO_CS_PIN; }

static uint8_t gyro_read_reg(uint8_t reg) {
    gyro_cs_low();
    spi_transfer(reg | 0x80);  // Read bit
    uint8_t val = spi_transfer(0x00);
    gyro_cs_high();
    return val;
}

static void gyro_write_reg(uint8_t reg, uint8_t val) {
    gyro_cs_low();
    spi_transfer(reg & 0x7F);  // Write bit
    spi_transfer(val);
    gyro_cs_high();
}

static bool gyro_init(void) {
    spi_init();

    // Small delay
    delay_ms(10);

    // Soft reset
    gyro_write_reg(BMI088_GYRO_SOFTRESET, 0xB6);
    delay_ms(50);

    // Check chip ID (should be 0x0F for BMI088 gyro)
    uint8_t chip_id = gyro_read_reg(BMI088_GYRO_CHIP_ID);
    if (chip_id != 0x0F) {
        return false;
    }

    // Set range: 0x00 = 2000 dps
    gyro_write_reg(BMI088_GYRO_RANGE, 0x00);

    // Set bandwidth: 0x02 = 116 Hz ODR, 47 Hz filter
    gyro_write_reg(BMI088_GYRO_BANDWIDTH, 0x02);

    delay_ms(10);
    return true;
}

static int16_t gyro_read_z(void) {
    uint8_t lsb = gyro_read_reg(BMI088_GYRO_RATE_Z_LSB);
    uint8_t msb = gyro_read_reg(BMI088_GYRO_RATE_Z_LSB + 1);
    return (int16_t)((msb << 8) | lsb);
}

// ============================================================================
// Test Functions
// ============================================================================

static void test_single_motor(int channel) {
    motor_info_t *info = &g_motors[channel];
    info->channel = channel;
    info->gyro_z_sum = 0;
    info->rotation = 0;

    // Blink to indicate which motor (1-4 blinks)
    blink_n(channel + 1, 200, 200);
    delay_ms(500);

    // Spin up motor
    motor_set(channel, TEST_SPEED);

    // Wait for motor to reach speed
    delay_ms(300);

    // Sample gyro while spinning
    int sample_interval = (SPIN_DURATION_MS - 600) / GYRO_SAMPLES;
    for (int i = 0; i < GYRO_SAMPLES; i++) {
        info->gyro_z_sum += gyro_read_z();
        led_toggle();
        delay_ms(sample_interval);
    }
    led_off();

    // Stop motor
    motor_set(channel, 0);
    delay_ms(500);

    // Analyze rotation direction from gyro
    // BMI088 gyro: positive Z = CCW (right-hand rule, Z up)
    int avg_gyro = info->gyro_z_sum / GYRO_SAMPLES;

    // Threshold for detection (raw value, ~61 LSB/dps at 2000dps range)
    // 5000 raw ~= 82 dps
    if (avg_gyro > 300) {
        info->rotation = -1;  // CCW (positive gyro = drone rotating CCW)
        blink_n(1, 400, 400);  // 1 slow blink = CCW
    } else if (avg_gyro < -300) {
        info->rotation = 1;   // CW
        blink_n(2, 400, 400);  // 2 slow blinks = CW
    } else {
        info->rotation = 0;   // Unclear
        blink_n(3, 400, 400);  // 3 slow blinks = unclear
    }

    delay_ms(500);
}

static void test_all_motors(void) {
    // 10 fast blinks = all motors test
    blink_n(10, 50, 50);
    delay_ms(500);

    // Start all motors
    motor_set(0, TEST_SPEED);
    motor_set(1, TEST_SPEED);
    motor_set(2, TEST_SPEED);
    motor_set(3, TEST_SPEED);

    delay_ms(300);

    // Sample gyro
    int gyro_sum = 0;
    for (int i = 0; i < 30; i++) {
        gyro_sum += gyro_read_z();
        led_toggle();
        delay_ms(50);
    }
    led_off();

    motors_stop_all();

    int avg = gyro_sum / 30;
    delay_ms(500);

    // Report result:
    // 1 slow blink = balanced (good!)
    // 2 slow blinks = rotating CCW
    // 3 slow blinks = rotating CW
    if (avg > -200 && avg < 200) {
        blink_n(1, 500, 500);  // Balanced
    } else if (avg > 0) {
        blink_n(2, 500, 500);  // Rotating CCW
    } else {
        blink_n(3, 500, 500);  // Rotating CW
    }

    delay_ms(500);
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

    // Initialize motors
    if (!motors_init()) {
        // Error: continuous fast blink
        while (1) { led_toggle(); delay_ms(100); }
    }

    // 3 quick blinks = Motors OK
    blink_n(3, 100, 100);
    delay_ms(500);

    // Initialize gyroscope
    if (!gyro_init()) {
        // Error: continuous medium blink
        while (1) { led_toggle(); delay_ms(250); }
    }

    // 4 quick blinks = Gyro OK
    blink_n(4, 100, 100);
    delay_ms(1000);

    // Test each motor individually
    for (int ch = 0; ch < 4; ch++) {
        test_single_motor(ch);
    }

    // Test all motors together
    test_all_motors();

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
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;
    main();
    while (1);
}

void Default_Handler(void) { while (1); }

void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector")))
const uint32_t g_vectors[] = {
    (uint32_t)&_estack,
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    (uint32_t)MemManage_Handler,
    (uint32_t)BusFault_Handler,
    (uint32_t)UsageFault_Handler,
    0, 0, 0, 0,
    (uint32_t)SVC_Handler,
    (uint32_t)DebugMon_Handler,
    0,
    (uint32_t)PendSV_Handler,
    (uint32_t)SysTick_Handler,
};

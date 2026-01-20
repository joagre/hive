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
 * BMI088: I2C3 (PA8=SCL, PC9=SDA), Gyro addr=0x68
 */

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Test Configuration
// ============================================================================

#define TEST_SPEED 40         // ~16% duty cycle - enough to spin but safe
#define SPIN_DURATION_MS 2000 // How long to spin each motor
#define GYRO_SAMPLES 50       // Gyro samples during spin

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
#define GPIOA_AFR0 (*(volatile uint32_t *)(GPIOA_BASE + 0x20))
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

// TIM2
#define TIM2_BASE (APB1PERIPH_BASE + 0x0000UL)
#define TIM2_CR1 (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_CCMR1 (*(volatile uint32_t *)(TIM2_BASE + 0x18))
#define TIM2_CCMR2 (*(volatile uint32_t *)(TIM2_BASE + 0x1C))
#define TIM2_CCER (*(volatile uint32_t *)(TIM2_BASE + 0x20))
#define TIM2_PSC (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR (*(volatile uint32_t *)(TIM2_BASE + 0x2C))
#define TIM2_CCR1 (*(volatile uint32_t *)(TIM2_BASE + 0x34))
#define TIM2_CCR2 (*(volatile uint32_t *)(TIM2_BASE + 0x38))
#define TIM2_CCR3 (*(volatile uint32_t *)(TIM2_BASE + 0x3C))
#define TIM2_CCR4 (*(volatile uint32_t *)(TIM2_BASE + 0x40))
#define TIM2_EGR (*(volatile uint32_t *)(TIM2_BASE + 0x14))

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

// PWM configuration
#define PWM_PRESCALER 0
#define PWM_PERIOD 255

// LED Blue (PD2) - directly on STM32, active high
#define LED_PIN (1 << 2)

// BMI088 I2C address (gyroscope)
#define BMI088_GYRO_I2C_ADDR 0x68

// BMI088 registers
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_RATE_Z_LSB 0x06
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_SOFTRESET 0x14

// ============================================================================
// Global State
// ============================================================================

static volatile uint32_t g_ticks = 0;

typedef struct {
    int channel;
    int rotation; // +1=CW, -1=CCW, 0=unknown
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
    // Enable GPIO clocks (GPIOA for motors/I2C, GPIOC for I2C SDA, GPIOD for LED)
    RCC_AHB1ENR |= (1 << 0) | (1 << 2) | (1 << 3); // GPIOA, GPIOC, GPIOD
    for (volatile int i = 0; i < 100; i++)
        ;

    // Configure PD2 as output (Blue LED)
    GPIOD_MODER &= ~(3 << 4);  // Clear bits for pin 2
    GPIOD_MODER |= (1 << 4);   // Output mode
    GPIOD_OSPEEDR |= (3 << 4); // High speed
    GPIOD_ODR &= ~LED_PIN;     // LED off
}

static bool motors_init(void) {
    // Enable TIM2 clock
    RCC_APB1ENR |= (1 << 0);
    for (volatile int i = 0; i < 100; i++)
        ;

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
    if (speed > PWM_PERIOD)
        speed = PWM_PERIOD;
    switch (channel) {
    case 0:
        TIM2_CCR1 = speed;
        break;
    case 1:
        TIM2_CCR2 = speed;
        break;
    case 2:
        TIM2_CCR3 = speed;
        break;
    case 3:
        TIM2_CCR4 = speed;
        break;
    }
}

static void motors_stop_all(void) {
    TIM2_CCR1 = 0;
    TIM2_CCR2 = 0;
    TIM2_CCR3 = 0;
    TIM2_CCR4 = 0;
}

// ============================================================================
// I2C Functions (I2C3: PA8=SCL, PC9=SDA)
// ============================================================================

static void i2c_init(void) {
    // Enable I2C3 clock
    RCC_APB1ENR |= (1 << 23);
    for (volatile int i = 0; i < 100; i++)
        ;

    // Configure PA8 as AF4 (I2C3_SCL), open-drain, pull-up
    GPIOA_MODER &= ~(3 << 16);
    GPIOA_MODER |= (2 << 16);   // AF mode
    GPIOA_OTYPER |= (1 << 8);   // Open-drain
    GPIOA_OSPEEDR |= (3 << 16); // High speed
    GPIOA_PUPDR &= ~(3 << 16);
    GPIOA_PUPDR |= (1 << 16); // Pull-up
    GPIOA_AFR1 &= ~(0xF << 0);
    GPIOA_AFR1 |= (4 << 0); // AF4 = I2C3

    // Configure PC9 as AF4 (I2C3_SDA), open-drain, pull-up
    GPIOC_MODER &= ~(3 << 18);
    GPIOC_MODER |= (2 << 18);   // AF mode
    GPIOC_OTYPER |= (1 << 9);   // Open-drain
    GPIOC_OSPEEDR |= (3 << 18); // High speed
    GPIOC_PUPDR &= ~(3 << 18);
    GPIOC_PUPDR |= (1 << 18); // Pull-up
    GPIOC_AFR1 &= ~(0xF << 4);
    GPIOC_AFR1 |= (4 << 4); // AF4 = I2C3

    // Reset I2C3
    I2C3_CR1 = (1 << 15); // SWRST
    I2C3_CR1 = 0;

    // Configure I2C3: 400 kHz (APB1 = 42 MHz)
    I2C3_CR2 = 42;       // FREQ = 42 MHz
    I2C3_CCR = 35;       // CCR for 400 kHz fast mode
    I2C3_TRISE = 13;     // Maximum rise time
    I2C3_CR1 = (1 << 0); // PE = enable I2C
}

static bool i2c_start(void) {
    I2C3_CR1 |= (1 << 8); // START
    uint32_t timeout = 10000;
    while (!(I2C3_SR1 & (1 << 0)) && timeout--)
        ;
    return timeout > 0;
}

static bool i2c_send_addr(uint8_t addr, bool read) {
    I2C3_DR = (addr << 1) | (read ? 1 : 0);
    uint32_t timeout = 10000;
    while (!(I2C3_SR1 & (1 << 1)) && timeout--)
        ;
    if (timeout == 0)
        return false;
    (void)I2C3_SR2; // Clear ADDR by reading SR2
    return true;
}

static void i2c_stop(void) {
    I2C3_CR1 |= (1 << 9); // STOP
}

static bool i2c_write_byte(uint8_t data) {
    uint32_t timeout = 10000;
    while (!(I2C3_SR1 & (1 << 7)) && timeout--)
        ; // TXE
    if (timeout == 0)
        return false;
    I2C3_DR = data;
    timeout = 10000;
    while (!(I2C3_SR1 & (1 << 2)) && timeout--)
        ; // BTF
    return timeout > 0;
}

static uint8_t i2c_read_byte(bool ack) {
    if (ack) {
        I2C3_CR1 |= (1 << 10); // ACK
    } else {
        I2C3_CR1 &= ~(1 << 10); // NACK
    }
    uint32_t timeout = 10000;
    while (!(I2C3_SR1 & (1 << 6)) && timeout--)
        ; // RXNE
    return I2C3_DR;
}

static uint8_t gyro_read_reg(uint8_t reg) {
    if (!i2c_start())
        return 0;
    if (!i2c_send_addr(BMI088_GYRO_I2C_ADDR, false)) {
        i2c_stop();
        return 0;
    }
    if (!i2c_write_byte(reg)) {
        i2c_stop();
        return 0;
    }

    // Repeated start for read
    if (!i2c_start())
        return 0;
    if (!i2c_send_addr(BMI088_GYRO_I2C_ADDR, true)) {
        i2c_stop();
        return 0;
    }

    uint8_t val = i2c_read_byte(false); // NACK for single byte
    i2c_stop();
    return val;
}

static bool gyro_write_reg(uint8_t reg, uint8_t val) {
    if (!i2c_start())
        return false;
    if (!i2c_send_addr(BMI088_GYRO_I2C_ADDR, false)) {
        i2c_stop();
        return false;
    }
    if (!i2c_write_byte(reg)) {
        i2c_stop();
        return false;
    }
    if (!i2c_write_byte(val)) {
        i2c_stop();
        return false;
    }
    i2c_stop();
    return true;
}

static bool gyro_init(void) {
    i2c_init();

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
        blink_n(1, 400, 400); // 1 slow blink = CCW
    } else if (avg_gyro < -300) {
        info->rotation = 1;   // CW
        blink_n(2, 400, 400); // 2 slow blinks = CW
    } else {
        info->rotation = 0;   // Unclear
        blink_n(3, 400, 400); // 3 slow blinks = unclear
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
        blink_n(1, 500, 500); // Balanced
    } else if (avg > 0) {
        blink_n(2, 500, 500); // Rotating CCW
    } else {
        blink_n(3, 500, 500); // Rotating CW
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
        while (1) {
            led_toggle();
            delay_ms(100);
        }
    }

    // 3 quick blinks = Motors OK
    blink_n(3, 100, 100);
    delay_ms(500);

    // Initialize gyroscope
    if (!gyro_init()) {
        // Error: continuous medium blink
        while (1) {
            led_toggle();
            delay_ms(250);
        }
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

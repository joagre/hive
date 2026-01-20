// Crazyflie 2.1+ Platform Layer Implementation
//
// Implements the platform interface using direct STM32F405 peripheral access
// and the sensor drivers (BMI088, BMP388, PMW3901, VL53L1x).

#include "platform_crazyflie.h"
#include "stm32f4xx.h"
#include "bmi088.h"
#include "bmp388.h"
#include "pmw3901.h"
#include "vl53l1x.h"
#include "motors.h"

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
#define LED_PIN GPIO_ODR_OD4 // PC4 blue LED on Crazyflie
#define LED_PORT GPIOC

// SPI pins for BMI088 (SPI1)
#define SPI1_SCK_PIN 5       // PA5
#define SPI1_MISO_PIN 6      // PA6
#define SPI1_MOSI_PIN 7      // PA7
#define BMI088_ACC_CS_PIN 1  // PB1
#define BMI088_GYRO_CS_PIN 0 // PB0

// I2C pins (I2C3)
#define I2C3_SCL_PIN 8 // PA8
#define I2C3_SDA_PIN 9 // PC9

// SPI pins for PMW3901 on Flow deck (directly on expansion connector)
#define FLOW_SPI_CS_PIN 12 // PB12 (expansion deck)

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

// System tick counter
static volatile uint32_t s_sys_tick_ms = 0;

// ----------------------------------------------------------------------------
// SysTick Handler
// ----------------------------------------------------------------------------

void SysTick_Handler(void) {
    s_sys_tick_ms++;
}

// ----------------------------------------------------------------------------
// Low-Level Platform Functions
// ----------------------------------------------------------------------------

static void system_clock_init(void) {
    // Configure flash latency for 168 MHz
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |
                 FLASH_ACR_DCEN;

    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    // Configure PLL: HSE (8MHz) * 336 / 2 = 168 MHz
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) |   // PLLM = 8
                   (336 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 336
                   (0 << RCC_PLLCFGR_PLLP_Pos) |   // PLLP = 2 (0 = /2)
                   RCC_PLLCFGR_PLLSRC_HSE |        // HSE as source
                   (7 << RCC_PLLCFGR_PLLQ_Pos);    // PLLQ = 7 (for USB)

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Configure AHB, APB1, APB2 prescalers
    // AHB = 168 MHz, APB1 = 42 MHz, APB2 = 84 MHz
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 |  // AHB = SYSCLK
                RCC_CFGR_PPRE1_DIV4 | // APB1 = AHB/4
                RCC_CFGR_PPRE2_DIV2;  // APB2 = AHB/2

    // Switch to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    // Update SystemCoreClock
    SystemCoreClock = 168000000;
}

static void systick_init(void) {
    // Configure SysTick for 1ms interrupts
    SysTick_Config(SystemCoreClock / 1000);
}

static void gpio_init(void) {
    // Enable GPIO clocks
    RCC->AHB1ENR |=
        RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // LED (PC4) as output
    LED_PORT->MODER &= ~GPIO_MODER_MODER4;
    LED_PORT->MODER |= GPIO_MODER_MODER4_0;      // Output mode
    LED_PORT->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4; // High speed
    LED_PORT->ODR &= ~LED_PIN;                   // LED off
}

// ----------------------------------------------------------------------------
// SPI Interface for BMI088
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

    // Configure CS pins (PB0, PB1) as outputs
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1);
    GPIOB->ODR |=
        (1 << BMI088_ACC_CS_PIN) | (1 << BMI088_GYRO_CS_PIN); // CS high

    // Configure SPI1: Master, 8-bit, CPOL=0, CPHA=0, ~10 MHz (84/8)
    SPI1->CR1 = SPI_CR1_MSTR | // Master mode
                SPI_CR1_BR_1 | // Baud rate = fPCLK/8
                SPI_CR1_SSM |  // Software slave management
                SPI_CR1_SSI;   // Internal slave select

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

// BMI088 SPI callbacks
void bmi088_acc_cs_low(void) {
    GPIOB->ODR &= ~(1 << BMI088_ACC_CS_PIN);
}
void bmi088_acc_cs_high(void) {
    GPIOB->ODR |= (1 << BMI088_ACC_CS_PIN);
}
void bmi088_gyro_cs_low(void) {
    GPIOB->ODR &= ~(1 << BMI088_GYRO_CS_PIN);
}
void bmi088_gyro_cs_high(void) {
    GPIOB->ODR |= (1 << BMI088_GYRO_CS_PIN);
}
uint8_t bmi088_spi_transfer(uint8_t data) {
    return spi1_transfer(data);
}
void bmi088_delay_us(uint32_t us) {
    platform_delay_us(us);
}
void bmi088_delay_ms(uint32_t ms) {
    platform_delay_ms(ms);
}

// ----------------------------------------------------------------------------
// I2C Interface for BMP388 and VL53L1x
// ----------------------------------------------------------------------------

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

    // Configure I2C3: 400 kHz
    // APB1 = 42 MHz
    I2C3->CR2 = 42;         // FREQ = 42 MHz
    I2C3->CCR = 35;         // CCR for 400 kHz
    I2C3->TRISE = 13;       // Maximum rise time
    I2C3->CR1 = I2C_CR1_PE; // Enable I2C
}

static bool i2c3_write(uint8_t addr, uint8_t *data, uint16_t len) {
    // Start
    I2C3->CR1 |= I2C_CR1_START;
    while (!(I2C3->SR1 & I2C_SR1_SB))
        ;

    // Address (write)
    I2C3->DR = addr << 1;
    while (!(I2C3->SR1 & I2C_SR1_ADDR))
        ;
    (void)I2C3->SR2; // Clear ADDR

    // Data
    for (uint16_t i = 0; i < len; i++) {
        while (!(I2C3->SR1 & I2C_SR1_TXE))
            ;
        I2C3->DR = data[i];
    }
    while (!(I2C3->SR1 & I2C_SR1_BTF))
        ;

    // Stop
    I2C3->CR1 |= I2C_CR1_STOP;

    return true;
}

static bool i2c3_read(uint8_t addr, uint8_t *data, uint16_t len) {
    if (len == 0)
        return false;

    // Start
    I2C3->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
    while (!(I2C3->SR1 & I2C_SR1_SB))
        ;

    // Address (read)
    I2C3->DR = (addr << 1) | 1;
    while (!(I2C3->SR1 & I2C_SR1_ADDR))
        ;
    (void)I2C3->SR2;

    // Read data
    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            I2C3->CR1 &= ~I2C_CR1_ACK; // NACK for last byte
        }
        while (!(I2C3->SR1 & I2C_SR1_RXNE))
            ;
        data[i] = I2C3->DR;
    }

    // Stop
    I2C3->CR1 |= I2C_CR1_STOP;

    return true;
}

// BMP388 I2C callbacks
bool bmp388_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    if (!i2c3_write(addr, &reg, 1))
        return false;
    return i2c3_read(addr, data, len);
}

bool bmp388_i2c_write(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t buf[len + 1];
    buf[0] = reg;
    for (uint8_t i = 0; i < len; i++)
        buf[i + 1] = data[i];
    return i2c3_write(addr, buf, len + 1);
}

void bmp388_delay_ms(uint32_t ms) {
    platform_delay_ms(ms);
}

// VL53L1x I2C callbacks (16-bit register addresses)
bool vl53l1x_i2c_read(uint8_t addr, uint16_t reg, uint8_t *data, uint16_t len) {
    uint8_t reg_buf[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    if (!i2c3_write(addr, reg_buf, 2))
        return false;
    return i2c3_read(addr, data, len);
}

bool vl53l1x_i2c_write(uint8_t addr, uint16_t reg, uint8_t *data,
                       uint16_t len) {
    uint8_t buf[len + 2];
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    for (uint16_t i = 0; i < len; i++)
        buf[i + 2] = data[i];
    return i2c3_write(addr, buf, len + 2);
}

void vl53l1x_delay_ms(uint32_t ms) {
    platform_delay_ms(ms);
}

// ----------------------------------------------------------------------------
// SPI Interface for PMW3901 (Flow deck)
// ----------------------------------------------------------------------------

// PMW3901 SPI callbacks (stubbed - need to implement when deck pinout
// confirmed)
void pmw3901_cs_low(void) {
    GPIOB->ODR &= ~(1 << FLOW_SPI_CS_PIN);
}
void pmw3901_cs_high(void) {
    GPIOB->ODR |= (1 << FLOW_SPI_CS_PIN);
}
uint8_t pmw3901_spi_transfer(uint8_t data) {
    return spi1_transfer(data);
}
void pmw3901_delay_us(uint32_t us) {
    platform_delay_us(us);
}
void pmw3901_delay_ms(uint32_t ms) {
    platform_delay_ms(ms);
}

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
// Platform Interface Implementation
// ----------------------------------------------------------------------------

int platform_init(void) {
    // Initialize system clock (168 MHz)
    system_clock_init();

    // Initialize SysTick (1ms)
    systick_init();

    // Initialize GPIO (LED)
    gpio_init();

    // 1 blink = starting
    init_blink(1, 200, 200);

    // Initialize SPI1 for BMI088
    spi1_init();

    // Initialize I2C3 for BMP388 and VL53L1x
    i2c3_init();

    // Initialize BMI088 IMU
    if (!bmi088_init(NULL)) {
        init_blink(3, 100, 100); // 3 fast blinks = IMU failed
        error_blink_forever();
    }

    // 2 blinks = IMU OK
    init_blink(2, 200, 200);

    // Initialize BMP388 barometer
    if (!bmp388_init(NULL)) {
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
    if (pmw3901_init()) {
        if (vl53l1x_init(NULL)) {
            vl53l1x_start_ranging();
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

    // Required sensors: BMI088 (IMU) and BMP388 (barometer)
    if (!bmi088_is_ready()) {
        return false;
    }
    if (!bmp388_is_ready()) {
        return false;
    }

    // Optional sensors: Flow deck (PMW3901 + VL53L1x)
    // Only test if deck was detected during init
    if (s_flow_deck_present) {
        if (!pmw3901_is_ready()) {
            return false;
        }
        if (!vl53l1x_is_ready()) {
            return false;
        }
    }

    return true;
}

int platform_calibrate(void) {
    if (!s_initialized) {
        return -1;
    }

    // Gyro bias calibration
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    bmi088_data_t gyro;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (bmi088_read_gyro(&gyro)) {
            gyro_sum[0] += gyro.x;
            gyro_sum[1] += gyro.y;
            gyro_sum[2] += gyro.z;
        }
        platform_delay_ms(2);
    }

    s_gyro_bias[0] = gyro_sum[0] / CALIBRATION_SAMPLES;
    s_gyro_bias[1] = gyro_sum[1] / CALIBRATION_SAMPLES;
    s_gyro_bias[2] = gyro_sum[2] / CALIBRATION_SAMPLES;

    // Barometer reference calibration
    float pressure_sum = 0.0f;
    bmp388_data_t baro;

    for (int i = 0; i < BARO_CALIBRATION_SAMPLES; i++) {
        if (bmp388_read(&baro)) {
            pressure_sum += baro.pressure_pa;
        }
        platform_delay_ms(20);
    }

    s_ref_pressure = pressure_sum / BARO_CALIBRATION_SAMPLES;

    s_calibrated = true;
    return 0;
}

void platform_read_sensors(sensor_data_t *sensors) {
    // IMU (BMI088)
    bmi088_data_t accel, gyro;

    if (bmi088_read_accel(&accel)) {
        sensors->accel[0] = accel.x;
        sensors->accel[1] = accel.y;
        sensors->accel[2] = accel.z;
    }

    if (bmi088_read_gyro(&gyro)) {
        sensors->gyro[0] = gyro.x - s_gyro_bias[0];
        sensors->gyro[1] = gyro.y - s_gyro_bias[1];
        sensors->gyro[2] = gyro.z - s_gyro_bias[2];
    }

    // Barometer (BMP388)
    bmp388_data_t baro;
    if (bmp388_read(&baro)) {
        sensors->pressure_hpa = baro.pressure_pa / 100.0f;
        sensors->baro_temp_c = baro.temperature_c;
        sensors->baro_valid = true;
    } else {
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
    // SysTick counts down from reload value
    uint32_t ms = s_sys_tick_ms;
    uint32_t ticks = SysTick->VAL;
    uint32_t load = SysTick->LOAD;

    // Convert remaining ticks to microseconds
    uint32_t us_in_tick = ((load - ticks) * 1000) / (load + 1);

    return ms * 1000 + us_in_tick;
}

void platform_delay_ms(uint32_t ms) {
    uint32_t start = s_sys_tick_ms;
    while ((s_sys_tick_ms - start) < ms) {
        __WFI(); // Wait for interrupt (low power)
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
    // TODO: Implement debug printf
    (void)fmt;
}

void platform_emergency_stop(void) {
    motors_emergency_stop();
    s_armed = false;

    // Fast blink LED
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
    return pmw3901_read_delta(delta_x, delta_y);
}

bool platform_read_height(uint16_t *height_mm) {
    if (!s_flow_deck_present) {
        return false;
    }

    if (!vl53l1x_data_ready()) {
        return false;
    }

    *height_mm = vl53l1x_read_distance();
    return *height_mm > 0;
}

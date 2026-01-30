// Crazyflie 2.1+ Platform Layer Implementation (v2)
//
// Uses Bitcraze-derived DMA/interrupt I2C driver for reliable sensor communication.
// Implements platform interface for BMI088 IMU and BMP388 barometer.

#include "platform.h"
#include "i2cdev.h"
#include "debug_swo.h"
#include "stm32f4xx.h"

// Vendor drivers (Bitcraze Bosch drivers)
#include "bmi088.h"
#include "bmp3.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

// CPU and SWO configuration
#define CPU_FREQ_HZ 168000000
#define SWO_BAUD 2000000

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Calibration parameters
#define CALIBRATION_SAMPLES 512
#define GYRO_VARIANCE_THRESHOLD 100.0f
#define BARO_CALIBRATION_SAMPLES 50

// Startup delay for sensor power stabilization
#define SENSOR_STARTUP_DELAY_MS 1000

// Conversion constants
#define GRAVITY 9.80665f
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// GPIO Pins
#define LED_PIN (1U << 2) // PD2 blue LED on Crazyflie
#define LED_PORT GPIOD

// I2C addresses (use Bitcraze driver defines)
// BMI088_ACCEL_I2C_ADDR_PRIMARY = 0x18
// BMI088_GYRO_I2C_ADDR_SECONDARY = 0x69
// BMP3_I2C_ADDR_SEC = 0x77

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;
static bool s_calibrated = false;
static bool s_armed = false;

// Gyro bias (rad/s)
static float s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// Accelerometer scale factor
static float s_accel_scale = 1.0f;

// Barometer reference pressure (Pa)
static float s_ref_pressure = 0.0f;

// System tick counter
static volatile uint32_t s_sys_tick_ms = 0;

// Vendor driver device handles (Bitcraze Bosch drivers)
static struct bmi088_dev s_bmi088_dev;
static struct bmp3_dev s_bmp3_dev;

// ----------------------------------------------------------------------------
// SysTick Handler
// ----------------------------------------------------------------------------

void SysTick_Handler(void) {
    s_sys_tick_ms++;
}

// ----------------------------------------------------------------------------
// Low-Level Initialization
// ----------------------------------------------------------------------------

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
    LED_PORT->MODER |= GPIO_MODER_MODER2_0;
    LED_PORT->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;
    LED_PORT->ODR &= ~LED_PIN;
}

// ----------------------------------------------------------------------------
// Bosch Driver I2C Callbacks (Bitcraze API - using i2cdev)
// ----------------------------------------------------------------------------

// BMI088 callback: int8_t (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
static int8_t bmi088_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                              uint16_t len) {
    return i2cdevReadReg8(I2C3_DEV, dev_addr, reg_addr, len, data) ? 0 : -1;
}

static int8_t bmi088_i2c_write(uint8_t dev_addr, uint8_t reg_addr,
                               uint8_t *data, uint16_t len) {
    return i2cdevWriteReg8(I2C3_DEV, dev_addr, reg_addr, len, data) ? 0 : -1;
}

// BMI088 delay callback: void (uint32_t period_ms)
static void bmi088_delay_ms(uint32_t period) {
    platform_delay_ms(period);
}

// BMP3 callback: int8_t (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
static int8_t bmp3_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
                            uint16_t len) {
    return i2cdevReadReg8(I2C3_DEV, dev_id, reg_addr, len, data) ? 0 : -1;
}

static int8_t bmp3_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
                             uint16_t len) {
    return i2cdevWriteReg8(I2C3_DEV, dev_id, reg_addr, len, data) ? 0 : -1;
}

// BMP3 delay callback: void (uint32_t period_ms)
static void bmp3_delay_ms(uint32_t period) {
    platform_delay_ms(period);
}

// ----------------------------------------------------------------------------
// Sensor Initialization
// ----------------------------------------------------------------------------

static bool bmi088_init(void) {
    uint16_t rslt;

    // Setup BMI088 device structure (Bitcraze API)
    s_bmi088_dev.accel_id = BMI088_ACCEL_I2C_ADDR_PRIMARY; // 0x18
    s_bmi088_dev.gyro_id = BMI088_GYRO_I2C_ADDR_SECONDARY; // 0x69
    s_bmi088_dev.interface = BMI088_I2C_INTF;
    s_bmi088_dev.read = bmi088_i2c_read;
    s_bmi088_dev.write = bmi088_i2c_write;
    s_bmi088_dev.delay_ms = bmi088_delay_ms;

    // Initialize gyroscope first (Bitcraze order)
    // Gyro starts in normal mode by default
    rslt = bmi088_gyro_init(&s_bmi088_dev);
    if (rslt != BMI088_OK) {
        debug_swo_printf("  BMI088 gyro init failed: %u\n", rslt);
        return false;
    }
    debug_swo_printf("  BMI088: gyro_id=0x%02X\n", s_bmi088_dev.gyro_chip_id);

    // IMPORTANT: Power on accelerometer before init (starts in suspend mode)
    rslt =
        bmi088_accel_switch_control(&s_bmi088_dev, BMI088_ACCEL_POWER_ENABLE);
    if (rslt != BMI088_OK) {
        debug_swo_printf("  BMI088 accel power-on failed: %u\n", rslt);
        return false;
    }
    platform_delay_ms(5);

    // Initialize accelerometer (now powered on)
    rslt = bmi088_accel_init(&s_bmi088_dev);
    if (rslt != BMI088_OK) {
        debug_swo_printf("  BMI088 accel init failed: %u\n", rslt);
        return false;
    }
    debug_swo_printf("  BMI088: accel_id=0x%02X\n", s_bmi088_dev.accel_chip_id);

    // Configure accelerometer
    s_bmi088_dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
    s_bmi088_dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
    s_bmi088_dev.accel_cfg.bw = BMI088_ACCEL_BW_OSR4;
    s_bmi088_dev.accel_cfg.power = BMI088_ACCEL_PM_ACTIVE;

    rslt = bmi088_set_accel_power_mode(&s_bmi088_dev);
    if (rslt != BMI088_OK)
        return false;

    platform_delay_ms(10);

    rslt = bmi088_set_accel_meas_conf(&s_bmi088_dev);
    if (rslt != BMI088_OK)
        return false;

    // Configure gyroscope
    s_bmi088_dev.gyro_cfg.odr = BMI088_GYRO_BW_116_ODR_1000_HZ;
    s_bmi088_dev.gyro_cfg.range = BMI088_GYRO_RANGE_2000_DPS;
    s_bmi088_dev.gyro_cfg.bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
    s_bmi088_dev.gyro_cfg.power = BMI088_GYRO_PM_NORMAL;

    rslt = bmi088_set_gyro_power_mode(&s_bmi088_dev);
    if (rslt != BMI088_OK)
        return false;

    platform_delay_ms(30);

    rslt = bmi088_set_gyro_meas_conf(&s_bmi088_dev);
    if (rslt != BMI088_OK)
        return false;

    return true;
}

static bool bmp388_init(void) {
    int8_t rslt;

    // Setup BMP3 device structure (Bitcraze API)
    s_bmp3_dev.dev_id = BMP3_I2C_ADDR_SEC; // 0x77
    s_bmp3_dev.intf = BMP3_I2C_INTF;
    s_bmp3_dev.read = bmp3_i2c_read;
    s_bmp3_dev.write = bmp3_i2c_write;
    s_bmp3_dev.delay_ms = bmp3_delay_ms;

    // BMP388 often doesn't work first time (Bitcraze comment), so retry
    int retries = 3;
    do {
        platform_delay_ms(1);
        rslt = bmp3_init(&s_bmp3_dev);
    } while (rslt != BMP3_OK && --retries > 0);

    if (rslt != BMP3_OK) {
        debug_swo_printf("  BMP388 init failed: %d\n", rslt);
        return false;
    }
    debug_swo_printf("  BMP388: chip_id=0x%02X\n", s_bmp3_dev.chip_id);

    platform_delay_ms(20);

    // Configure settings using device's settings structure
    // Note: ODR must be low enough to allow OSR measurement time
    // Press 4X + Temp 1X at 25Hz gives ~21ms measurement in 40ms period
    s_bmp3_dev.settings.press_en = BMP3_ENABLE;
    s_bmp3_dev.settings.temp_en = BMP3_ENABLE;
    s_bmp3_dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    s_bmp3_dev.settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    s_bmp3_dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    s_bmp3_dev.settings.odr_filter.odr = BMP3_ODR_25_HZ;

    uint32_t settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL |
                            BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL |
                            BMP3_IIR_FILTER_SEL | BMP3_ODR_SEL;

    rslt = bmp3_set_sensor_settings(settings_sel, &s_bmp3_dev);
    if (rslt != BMP3_OK) {
        debug_swo_printf("  BMP388 config failed: %d\n", rslt);
        return false;
    }

    // Set to normal mode
    s_bmp3_dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(&s_bmp3_dev);
    if (rslt != BMP3_OK) {
        debug_swo_printf("  BMP388 mode failed: %d\n", rslt);
        return false;
    }

    platform_delay_ms(20);

    return true;
}

// ----------------------------------------------------------------------------
// Motor Control (PWM on TIM2 + TIM4)
// Crazyflie 2.1+ motor pin mapping (from Bitcraze motors_def.c):
//   M1: PA1,  TIM2_CH2
//   M2: PB11, TIM2_CH4
//   M3: PA15, TIM2_CH1
//   M4: PB9,  TIM4_CH4
// ----------------------------------------------------------------------------

static void motors_init(void) {
    // Enable TIM2 and TIM4 clocks
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;

    // M1: PA1 as AF1 (TIM2_CH2)
    // AFR[0] pin1 = bits[7:4]
    GPIOA->MODER &= ~GPIO_MODER_MODER1;
    GPIOA->MODER |= GPIO_MODER_MODER1_1; // AF mode
    GPIOA->AFR[0] &= ~(0xFU << 4);       // Clear AF for PA1
    GPIOA->AFR[0] |= (1U << 4);          // AF1 for PA1

    // M2: PB11 as AF1 (TIM2_CH4)
    // AFR[1] pin11 = bits[15:12]
    GPIOB->MODER &= ~GPIO_MODER_MODER11;
    GPIOB->MODER |= GPIO_MODER_MODER11_1; // AF mode
    GPIOB->AFR[1] &= ~(0xFU << 12);       // Clear AF for PB11
    GPIOB->AFR[1] |= (1U << 12);          // AF1 for PB11

    // M3: PA15 as AF1 (TIM2_CH1)
    // AFR[1] pin15 = bits[31:28]
    GPIOA->MODER &= ~GPIO_MODER_MODER15;
    GPIOA->MODER |= GPIO_MODER_MODER15_1; // AF mode
    GPIOA->AFR[1] &= ~(0xFU << 28);       // Clear AF for PA15
    GPIOA->AFR[1] |= (1U << 28);          // AF1 for PA15

    // M4: PB9 as AF2 (TIM4_CH4)
    // AFR[1] pin9 = bits[7:4]
    GPIOB->MODER &= ~GPIO_MODER_MODER9;
    GPIOB->MODER |= GPIO_MODER_MODER9_1; // AF mode
    GPIOB->AFR[1] &= ~(0xFU << 4);       // Clear AF for PB9
    GPIOB->AFR[1] |= (2U << 4);          // AF2 for PB9

    // Configure TIM2 for 328Hz PWM (Crazyflie motor ESC frequency)
    // APB1 timer clock = 84MHz (42MHz APB1 x2)
    // PSC = 999 -> 84MHz/1000 = 84kHz
    // ARR = 255 -> 84kHz/256 = 328Hz
    TIM2->PSC = 999;
    TIM2->ARR = 255;

    // PWM mode 1 on channels 1, 2, 4 (CH3 not used on TIM2)
    TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | // CH1 PWM mode 1
                  TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;  // CH2 PWM mode 1
    TIM2->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;  // CH4 PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 |= TIM_CCMR2_OC4PE;
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC4E;
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR4 = 0;
    TIM2->CR1 = TIM_CR1_CEN;

    // Configure TIM4 for 328Hz PWM (same frequency)
    TIM4->PSC = 999;
    TIM4->ARR = 255;
    TIM4->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; // CH4 PWM mode 1
    TIM4->CCMR2 |= TIM_CCMR2_OC4PE;
    TIM4->CCER = TIM_CCER_CC4E;
    TIM4->CCR4 = 0;
    TIM4->CR1 = TIM_CR1_CEN;
}

static void motors_set(float m1, float m2, float m3, float m4) {
    // Clamp to 0-1
    if (m1 < 0)
        m1 = 0;
    if (m1 > 1)
        m1 = 1;
    if (m2 < 0)
        m2 = 0;
    if (m2 > 1)
        m2 = 1;
    if (m3 < 0)
        m3 = 0;
    if (m3 > 1)
        m3 = 1;
    if (m4 < 0)
        m4 = 0;
    if (m4 > 1)
        m4 = 1;

    // Motor mapping:
    // M1: TIM2_CH2, M2: TIM2_CH4, M3: TIM2_CH1, M4: TIM4_CH4
    TIM2->CCR2 = (uint32_t)(m1 * 255); // M1
    TIM2->CCR4 = (uint32_t)(m2 * 255); // M2
    TIM2->CCR1 = (uint32_t)(m3 * 255); // M3
    TIM4->CCR4 = (uint32_t)(m4 * 255); // M4
}

// ----------------------------------------------------------------------------
// Platform Public API
// ----------------------------------------------------------------------------

int platform_init(void) {
    if (s_initialized)
        return 0;

    // Initialize debug output first
    debug_swo_init(CPU_FREQ_HZ, SWO_BAUD);
    debug_swo_printf("\n[INIT] Crazyflie 2.1+ v2 HAL starting...\n");

    // Core initialization
    systick_init();
    gpio_init();
    motors_init();

    // Wait for sensor power stabilization
    debug_swo_printf("[INIT] Waiting %dms for sensor power...\n",
                     SENSOR_STARTUP_DELAY_MS);
    platform_delay_ms(SENSOR_STARTUP_DELAY_MS);

    // Initialize I2C driver (Bitcraze DMA/interrupt based)
    debug_swo_printf("[INIT] I2C3 init (DMA-based)...\n");
    i2cdevInit(I2C3_DEV);

    // I2C bus scan - check what devices respond
    debug_swo_printf("[INIT] I2C bus scan:\n");
    uint8_t dummy;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2cdevRead(I2C3_DEV, addr, 1, &dummy)) {
            debug_swo_printf("  Found: 0x%02X\n", addr);
        }
    }

    // Debug: Test direct register read
    debug_swo_printf("[INIT] Direct register read test:\n");
    uint8_t chip_id = 0xFF;
    bool ok;

    // Try BMI088 gyro chip ID (addr 0x69, reg 0x00, expected 0x0F)
    ok = i2cdevReadByte(I2C3_DEV, 0x69, 0x00, &chip_id);
    debug_swo_printf("  Gyro 0x69 reg 0x00: ok=%d val=0x%02X (exp 0x0F)\n", ok,
                     chip_id);

    // Try BMI088 accel chip ID (addr 0x18, reg 0x00, expected 0x1E)
    chip_id = 0xFF;
    ok = i2cdevReadByte(I2C3_DEV, 0x18, 0x00, &chip_id);
    debug_swo_printf("  Accel 0x18 reg 0x00: ok=%d val=0x%02X (exp 0x1E)\n", ok,
                     chip_id);

    // Try BMP388 chip ID (addr 0x77, reg 0x00, expected 0x50)
    chip_id = 0xFF;
    ok = i2cdevReadByte(I2C3_DEV, 0x77, 0x00, &chip_id);
    debug_swo_printf("  Baro 0x77 reg 0x00: ok=%d val=0x%02X (exp 0x50)\n", ok,
                     chip_id);

    // Initialize sensors
    debug_swo_printf("[INIT] BMI088 init...\n");
    if (!bmi088_init()) {
        debug_swo_printf("[INIT] BMI088 init FAILED\n");
        return -1;
    }

    debug_swo_printf("[INIT] BMP388 init...\n");
    if (!bmp388_init()) {
        debug_swo_printf("[INIT] BMP388 init FAILED\n");
        return -1;
    }

    s_initialized = true;
    debug_swo_printf("[INIT] Platform init complete\n");
    return 0;
}

bool platform_self_test(void) {
    debug_swo_printf("[TEST] Self-test starting...\n");

    // Read a few samples to verify sensors work
    struct bmi088_sensor_data accel_data;
    struct bmi088_sensor_data gyro_data;
    struct bmp3_data baro_data;

    // Test accelerometer
    if (bmi088_get_accel_data(&accel_data, &s_bmi088_dev) != BMI088_OK) {
        debug_swo_printf("[TEST] Accel read FAILED\n");
        return false;
    }
    debug_swo_printf("[TEST] Accel: x=%d y=%d z=%d\n", accel_data.x,
                     accel_data.y, accel_data.z);

    // Test gyroscope
    if (bmi088_get_gyro_data(&gyro_data, &s_bmi088_dev) != BMI088_OK) {
        debug_swo_printf("[TEST] Gyro read FAILED\n");
        return false;
    }
    debug_swo_printf("[TEST] Gyro: x=%d y=%d z=%d\n", gyro_data.x, gyro_data.y,
                     gyro_data.z);

    // Test barometer
    if (bmp3_get_sensor_data(BMP3_ALL, &baro_data, &s_bmp3_dev) != BMP3_OK) {
        debug_swo_printf("[TEST] Baro read FAILED\n");
        return false;
    }
    debug_swo_printf("[TEST] Baro: P=%.1f Pa T=%.2f C\n", baro_data.pressure,
                     baro_data.temperature);

    debug_swo_printf("[TEST] Self-test PASSED\n");
    return true;
}

int platform_calibrate(void) {
    debug_swo_printf("[CAL] Calibration starting...\n");

    // Gyro bias calibration
    float gyro_sum[3] = {0, 0, 0};
    struct bmi088_sensor_data gyro_data;
    const float gyro_scale = (2000.0f / 32768.0f) * (M_PI / 180.0f);

    debug_swo_printf("[CAL] Gyro calibration (%d samples)...\n",
                     CALIBRATION_SAMPLES);

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (bmi088_get_gyro_data(&gyro_data, &s_bmi088_dev) == BMI088_OK) {
            gyro_sum[0] += gyro_data.x * gyro_scale;
            gyro_sum[1] += gyro_data.y * gyro_scale;
            gyro_sum[2] += gyro_data.z * gyro_scale;
        }
        if (i % 50 == 0) {
            platform_led_toggle();
        }
        platform_delay_ms(1);
    }

    s_gyro_bias[0] = gyro_sum[0] / CALIBRATION_SAMPLES;
    s_gyro_bias[1] = gyro_sum[1] / CALIBRATION_SAMPLES;
    s_gyro_bias[2] = gyro_sum[2] / CALIBRATION_SAMPLES;

    debug_swo_printf("[CAL] Gyro bias: x=%.4f y=%.4f z=%.4f rad/s\n",
                     (double)s_gyro_bias[0], (double)s_gyro_bias[1],
                     (double)s_gyro_bias[2]);

    // Barometer reference calibration
    float pressure_sum = 0;
    struct bmp3_data baro_data;

    debug_swo_printf("[CAL] Baro calibration (%d samples)...\n",
                     BARO_CALIBRATION_SAMPLES);

    for (int i = 0; i < BARO_CALIBRATION_SAMPLES; i++) {
        if (bmp3_get_sensor_data(BMP3_ALL, &baro_data, &s_bmp3_dev) ==
            BMP3_OK) {
            pressure_sum += baro_data.pressure;
        }
        if (i % 5 == 0) {
            platform_led_toggle();
        }
        platform_delay_ms(20);
    }

    s_ref_pressure = pressure_sum / BARO_CALIBRATION_SAMPLES;
    debug_swo_printf("[CAL] Baro reference: %.1f Pa\n", (double)s_ref_pressure);

    // Verify sensors still work after calibration
    debug_swo_printf("[CAL] Post-calibration verify...\n");
    struct bmi088_sensor_data accel_data;
    for (int i = 0; i < 5; i++) {
        bool ok = true;
        if (bmi088_get_accel_data(&accel_data, &s_bmi088_dev) != BMI088_OK)
            ok = false;
        if (bmi088_get_gyro_data(&gyro_data, &s_bmi088_dev) != BMI088_OK)
            ok = false;
        if (bmp3_get_sensor_data(BMP3_ALL, &baro_data, &s_bmp3_dev) != BMP3_OK)
            ok = false;

        if (!ok) {
            debug_swo_printf("[CAL] Verify %d FAILED\n", i);
        }
        platform_delay_ms(10);
    }

    s_calibrated = true;
    platform_led_off();
    debug_swo_printf("[CAL] Calibration complete\n");
    return 0;
}

void platform_read_sensors(sensor_data_t *sensors) {
    struct bmi088_sensor_data accel_data;
    struct bmi088_sensor_data gyro_data;
    struct bmp3_data baro_data;

    // Scaling factors
    const float accel_scale = (24.0f * GRAVITY / 32768.0f) * s_accel_scale;
    const float gyro_scale = (2000.0f / 32768.0f) * (M_PI / 180.0f);

    // Read accelerometer
    if (bmi088_get_accel_data(&accel_data, &s_bmi088_dev) == BMI088_OK) {
        sensors->accel[0] = accel_data.x * accel_scale;
        sensors->accel[1] = accel_data.y * accel_scale;
        sensors->accel[2] = accel_data.z * accel_scale;
    }

    // Read gyroscope (with bias correction)
    if (bmi088_get_gyro_data(&gyro_data, &s_bmi088_dev) == BMI088_OK) {
        sensors->gyro[0] = gyro_data.x * gyro_scale - s_gyro_bias[0];
        sensors->gyro[1] = gyro_data.y * gyro_scale - s_gyro_bias[1];
        sensors->gyro[2] = gyro_data.z * gyro_scale - s_gyro_bias[2];
    }

    // Read barometer
    if (bmp3_get_sensor_data(BMP3_ALL, &baro_data, &s_bmp3_dev) == BMP3_OK) {
        sensors->pressure_hpa = baro_data.pressure / 100.0f; // Pa to hPa
        sensors->baro_temp_c = baro_data.temperature;
        sensors->baro_valid = true;
        // Calculate altitude from pressure difference
        if (s_ref_pressure > 0) {
            sensors->baro_altitude =
                44330.0f * (1.0f - powf(baro_data.pressure / s_ref_pressure,
                                        1.0f / 5.255f));
        }
    } else {
        sensors->baro_valid = false;
    }

    // Flow deck and GPS not implemented in v2 yet
    sensors->gps_valid = false;
    sensors->velocity_valid = false;
    sensors->mag_valid = false;
}

void platform_write_motors(const motor_cmd_t *cmd) {
    if (!s_armed) {
        motors_set(0, 0, 0, 0);
        return;
    }
    motors_set(cmd->motor[0], cmd->motor[1], cmd->motor[2], cmd->motor[3]);
}

void platform_arm(void) {
    s_armed = true;
    debug_swo_printf("[MOTOR] Armed\n");
}

void platform_disarm(void) {
    s_armed = false;
    motors_set(0, 0, 0, 0);
    debug_swo_printf("[MOTOR] Disarmed\n");
}

void platform_emergency_stop(void) {
    s_armed = false;
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
}

uint32_t platform_get_time_ms(void) {
    return s_sys_tick_ms;
}

uint32_t platform_get_time_us(void) {
    uint32_t ms = s_sys_tick_ms;
    uint32_t ticks = SysTick->VAL;
    uint32_t load = SysTick->LOAD;
    uint32_t us_in_tick = ((load - ticks) * 1000) / load;
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

void platform_led_on(void) {
    LED_PORT->ODR |= LED_PIN;
}

void platform_led_off(void) {
    LED_PORT->ODR &= ~LED_PIN;
}

void platform_led_toggle(void) {
    LED_PORT->ODR ^= LED_PIN;
}

bool platform_has_flow_deck(void) {
    return false; // Not implemented in v2 yet
}

bool platform_read_flow(int16_t *delta_x, int16_t *delta_y) {
    (void)delta_x;
    (void)delta_y;
    return false;
}

bool platform_read_height(uint16_t *height_mm) {
    (void)height_mm;
    return false;
}

void platform_debug_init(void) {
    debug_swo_init(CPU_FREQ_HZ, SWO_BAUD);
}

void platform_debug_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    // Use a local buffer for formatting
    char buf[128];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    debug_swo_printf("%s", buf);
}

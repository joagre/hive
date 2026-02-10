// Crazyflie 2.1+ Platform Layer Implementation
//
// Uses Bitcraze-derived DMA/interrupt I2C driver for reliable sensor communication.
// Implements platform interface for BMI088 IMU and BMP388 barometer.

#include "platform.h"
#include "hal_config.h"
#include "i2cdev.h"
#include "debug_swo.h"
#include "stm32f4xx.h"

// Vendor drivers (Bitcraze Bosch drivers)
#include "bmi088.h"
#include "bmp3.h"

// Flow deck drivers
#include "pmw3901.h"
#include "vl53l1x_uld.h"

#include <stdbool.h>
#include <stdarg.h>
#include <math.h>

// CPU and SWO configuration
#define CPU_FREQ_HZ 168000000
#define SWO_BAUD 2000000

// Debug tracing - set to 1 to enable I2C bus scan and register tests during init
#define PLATFORM_DEBUG_INIT 0

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Calibration parameters
#define CALIBRATION_SAMPLES 512
#define BARO_CALIBRATION_SAMPLES 50
// Gyro variance threshold in (rad/s)^2 - ~0.01 rad/s std dev max for stillness
#define GYRO_VARIANCE_THRESHOLD 0.0001f

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

// Flow deck pins
#define FLOW_SPI_CS_PIN 4 // PB4 (IO3 on deck connector)
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL_PIN_NUM 6
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA_PIN_NUM 7

// I2C timeout for polling operations
#define I2C_TIMEOUT_US 100000

// VL53L1x register addresses
#define VL53L1X_REG_MODEL_ID 0x010F // Expected value: 0xEACC

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

// Flow deck state
static bool s_flow_deck_present = false;
static bool s_i2c1_initialized = false;
static pmw3901_dev_t s_pmw3901_dev;
static vl53l1x_dev_t s_vl53l1x_dev;

// Flow deck integration state (provides pseudo-GPS position)
static float s_integrated_x = 0.0f; // Integrated X position (m, world frame)
static float s_integrated_y = 0.0f; // Integrated Y position (m, world frame)
static float s_prev_yaw = 0.0f;     // Previous yaw for body-to-world rotation
static uint32_t s_prev_flow_time_us = 0;

// ----------------------------------------------------------------------------
// SysTick Handler
// ----------------------------------------------------------------------------

// Hive timer tick - called every 1ms to drive actor timers
extern void hive_timer_tick_isr(void);

void SysTick_Handler(void) {
    s_sys_tick_ms++;
    hive_timer_tick_isr();
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
// I2C1 Low-Level Interface (for VL53L1x on flow deck)
// PB6 = SCL, PB7 = SDA
// ----------------------------------------------------------------------------

static bool i2c1_wait(volatile uint16_t *reg, uint16_t mask,
                      uint16_t expected) {
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

    // Set SCL as GPIO output
    I2C1_SCL_PORT->MODER &= ~(3U << (I2C1_SCL_PIN_NUM * 2));
    I2C1_SCL_PORT->MODER |= (1U << (I2C1_SCL_PIN_NUM * 2));
    I2C1_SCL_PORT->OTYPER |= (1U << I2C1_SCL_PIN_NUM);

    // Set SDA as input to monitor
    I2C1_SDA_PORT->MODER &= ~(3U << (I2C1_SDA_PIN_NUM * 2));

    // Clock out up to 9 bits to release stuck slave
    // Delay ~3us per half-cycle at 168MHz (100 NOPs ~= 0.6us, need ~500 for 3us)
    for (int i = 0; i < 9; i++) {
        if (I2C1_SDA_PORT->IDR & (1U << I2C1_SDA_PIN_NUM)) {
            break;
        }
        I2C1_SCL_PORT->ODR &= ~(1U << I2C1_SCL_PIN_NUM);
        for (volatile int d = 0; d < 100; d++) {
            __NOP();
        }
        I2C1_SCL_PORT->ODR |= (1U << I2C1_SCL_PIN_NUM);
        for (volatile int d = 0; d < 100; d++) {
            __NOP();
        }
    }

    // Generate STOP condition (SDA low->high while SCL high)
    I2C1_SDA_PORT->MODER |= (1U << (I2C1_SDA_PIN_NUM * 2));
    I2C1_SDA_PORT->OTYPER |= (1U << I2C1_SDA_PIN_NUM);
    I2C1_SDA_PORT->ODR &= ~(1U << I2C1_SDA_PIN_NUM);
    for (volatile int d = 0; d < 100; d++) {
        __NOP();
    }
    I2C1_SDA_PORT->ODR |= (1U << I2C1_SDA_PIN_NUM);
    for (volatile int d = 0; d < 100; d++) {
        __NOP();
    }

    // Restore pins to AF mode
    I2C1_SCL_PORT->MODER &= ~(3U << (I2C1_SCL_PIN_NUM * 2));
    I2C1_SCL_PORT->MODER |= (2U << (I2C1_SCL_PIN_NUM * 2));
    I2C1_SDA_PORT->MODER &= ~(3U << (I2C1_SDA_PIN_NUM * 2));
    I2C1_SDA_PORT->MODER |= (2U << (I2C1_SDA_PIN_NUM * 2));

    I2C1->CR1 |= I2C_CR1_PE;
}

static void i2c1_reset(void) {
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // Reconfigure: 400kHz fast mode (APB1 = 42 MHz)
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
    GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;      // Open-drain
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
    // Clear any pending errors
    i2c1_check_error();

    // Wait for bus not busy
    if (!i2c1_wait(&I2C1->SR2, I2C_SR2_BUSY, 0)) {
        i2c1_bus_recovery();
        i2c1_reset();
        return false;
    }

    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait(&I2C1->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (write)
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

    // Send data
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

    // Wait for transfer complete
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

    // Clear any pending errors
    i2c1_check_error();

    // Wait for bus not busy
    if (!i2c1_wait(&I2C1->SR2, I2C_SR2_BUSY, 0)) {
        i2c1_bus_recovery();
        i2c1_reset();
        return false;
    }

    I2C1->CR1 |= I2C_CR1_ACK;

    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait(&I2C1->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (read)
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
        // N=1: Disable ACK before clearing ADDR, then set STOP
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
        // N=2: Use POS bit per STM32 reference manual
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

    // N>2
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
// SPI1 Low-Level Interface (for PMW3901 on flow deck)
// PA5 = SCK, PA6 = MISO, PA7 = MOSI, PB4 = CS
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

    // Configure PMW3901 CS pin (PB4) as output
    GPIOB->MODER &= ~(3U << (FLOW_SPI_CS_PIN * 2));
    GPIOB->MODER |= (1U << (FLOW_SPI_CS_PIN * 2));
    GPIOB->OSPEEDR |= (3U << (FLOW_SPI_CS_PIN * 2));
    GPIOB->ODR |= (1 << FLOW_SPI_CS_PIN); // CS high (deselected)

    // Configure SPI1: Master, 8-bit, CPOL=0, CPHA=0 (Mode 0), ~1.3 MHz (84/64)
    // Mode 0 is used by Bitcraze firmware for deck SPI (PMW3901 + SD card)
    // PMW3901 max SPI clock is 2 MHz per datasheet
    SPI1->CR1 = SPI_CR1_MSTR |                // Master mode
                SPI_CR1_BR_2 | SPI_CR1_BR_0 | // Baud rate = fPCLK/64 = 1.3 MHz
                SPI_CR1_SSM |                 // Software slave management
                SPI_CR1_SSI;                  // Internal slave select
    // CPOL=0, CPHA=0 (Mode 0) - both bits clear

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
// VL53L1x Platform Callbacks (I2C1)
// ----------------------------------------------------------------------------

#define I2C_WRITE_BUF_MAX 32

static int vl53l1x_i2c_write(uint8_t dev_addr, uint16_t reg_addr,
                             const uint8_t *data, uint16_t len) {
    if (len + 2 > I2C_WRITE_BUF_MAX)
        return -1;
    uint8_t buf[I2C_WRITE_BUF_MAX];
    buf[0] = (uint8_t)(reg_addr >> 8);
    buf[1] = (uint8_t)(reg_addr & 0xFF);
    for (uint16_t i = 0; i < len; i++)
        buf[i + 2] = data[i];
    if (!i2c1_write(dev_addr, buf, len + 2))
        return -1;
    // Delay after write to give sensor time to process
    // VL53L1x needs this during rapid config writes in sensor_init
    for (volatile int d = 0; d < 1000; d++)
        __NOP();
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

// ----------------------------------------------------------------------------
// PMW3901 Platform Callbacks (SPI1)
// ----------------------------------------------------------------------------

static int pmw3901_spi_transfer(uint8_t tx_data, uint8_t *rx_data) {
    uint8_t rx = spi1_transfer(tx_data);
    if (rx_data != NULL)
        *rx_data = rx;
    return 0;
}

static void pmw3901_cs_set(int level) {
    if (level != 0)
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
// Flow Deck Initialization
// ----------------------------------------------------------------------------

// Probe for VL53L1x at a specific address by reading model ID
// Returns true if VL53L1x found at this address
static bool vl53l1x_probe_address(uint8_t addr) {
    uint8_t id[2];
    if (vl53l1x_i2c_read(addr, VL53L1X_REG_MODEL_ID, id, 2) != 0) {
        return false;
    }
    uint16_t model_id = (uint16_t)(id[0] << 8) | id[1];
    return (model_id == 0xEACC); // Expected VL53L1x model ID
}

static bool flow_deck_init(void) {
    debug_swo_printf("[FLOW] Initializing flow deck...\n");

    // Initialize SPI1 for PMW3901
    spi1_init();

    // Initialize I2C1 for VL53L1x
    i2c1_init();

    // Try to initialize PMW3901 (optical flow)
    if (!pmw3901_init(&s_pmw3901_dev, &s_pmw3901_platform)) {
        debug_swo_printf("[FLOW] PMW3901 not found (no flow deck?)\n");
        return false;
    }
    debug_swo_printf("[FLOW] PMW3901 detected\n");

    // Allow I2C1 bus to stabilize after init
    platform_delay_ms(10);

    // Wake up VL53L1x by doing a few dummy I2C transactions
    // The sensor may need a few I2C cycles before responding to register reads
    {
        uint8_t dummy;
        for (int i = 0; i < 3; i++) {
            uint8_t reg = 0x00;
            i2c1_write(0x29, &reg, 1);
            i2c1_read(0x29, &dummy, 1);
            platform_delay_ms(5);
        }
    }

    // Scan I2C1 for VL53L1x at known addresses
    // Bitcraze firmware may reassign from default 0x29 to avoid conflicts
    static const uint8_t addrs_to_scan[] = {0x29, 0x6A, 0x30, 0x31, 0x32, 0x52};
    uint8_t found_addr = 0;

    for (size_t i = 0; i < sizeof(addrs_to_scan); i++) {
        if (vl53l1x_probe_address(addrs_to_scan[i])) {
            found_addr = addrs_to_scan[i];
            break;
        }
    }

    if (found_addr == 0) {
        debug_swo_printf("[FLOW] VL53L1x not found on I2C1\n");
        return false;
    }
    debug_swo_printf("[FLOW] VL53L1x found at 0x%02X\n", found_addr);

    // Initialize VL53L1x handle (sets platform callbacks)
    if (vl53l1x_init(&s_vl53l1x_dev, &s_vl53l1x_platform) != 0) {
        debug_swo_printf("[FLOW] VL53L1x init failed\n");
        return false;
    }

    // Override with discovered address (driver defaults to 0x29)
    s_vl53l1x_dev.i2c_address = found_addr;

    // Soft reset the sensor to ensure clean state
    debug_swo_printf("[FLOW] VL53L1x soft reset...\n");
    vl53l1x_reset(&s_vl53l1x_dev);
    platform_delay_ms(100); // Wait for reset to complete

    // Wait for sensor to boot (firmware ready)
    debug_swo_printf("[FLOW] VL53L1x waiting for boot...\n");
    uint8_t boot_state = 0;
    for (int i = 0; i < 150 && boot_state == 0; i++) {
        if (vl53l1x_boot_state(&s_vl53l1x_dev, &boot_state) != 0) {
            debug_swo_printf("[FLOW] VL53L1x boot_state read error\n");
        }
        platform_delay_ms(10);
    }
    if (boot_state == 0) {
        debug_swo_printf("[FLOW] VL53L1x boot timeout\n");
        return false;
    }
    debug_swo_printf("[FLOW] VL53L1x boot OK\n");

    // Additional delay before config writes
    platform_delay_ms(10);

    // Initialize sensor with default config (91 register writes)
    debug_swo_printf("[FLOW] VL53L1x sensor init (config writes)...\n");
    int8_t init_status = vl53l1x_sensor_init(&s_vl53l1x_dev);
    if (init_status != 0) {
        debug_swo_printf("[FLOW] VL53L1x sensor init failed: %d\n",
                         init_status);
        return false;
    }
    debug_swo_printf("[FLOW] VL53L1x initialized\n");

    // Configure for short distance mode (up to 1.3m, faster)
    vl53l1x_set_distance_mode(&s_vl53l1x_dev, VL53L1X_DISTANCE_MODE_SHORT);
    vl53l1x_set_timing_budget(&s_vl53l1x_dev, VL53L1X_TIMING_20MS);
    vl53l1x_set_inter_measurement(&s_vl53l1x_dev, 25); // 25ms period -> 40Hz

    // Start continuous ranging
    if (vl53l1x_start_ranging(&s_vl53l1x_dev) != 0) {
        debug_swo_printf("[FLOW] VL53L1x start ranging failed\n");
        return false;
    }

    debug_swo_printf("[FLOW] VL53L1x configured (short mode, 40Hz)\n");
    debug_swo_printf("[FLOW] Flow deck initialized OK\n");
    return true;
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
    debug_swo_printf("\n[INIT] Crazyflie 2.1+ HAL starting...\n");

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

#if PLATFORM_DEBUG_INIT
    // I2C bus scan - check what devices respond (slow, enable for debugging)
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
#endif

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

    // Try to initialize flow deck (optional - may not be present)
    debug_swo_printf("[INIT] Checking for flow deck...\n");
    s_flow_deck_present = flow_deck_init();

    s_initialized = true;
    debug_swo_printf("[INIT] Platform init complete\n");
    return 0;
}

bool platform_self_test(void) {
    debug_swo_printf("[TEST] Self-test starting...\n");

    // BMI088 hardware self-test (uses sensor's internal test mechanism)
    int8_t selftest_result = 0;
    uint16_t rslt;

    debug_swo_printf("[TEST] BMI088 gyro hardware self-test...\n");
    rslt = bmi088_perform_gyro_selftest(&selftest_result, &s_bmi088_dev);
    if (rslt != BMI088_OK || selftest_result != BMI088_SELFTEST_PASS) {
        debug_swo_printf("[TEST] Gyro self-test FAILED (rslt=%u, result=%d)\n",
                         rslt, selftest_result);
        return false;
    }
    debug_swo_printf("[TEST] Gyro self-test PASSED\n");

    // Re-initialize gyro after self-test (self-test may change settings)
    platform_delay_ms(50);
    rslt = bmi088_gyro_init(&s_bmi088_dev);
    if (rslt != BMI088_OK) {
        debug_swo_printf("[TEST] Gyro re-init failed: %u\n", rslt);
        return false;
    }
    s_bmi088_dev.gyro_cfg.odr = BMI088_GYRO_BW_116_ODR_1000_HZ;
    s_bmi088_dev.gyro_cfg.range = BMI088_GYRO_RANGE_2000_DPS;
    s_bmi088_dev.gyro_cfg.bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
    s_bmi088_dev.gyro_cfg.power = BMI088_GYRO_PM_NORMAL;
    bmi088_set_gyro_power_mode(&s_bmi088_dev);
    platform_delay_ms(30);
    bmi088_set_gyro_meas_conf(&s_bmi088_dev);

    debug_swo_printf("[TEST] BMI088 accel hardware self-test...\n");
    rslt = bmi088_perform_accel_selftest(&selftest_result, &s_bmi088_dev);
    if (rslt != BMI088_OK || selftest_result != BMI088_SELFTEST_PASS) {
        debug_swo_printf("[TEST] Accel self-test FAILED (rslt=%u, result=%d)\n",
                         rslt, selftest_result);
        return false;
    }
    debug_swo_printf("[TEST] Accel self-test PASSED\n");

    // Re-initialize accel after self-test
    platform_delay_ms(50);
    rslt =
        bmi088_accel_switch_control(&s_bmi088_dev, BMI088_ACCEL_POWER_ENABLE);
    platform_delay_ms(5);
    rslt = bmi088_accel_init(&s_bmi088_dev);
    if (rslt != BMI088_OK) {
        debug_swo_printf("[TEST] Accel re-init failed: %u\n", rslt);
        return false;
    }
    s_bmi088_dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
    s_bmi088_dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
    s_bmi088_dev.accel_cfg.bw = BMI088_ACCEL_BW_OSR4;
    s_bmi088_dev.accel_cfg.power = BMI088_ACCEL_PM_ACTIVE;
    bmi088_set_accel_power_mode(&s_bmi088_dev);
    platform_delay_ms(10);
    bmi088_set_accel_meas_conf(&s_bmi088_dev);

    // Read samples to verify sensors work after re-init
    struct bmi088_sensor_data accel_data;
    struct bmi088_sensor_data gyro_data;
    struct bmp3_data baro_data;

    // Test accelerometer read
    if (bmi088_get_accel_data(&accel_data, &s_bmi088_dev) != BMI088_OK) {
        debug_swo_printf("[TEST] Accel read FAILED\n");
        return false;
    }
    debug_swo_printf("[TEST] Accel: x=%d y=%d z=%d\n", accel_data.x,
                     accel_data.y, accel_data.z);

    // Test gyroscope read
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
    debug_swo_printf("[CAL] Keep drone STILL and LEVEL!\n");

    // Gyro bias calibration with variance-based stillness detection
    float gyro_sum[3] = {0, 0, 0};
    float gyro_sum_sq[3] = {0, 0, 0};
    struct bmi088_sensor_data gyro_data;
    const float gyro_scale = (2000.0f / 32768.0f) * (M_PI / 180.0f);
    int valid_samples = 0;

    debug_swo_printf("[CAL] Gyro calibration (%d samples)...\n",
                     CALIBRATION_SAMPLES);

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (bmi088_get_gyro_data(&gyro_data, &s_bmi088_dev) == BMI088_OK) {
            float gx = gyro_data.x * gyro_scale;
            float gy = gyro_data.y * gyro_scale;
            float gz = gyro_data.z * gyro_scale;
            gyro_sum[0] += gx;
            gyro_sum[1] += gy;
            gyro_sum[2] += gz;
            gyro_sum_sq[0] += gx * gx;
            gyro_sum_sq[1] += gy * gy;
            gyro_sum_sq[2] += gz * gz;
            valid_samples++;
        }
        if (i % 50 == 0) {
            platform_led_toggle();
        }
        platform_delay_ms(2); // ~500 Hz sample rate, ~1 second total
    }

    if (valid_samples < CALIBRATION_SAMPLES / 2) {
        debug_swo_printf("[CAL] Too few valid samples (%d)\n", valid_samples);
        return -1;
    }

    // Compute mean and variance
    float n = (float)valid_samples;
    float gyro_mean[3], gyro_var[3];
    for (int i = 0; i < 3; i++) {
        gyro_mean[i] = gyro_sum[i] / n;
        gyro_var[i] = (gyro_sum_sq[i] / n) - (gyro_mean[i] * gyro_mean[i]);
    }

    // Check variance - reject if drone was moving
    if (gyro_var[0] > GYRO_VARIANCE_THRESHOLD ||
        gyro_var[1] > GYRO_VARIANCE_THRESHOLD ||
        gyro_var[2] > GYRO_VARIANCE_THRESHOLD) {
        debug_swo_printf("[CAL] Gyro variance too high - drone not still!\n");
        debug_swo_printf("[CAL] Variance: x=%.6f y=%.6f z=%.6f (max=%.6f)\n",
                         (double)gyro_var[0], (double)gyro_var[1],
                         (double)gyro_var[2], (double)GYRO_VARIANCE_THRESHOLD);
        return -1;
    }

    s_gyro_bias[0] = gyro_mean[0];
    s_gyro_bias[1] = gyro_mean[1];
    s_gyro_bias[2] = gyro_mean[2];

    debug_swo_printf("[CAL] Gyro bias: x=%.4f y=%.4f z=%.4f rad/s\n",
                     (double)s_gyro_bias[0], (double)s_gyro_bias[1],
                     (double)s_gyro_bias[2]);
    debug_swo_printf("[CAL] Gyro variance: x=%.6f y=%.6f z=%.6f (OK)\n",
                     (double)gyro_var[0], (double)gyro_var[1],
                     (double)gyro_var[2]);

    // Accelerometer scale calibration (200 samples like Bitcraze)
    // Measure gravity magnitude and compute correction factor
    debug_swo_printf("[CAL] Accel scale calibration (200 samples)...\n");
    struct bmi088_sensor_data accel_cal_data;
    const float accel_scale_raw = (24.0f * GRAVITY) / 32768.0f;
    float accel_magnitude_sum = 0.0f;
    int accel_samples = 0;

    for (int i = 0; i < 200; i++) {
        if (bmi088_get_accel_data(&accel_cal_data, &s_bmi088_dev) ==
            BMI088_OK) {
            float ax = accel_cal_data.x * accel_scale_raw;
            float ay = accel_cal_data.y * accel_scale_raw;
            float az = accel_cal_data.z * accel_scale_raw;
            float magnitude = sqrtf(ax * ax + ay * ay + az * az);
            accel_magnitude_sum += magnitude;
            accel_samples++;
        }
        if (i % 20 == 0) {
            platform_led_toggle();
        }
        platform_delay_ms(5);
    }

    if (accel_samples > 0) {
        float avg_magnitude = accel_magnitude_sum / accel_samples;
        // Scale factor: expected is GRAVITY, actual is avg_magnitude
        if (avg_magnitude > 0.1f) {
            s_accel_scale = GRAVITY / avg_magnitude;
        }
        debug_swo_printf("[CAL] Accel scale: %.4f (raw mag=%.2f m/s^2)\n",
                         (double)s_accel_scale, (double)avg_magnitude);
    }

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

    // Read barometer (disabled - using ToF rangefinder for altitude instead)
    // Baro is unreliable due to prop wash and was causing sensor validation
    // failures that blocked the entire control pipeline.
    sensors->baro_valid = false;
    sensors->baro_altitude = 0.0f;
    (void)baro_data; // Suppress unused warning

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
    // M1: TIM2_CH2, M2: TIM2_CH4, M3: TIM2_CH1, M4: TIM4_CH4
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR4 = 0;
    TIM4->CCR4 = 0;
}

uint32_t platform_get_time_ms(void) {
    return s_sys_tick_ms;
}

uint32_t platform_get_time_us(void) {
    uint32_t ms = s_sys_tick_ms;
    uint32_t ticks = SysTick->VAL;
    uint32_t load = SysTick->LOAD;
    // Use 64-bit arithmetic to avoid overflow (load can be 168000)
    uint32_t us_in_tick = (uint32_t)(((uint64_t)(load - ticks) * 1000) / load);
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
    return s_flow_deck_present;
}

bool platform_read_flow(int16_t *delta_x, int16_t *delta_y) {
    if (!s_flow_deck_present)
        return false;
    pmw3901_read_motion(&s_pmw3901_dev, delta_x, delta_y);
    return true;
}

bool platform_read_height(uint16_t *height_mm) {
    if (!s_flow_deck_present)
        return false;

    uint8_t ready = 0;
    vl53l1x_check_data_ready(&s_vl53l1x_dev, &ready);
    if (ready == 0)
        return false;

    uint16_t distance;
    vl53l1x_get_distance(&s_vl53l1x_dev, &distance);
    vl53l1x_clear_interrupt(&s_vl53l1x_dev);

    // Outlier filtering: discard readings > 5m (sensor max range, likely invalid)
    if (distance > 5000) {
        return false;
    }

    *height_mm = distance;
    return true;
}

void platform_debug_init(void) {
    debug_swo_init(CPU_FREQ_HZ, SWO_BAUD);
}

void platform_debug_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    debug_swo_vprintf(fmt, args);
    va_end(args);
}

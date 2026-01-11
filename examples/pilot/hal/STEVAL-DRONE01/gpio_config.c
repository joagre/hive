// GPIO configuration for STEVAL-DRONE01
//
// Configures all GPIO pins for peripherals.

#include "gpio_config.h"
#include "system_config.h"

// ----------------------------------------------------------------------------
// GPIO Register Definitions
// ----------------------------------------------------------------------------

#define GPIOA_BASE          0x40020000U
#define GPIOB_BASE          0x40020400U
#define GPIOC_BASE          0x40020800U
#define GPIOD_BASE          0x40020C00U
#define GPIOE_BASE          0x40021000U
#define GPIOH_BASE          0x40021C00U

// GPIO register offsets
#define GPIO_MODER          0x00    // Mode register
#define GPIO_OTYPER         0x04    // Output type register
#define GPIO_OSPEEDR        0x08    // Output speed register
#define GPIO_PUPDR          0x0C    // Pull-up/pull-down register
#define GPIO_IDR            0x10    // Input data register
#define GPIO_ODR            0x14    // Output data register
#define GPIO_BSRR           0x18    // Bit set/reset register
#define GPIO_LCKR           0x1C    // Lock register
#define GPIO_AFRL           0x20    // Alternate function low register (pins 0-7)
#define GPIO_AFRH           0x24    // Alternate function high register (pins 8-15)

// ----------------------------------------------------------------------------
// Helper Functions
// ----------------------------------------------------------------------------

static uint32_t get_gpio_base(char port) {
    switch (port) {
        case 'A': case 'a': return GPIOA_BASE;
        case 'B': case 'b': return GPIOB_BASE;
        case 'C': case 'c': return GPIOC_BASE;
        case 'D': case 'd': return GPIOD_BASE;
        case 'E': case 'e': return GPIOE_BASE;
        case 'H': case 'h': return GPIOH_BASE;
        default: return 0;
    }
}

static volatile uint32_t *gpio_reg(char port, uint32_t offset) {
    uint32_t base = get_gpio_base(port);
    if (base == 0) return 0;
    return (volatile uint32_t *)(base + offset);
}

// ----------------------------------------------------------------------------
// Low-Level GPIO Configuration
// ----------------------------------------------------------------------------

void gpio_set_mode(char port, uint8_t pin, gpio_mode_t mode) {
    volatile uint32_t *moder = gpio_reg(port, GPIO_MODER);
    if (!moder) return;

    uint32_t mask = 3U << (pin * 2);
    uint32_t value = (uint32_t)mode << (pin * 2);
    *moder = (*moder & ~mask) | value;
}

void gpio_set_otype(char port, uint8_t pin, gpio_otype_t otype) {
    volatile uint32_t *otyper = gpio_reg(port, GPIO_OTYPER);
    if (!otyper) return;

    if (otype == GPIO_OTYPE_OPENDRAIN) {
        *otyper |= (1U << pin);
    } else {
        *otyper &= ~(1U << pin);
    }
}

void gpio_set_speed(char port, uint8_t pin, gpio_speed_t speed) {
    volatile uint32_t *ospeedr = gpio_reg(port, GPIO_OSPEEDR);
    if (!ospeedr) return;

    uint32_t mask = 3U << (pin * 2);
    uint32_t value = (uint32_t)speed << (pin * 2);
    *ospeedr = (*ospeedr & ~mask) | value;
}

void gpio_set_pupd(char port, uint8_t pin, gpio_pupd_t pupd) {
    volatile uint32_t *pupdr = gpio_reg(port, GPIO_PUPDR);
    if (!pupdr) return;

    uint32_t mask = 3U << (pin * 2);
    uint32_t value = (uint32_t)pupd << (pin * 2);
    *pupdr = (*pupdr & ~mask) | value;
}

void gpio_set_af(char port, uint8_t pin, uint8_t af) {
    volatile uint32_t *afr;
    uint8_t shift;

    if (pin < 8) {
        afr = gpio_reg(port, GPIO_AFRL);
        shift = pin * 4;
    } else {
        afr = gpio_reg(port, GPIO_AFRH);
        shift = (pin - 8) * 4;
    }

    if (!afr) return;

    uint32_t mask = 0xFU << shift;
    uint32_t value = (uint32_t)af << shift;
    *afr = (*afr & ~mask) | value;
}

// ----------------------------------------------------------------------------
// GPIO Read/Write
// ----------------------------------------------------------------------------

void gpio_write(char port, uint8_t pin, bool value) {
    volatile uint32_t *bsrr = gpio_reg(port, GPIO_BSRR);
    if (!bsrr) return;

    if (value) {
        *bsrr = (1U << pin);           // Set bit
    } else {
        *bsrr = (1U << (pin + 16));    // Reset bit
    }
}

bool gpio_read(char port, uint8_t pin) {
    volatile uint32_t *idr = gpio_reg(port, GPIO_IDR);
    if (!idr) return false;

    return (*idr & (1U << pin)) != 0;
}

void gpio_toggle(char port, uint8_t pin) {
    volatile uint32_t *odr = gpio_reg(port, GPIO_ODR);
    if (!odr) return;

    *odr ^= (1U << pin);
}

// ----------------------------------------------------------------------------
// SPI1 GPIO Configuration (LSM6DSL)
// ----------------------------------------------------------------------------

void gpio_init_spi1(void) {
    // Enable GPIO clocks
    system_enable_gpio(SPI1_SCK_PORT);
    system_enable_gpio(LSM6DSL_CS_PORT);

    // SPI1_SCK (PA5) - Alternate function, push-pull, high speed
    gpio_set_mode(SPI1_SCK_PORT, SPI1_SCK_PIN, GPIO_MODE_AF);
    gpio_set_otype(SPI1_SCK_PORT, SPI1_SCK_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(SPI1_SCK_PORT, SPI1_SCK_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(SPI1_SCK_PORT, SPI1_SCK_PIN, GPIO_PUPD_NONE);
    gpio_set_af(SPI1_SCK_PORT, SPI1_SCK_PIN, SPI1_AF);

    // SPI1_MISO (PA6) - Alternate function
    gpio_set_mode(SPI1_MISO_PORT, SPI1_MISO_PIN, GPIO_MODE_AF);
    gpio_set_speed(SPI1_MISO_PORT, SPI1_MISO_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(SPI1_MISO_PORT, SPI1_MISO_PIN, GPIO_PUPD_NONE);
    gpio_set_af(SPI1_MISO_PORT, SPI1_MISO_PIN, SPI1_AF);

    // SPI1_MOSI (PA7) - Alternate function, push-pull
    gpio_set_mode(SPI1_MOSI_PORT, SPI1_MOSI_PIN, GPIO_MODE_AF);
    gpio_set_otype(SPI1_MOSI_PORT, SPI1_MOSI_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(SPI1_MOSI_PORT, SPI1_MOSI_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(SPI1_MOSI_PORT, SPI1_MOSI_PIN, GPIO_PUPD_NONE);
    gpio_set_af(SPI1_MOSI_PORT, SPI1_MOSI_PIN, SPI1_AF);

    // LSM6DSL_CS (PA4) - GPIO output, push-pull, high speed
    // Start with CS high (deselected)
    gpio_write(LSM6DSL_CS_PORT, LSM6DSL_CS_PIN, true);
    gpio_set_mode(LSM6DSL_CS_PORT, LSM6DSL_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_otype(LSM6DSL_CS_PORT, LSM6DSL_CS_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(LSM6DSL_CS_PORT, LSM6DSL_CS_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(LSM6DSL_CS_PORT, LSM6DSL_CS_PIN, GPIO_PUPD_NONE);
}

void gpio_lsm6dsl_cs_low(void) {
    gpio_write(LSM6DSL_CS_PORT, LSM6DSL_CS_PIN, false);
}

void gpio_lsm6dsl_cs_high(void) {
    gpio_write(LSM6DSL_CS_PORT, LSM6DSL_CS_PIN, true);
}

// ----------------------------------------------------------------------------
// SPI2 GPIO Configuration (LIS2MDL, LPS22HD)
// ----------------------------------------------------------------------------

void gpio_init_spi2(void) {
    // Enable GPIO clocks
    system_enable_gpio(SPI2_SCK_PORT);

    // SPI2_SCK (PB13) - Alternate function, push-pull, high speed
    gpio_set_mode(SPI2_SCK_PORT, SPI2_SCK_PIN, GPIO_MODE_AF);
    gpio_set_otype(SPI2_SCK_PORT, SPI2_SCK_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(SPI2_SCK_PORT, SPI2_SCK_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(SPI2_SCK_PORT, SPI2_SCK_PIN, GPIO_PUPD_NONE);
    gpio_set_af(SPI2_SCK_PORT, SPI2_SCK_PIN, SPI2_AF);

    // SPI2_MISO (PB14) - Alternate function
    gpio_set_mode(SPI2_MISO_PORT, SPI2_MISO_PIN, GPIO_MODE_AF);
    gpio_set_speed(SPI2_MISO_PORT, SPI2_MISO_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(SPI2_MISO_PORT, SPI2_MISO_PIN, GPIO_PUPD_NONE);
    gpio_set_af(SPI2_MISO_PORT, SPI2_MISO_PIN, SPI2_AF);

    // SPI2_MOSI (PB15) - Alternate function, push-pull
    gpio_set_mode(SPI2_MOSI_PORT, SPI2_MOSI_PIN, GPIO_MODE_AF);
    gpio_set_otype(SPI2_MOSI_PORT, SPI2_MOSI_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(SPI2_MOSI_PORT, SPI2_MOSI_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(SPI2_MOSI_PORT, SPI2_MOSI_PIN, GPIO_PUPD_NONE);
    gpio_set_af(SPI2_MOSI_PORT, SPI2_MOSI_PIN, SPI2_AF);

    // LIS2MDL_CS (PB12) - GPIO output, push-pull, high speed
    // Start with CS high (deselected)
    gpio_write(LIS2MDL_CS_PORT, LIS2MDL_CS_PIN, true);
    gpio_set_mode(LIS2MDL_CS_PORT, LIS2MDL_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_otype(LIS2MDL_CS_PORT, LIS2MDL_CS_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(LIS2MDL_CS_PORT, LIS2MDL_CS_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(LIS2MDL_CS_PORT, LIS2MDL_CS_PIN, GPIO_PUPD_NONE);

    // LPS22HD_CS (PB10) - GPIO output, push-pull, high speed
    // Start with CS high (deselected)
    gpio_write(LPS22HD_CS_PORT, LPS22HD_CS_PIN, true);
    gpio_set_mode(LPS22HD_CS_PORT, LPS22HD_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_otype(LPS22HD_CS_PORT, LPS22HD_CS_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(LPS22HD_CS_PORT, LPS22HD_CS_PIN, GPIO_SPEED_VERYHIGH);
    gpio_set_pupd(LPS22HD_CS_PORT, LPS22HD_CS_PIN, GPIO_PUPD_NONE);
}

void gpio_lis2mdl_cs_low(void) {
    gpio_write(LIS2MDL_CS_PORT, LIS2MDL_CS_PIN, false);
}

void gpio_lis2mdl_cs_high(void) {
    gpio_write(LIS2MDL_CS_PORT, LIS2MDL_CS_PIN, true);
}

void gpio_lps22hd_cs_low(void) {
    gpio_write(LPS22HD_CS_PORT, LPS22HD_CS_PIN, false);
}

void gpio_lps22hd_cs_high(void) {
    gpio_write(LPS22HD_CS_PORT, LPS22HD_CS_PIN, true);
}

// ----------------------------------------------------------------------------
// I2C1 GPIO Configuration (not used, sensors on SPI2)
// ----------------------------------------------------------------------------

void gpio_init_i2c1(void) {
    // Enable GPIO clock
    system_enable_gpio(I2C1_SCL_PORT);

    // I2C1_SCL (PB6) - Alternate function, open-drain, pull-up
    gpio_set_mode(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_MODE_AF);
    gpio_set_otype(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_OTYPE_OPENDRAIN);
    gpio_set_speed(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_SPEED_HIGH);
    gpio_set_pupd(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PUPD_PULLUP);
    gpio_set_af(I2C1_SCL_PORT, I2C1_SCL_PIN, I2C1_AF);

    // I2C1_SDA (PB7) - Alternate function, open-drain, pull-up
    gpio_set_mode(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_MODE_AF);
    gpio_set_otype(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_OTYPE_OPENDRAIN);
    gpio_set_speed(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_SPEED_HIGH);
    gpio_set_pupd(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PUPD_PULLUP);
    gpio_set_af(I2C1_SDA_PORT, I2C1_SDA_PIN, I2C1_AF);
}

void gpio_i2c1_release_bus(void) {
    // I2C bus recovery: Toggle SCL to release stuck SDA
    // This handles the case where a slave holds SDA low due to incomplete transfer

    // Configure SCL as GPIO output
    gpio_set_mode(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_otype(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_OTYPE_OPENDRAIN);

    // Configure SDA as GPIO input to monitor
    gpio_set_mode(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_MODE_INPUT);
    gpio_set_pupd(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PUPD_PULLUP);

    // Toggle SCL up to 9 times until SDA goes high
    for (int i = 0; i < 9; i++) {
        gpio_write(I2C1_SCL_PORT, I2C1_SCL_PIN, false);
        system_delay_us(5);
        gpio_write(I2C1_SCL_PORT, I2C1_SCL_PIN, true);
        system_delay_us(5);

        // Check if SDA is released
        if (gpio_read(I2C1_SDA_PORT, I2C1_SDA_PIN)) {
            break;
        }
    }

    // Generate STOP condition (SDA low->high while SCL high)
    gpio_set_mode(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_otype(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_OTYPE_OPENDRAIN);
    gpio_write(I2C1_SDA_PORT, I2C1_SDA_PIN, false);
    system_delay_us(5);
    gpio_write(I2C1_SCL_PORT, I2C1_SCL_PIN, true);
    system_delay_us(5);
    gpio_write(I2C1_SDA_PORT, I2C1_SDA_PIN, true);
    system_delay_us(5);

    // Restore I2C alternate function mode
    gpio_init_i2c1();
}

// ----------------------------------------------------------------------------
// TIM4 GPIO Configuration (Motor PWM)
// ----------------------------------------------------------------------------

void gpio_init_tim4_pwm(void) {
    // Enable GPIO clock
    system_enable_gpio(TIM4_CH3_PORT);  // PB for channels 3,4

    // Note: TIM4_CH1/CH2 on PB6/PB7 conflict with I2C1
    // Using PB8/PB9 for CH3/CH4, alternative pins for CH1/CH2

    // For full 4-motor support, either:
    // 1. Use different I2C pins, or
    // 2. Use alternative TIM4 pins on port D

    // TIM4_CH3 (PB8) - Alternate function, push-pull
    gpio_set_mode(TIM4_CH3_PORT, TIM4_CH3_PIN, GPIO_MODE_AF);
    gpio_set_otype(TIM4_CH3_PORT, TIM4_CH3_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(TIM4_CH3_PORT, TIM4_CH3_PIN, GPIO_SPEED_HIGH);
    gpio_set_pupd(TIM4_CH3_PORT, TIM4_CH3_PIN, GPIO_PUPD_NONE);
    gpio_set_af(TIM4_CH3_PORT, TIM4_CH3_PIN, TIM4_AF);

    // TIM4_CH4 (PB9) - Alternate function, push-pull
    gpio_set_mode(TIM4_CH4_PORT, TIM4_CH4_PIN, GPIO_MODE_AF);
    gpio_set_otype(TIM4_CH4_PORT, TIM4_CH4_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(TIM4_CH4_PORT, TIM4_CH4_PIN, GPIO_SPEED_HIGH);
    gpio_set_pupd(TIM4_CH4_PORT, TIM4_CH4_PIN, GPIO_PUPD_NONE);
    gpio_set_af(TIM4_CH4_PORT, TIM4_CH4_PIN, TIM4_AF);

    // Alternative: Use port D for all 4 channels (if available)
    // Uncomment below and comment out I2C1 init if using PD12-15
    /*
    system_enable_gpio('D');

    gpio_set_mode('D', 12, GPIO_MODE_AF);  // TIM4_CH1
    gpio_set_otype('D', 12, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed('D', 12, GPIO_SPEED_HIGH);
    gpio_set_af('D', 12, TIM4_AF);

    gpio_set_mode('D', 13, GPIO_MODE_AF);  // TIM4_CH2
    gpio_set_otype('D', 13, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed('D', 13, GPIO_SPEED_HIGH);
    gpio_set_af('D', 13, TIM4_AF);

    gpio_set_mode('D', 14, GPIO_MODE_AF);  // TIM4_CH3
    gpio_set_otype('D', 14, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed('D', 14, GPIO_SPEED_HIGH);
    gpio_set_af('D', 14, TIM4_AF);

    gpio_set_mode('D', 15, GPIO_MODE_AF);  // TIM4_CH4
    gpio_set_otype('D', 15, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed('D', 15, GPIO_SPEED_HIGH);
    gpio_set_af('D', 15, TIM4_AF);
    */
}

// ----------------------------------------------------------------------------
// USART1 GPIO Configuration (Debug Serial)
// ----------------------------------------------------------------------------

void gpio_init_usart1(void) {
    // Enable GPIO clock
    system_enable_gpio(USART1_TX_PORT);

    // USART1_TX (PA9) - Alternate function, push-pull
    gpio_set_mode(USART1_TX_PORT, USART1_TX_PIN, GPIO_MODE_AF);
    gpio_set_otype(USART1_TX_PORT, USART1_TX_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(USART1_TX_PORT, USART1_TX_PIN, GPIO_SPEED_HIGH);
    gpio_set_pupd(USART1_TX_PORT, USART1_TX_PIN, GPIO_PUPD_NONE);
    gpio_set_af(USART1_TX_PORT, USART1_TX_PIN, USART1_AF);

    // USART1_RX (PA10) - Alternate function, pull-up
    gpio_set_mode(USART1_RX_PORT, USART1_RX_PIN, GPIO_MODE_AF);
    gpio_set_pupd(USART1_RX_PORT, USART1_RX_PIN, GPIO_PUPD_PULLUP);
    gpio_set_af(USART1_RX_PORT, USART1_RX_PIN, USART1_AF);
}

void gpio_init_usart2(void) {
    // Enable GPIO clock
    system_enable_gpio(USART2_TX_PORT);

    // USART2_TX (PA2) - Alternate function, push-pull
    gpio_set_mode(USART2_TX_PORT, USART2_TX_PIN, GPIO_MODE_AF);
    gpio_set_otype(USART2_TX_PORT, USART2_TX_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(USART2_TX_PORT, USART2_TX_PIN, GPIO_SPEED_HIGH);
    gpio_set_pupd(USART2_TX_PORT, USART2_TX_PIN, GPIO_PUPD_NONE);
    gpio_set_af(USART2_TX_PORT, USART2_TX_PIN, USART2_AF);

    // USART2_RX (PA3) - Alternate function, pull-up
    gpio_set_mode(USART2_RX_PORT, USART2_RX_PIN, GPIO_MODE_AF);
    gpio_set_pupd(USART2_RX_PORT, USART2_RX_PIN, GPIO_PUPD_PULLUP);
    gpio_set_af(USART2_RX_PORT, USART2_RX_PIN, USART2_AF);
}

// ----------------------------------------------------------------------------
// LED GPIO Configuration
// ----------------------------------------------------------------------------

void gpio_init_led(void) {
    system_enable_gpio(LED_PORT);

    // LED (PC13) - Output, push-pull
    gpio_write(LED_PORT, LED_PIN, false);  // Start off
    gpio_set_mode(LED_PORT, LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_otype(LED_PORT, LED_PIN, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed(LED_PORT, LED_PIN, GPIO_SPEED_LOW);
    gpio_set_pupd(LED_PORT, LED_PIN, GPIO_PUPD_NONE);
}

void gpio_led_on(void) {
    // Many boards have active-low LED
    gpio_write(LED_PORT, LED_PIN, false);
}

void gpio_led_off(void) {
    gpio_write(LED_PORT, LED_PIN, true);
}

void gpio_led_toggle(void) {
    gpio_toggle(LED_PORT, LED_PIN);
}

// ----------------------------------------------------------------------------
// Button GPIO Configuration
// ----------------------------------------------------------------------------

void gpio_init_button(void) {
    system_enable_gpio(BTN_PORT);

    // Button (PA0) - Input, pull-down (if button connects to VCC)
    gpio_set_mode(BTN_PORT, BTN_PIN, GPIO_MODE_INPUT);
    gpio_set_pupd(BTN_PORT, BTN_PIN, GPIO_PUPD_PULLDOWN);
}

bool gpio_button_read(void) {
    return gpio_read(BTN_PORT, BTN_PIN);
}

// ----------------------------------------------------------------------------
// Initialize All GPIO
// ----------------------------------------------------------------------------

void gpio_init_all(void) {
    gpio_init_spi1();       // IMU
    gpio_init_i2c1();       // Magnetometer, barometer
    gpio_init_tim4_pwm();   // Motors (channels 3,4 only due to I2C conflict)
    gpio_init_usart1();     // Debug serial
    gpio_init_led();        // Status LED
    gpio_init_button();     // User button
}

// GPIO configuration for STEVAL-DRONE01
//
// Pin mappings for all peripherals.

#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Pin Definitions
// ----------------------------------------------------------------------------

// SPI1 - LSM6DSL (IMU)
#define SPI1_SCK_PORT       'A'
#define SPI1_SCK_PIN        5
#define SPI1_MISO_PORT      'A'
#define SPI1_MISO_PIN       6
#define SPI1_MOSI_PORT      'A'
#define SPI1_MOSI_PIN       7
#define SPI1_AF             5       // Alternate function 5

// LSM6DSL chip select (directly controlled GPIO)
#define LSM6DSL_CS_PORT     'A'
#define LSM6DSL_CS_PIN      8       // PA8 on STEVAL-FCU001V1

// SPI2 - LIS2MDL, LPS22HD (magnetometer and barometer)
#define SPI2_SCK_PORT       'B'
#define SPI2_SCK_PIN        13
#define SPI2_MISO_PORT      'B'
#define SPI2_MISO_PIN       14
#define SPI2_MOSI_PORT      'B'
#define SPI2_MOSI_PIN       15
#define SPI2_AF             5       // Alternate function 5

// LIS2MDL chip select (magnetometer)
#define LIS2MDL_CS_PORT     'B'
#define LIS2MDL_CS_PIN      12

// LPS22HD chip select (barometer)
#define LPS22HD_CS_PORT     'B'
#define LPS22HD_CS_PIN      10

// I2C1 - (not used, sensors are on SPI2)
#define I2C1_SCL_PORT       'B'
#define I2C1_SCL_PIN        6
#define I2C1_SDA_PORT       'B'
#define I2C1_SDA_PIN        7
#define I2C1_AF             4       // Alternate function 4

// TIM4 - Motor PWM (channels 1-4)
#define TIM4_CH1_PORT       'B'
#define TIM4_CH1_PIN        6       // Note: Shared with I2C1_SCL on some boards
#define TIM4_CH2_PORT       'B'
#define TIM4_CH2_PIN        7       // Note: Shared with I2C1_SDA on some boards
#define TIM4_CH3_PORT       'B'
#define TIM4_CH3_PIN        8
#define TIM4_CH4_PORT       'B'
#define TIM4_CH4_PIN        9
#define TIM4_AF             2       // Alternate function 2

// Alternative TIM4 pins (if I2C1 uses PB6/PB7)
#define TIM4_CH1_ALT_PORT   'D'
#define TIM4_CH1_ALT_PIN    12
#define TIM4_CH2_ALT_PORT   'D'
#define TIM4_CH2_ALT_PIN    13
#define TIM4_CH3_ALT_PORT   'D'
#define TIM4_CH3_ALT_PIN    14
#define TIM4_CH4_ALT_PORT   'D'
#define TIM4_CH4_ALT_PIN    15

// USART1 - Debug serial
#define USART1_TX_PORT      'A'
#define USART1_TX_PIN       9
#define USART1_RX_PORT      'A'
#define USART1_RX_PIN       10
#define USART1_AF           7       // Alternate function 7

// USART2 - Alternative debug serial
#define USART2_TX_PORT      'A'
#define USART2_TX_PIN       2
#define USART2_RX_PORT      'A'
#define USART2_RX_PIN       3
#define USART2_AF           7       // Alternate function 7

// LED (PB5 = LD1 on STEVAL-FCU001V1)
#define LED_PORT            'B'
#define LED_PIN             5

// User button (optional, board-dependent)
#define BTN_PORT            'A'
#define BTN_PIN             0

// ----------------------------------------------------------------------------
// GPIO Mode and Configuration Types
// ----------------------------------------------------------------------------

typedef enum {
    GPIO_MODE_INPUT     = 0,
    GPIO_MODE_OUTPUT    = 1,
    GPIO_MODE_AF        = 2,    // Alternate function
    GPIO_MODE_ANALOG    = 3
} gpio_mode_t;

typedef enum {
    GPIO_OTYPE_PUSHPULL  = 0,
    GPIO_OTYPE_OPENDRAIN = 1
} gpio_otype_t;

typedef enum {
    GPIO_SPEED_LOW      = 0,
    GPIO_SPEED_MEDIUM   = 1,
    GPIO_SPEED_HIGH     = 2,
    GPIO_SPEED_VERYHIGH = 3
} gpio_speed_t;

typedef enum {
    GPIO_PUPD_NONE      = 0,
    GPIO_PUPD_PULLUP    = 1,
    GPIO_PUPD_PULLDOWN  = 2
} gpio_pupd_t;

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize all GPIO for peripherals
void gpio_init_all(void);

// Individual peripheral GPIO initialization
void gpio_init_spi1(void);
void gpio_init_spi2(void);
void gpio_init_i2c1(void);
void gpio_init_tim4_pwm(void);
void gpio_init_usart1(void);
void gpio_init_usart2(void);
void gpio_init_led(void);
void gpio_init_button(void);

// LSM6DSL chip select control (SPI1)
void gpio_lsm6dsl_cs_low(void);
void gpio_lsm6dsl_cs_high(void);

// LIS2MDL chip select control (SPI2)
void gpio_lis2mdl_cs_low(void);
void gpio_lis2mdl_cs_high(void);

// LPS22HD chip select control (SPI2)
void gpio_lps22hd_cs_low(void);
void gpio_lps22hd_cs_high(void);

// I2C1 bus recovery (clock toggle to release stuck SDA)
void gpio_i2c1_release_bus(void);

// LED control
void gpio_led_on(void);
void gpio_led_off(void);
void gpio_led_toggle(void);

// Button read
bool gpio_button_read(void);

// Low-level GPIO configuration
void gpio_set_mode(char port, uint8_t pin, gpio_mode_t mode);
void gpio_set_otype(char port, uint8_t pin, gpio_otype_t otype);
void gpio_set_speed(char port, uint8_t pin, gpio_speed_t speed);
void gpio_set_pupd(char port, uint8_t pin, gpio_pupd_t pupd);
void gpio_set_af(char port, uint8_t pin, uint8_t af);

// GPIO read/write
void gpio_write(char port, uint8_t pin, bool value);
bool gpio_read(char port, uint8_t pin);
void gpio_toggle(char port, uint8_t pin);

#endif // GPIO_CONFIG_H

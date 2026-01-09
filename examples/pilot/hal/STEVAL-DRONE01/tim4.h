// TIM4 PWM driver for STM32F401 (STEVAL-DRONE01)
//
// Configured for brushed DC motor control.
// 4-channel PWM output at 20kHz for quiet motor operation.

#ifndef TIM4_H
#define TIM4_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// PWM frequency options
typedef enum {
    TIM4_PWM_8KHZ   = 8000,     // 8 kHz - audible but efficient
    TIM4_PWM_16KHZ  = 16000,    // 16 kHz - near ultrasonic
    TIM4_PWM_20KHZ  = 20000,    // 20 kHz - ultrasonic (default)
    TIM4_PWM_25KHZ  = 25000     // 25 kHz - higher switching losses
} tim4_pwm_freq_t;

// Default PWM frequency (20kHz - inaudible)
#define TIM4_DEFAULT_PWM_FREQ   TIM4_PWM_20KHZ

// PWM resolution (10-bit = 1024 steps, good balance of resolution and frequency)
#define TIM4_PWM_RESOLUTION     1024

// Motor channel definitions
// Note: CH1/CH2 (PB6/PB7) conflict with I2C1, so we use alternative configuration
typedef enum {
    TIM4_CH1 = 0,   // Channel 1 (PB6 or PD12)
    TIM4_CH2 = 1,   // Channel 2 (PB7 or PD13)
    TIM4_CH3 = 2,   // Channel 3 (PB8 or PD14)
    TIM4_CH4 = 3    // Channel 4 (PB9 or PD15)
} tim4_channel_t;

// Pin configuration options
typedef enum {
    TIM4_PINS_PB6_PB9,      // PB6-PB9 (conflicts with I2C1 on PB6/PB7)
    TIM4_PINS_PD12_PD15,    // PD12-PD15 (alternative, no conflicts)
    TIM4_PINS_PB8_PB9_ONLY  // Only PB8/PB9 (CH3/CH4), for use with I2C1
} tim4_pin_config_t;

// Configuration structure
typedef struct {
    tim4_pwm_freq_t frequency;      // PWM frequency
    tim4_pin_config_t pin_config;   // Pin configuration
    bool ch1_enable;                // Enable channel 1
    bool ch2_enable;                // Enable channel 2
    bool ch3_enable;                // Enable channel 3
    bool ch4_enable;                // Enable channel 4
} tim4_config_t;

// Default configuration: 20kHz, PB8/PB9 only (compatible with I2C1)
#define TIM4_CONFIG_DEFAULT { \
    .frequency = TIM4_PWM_20KHZ, \
    .pin_config = TIM4_PINS_PB8_PB9_ONLY, \
    .ch1_enable = false, \
    .ch2_enable = false, \
    .ch3_enable = true, \
    .ch4_enable = true \
}

// ----------------------------------------------------------------------------
// API
// ----------------------------------------------------------------------------

// Initialize TIM4 for PWM output
// config: Configuration options (NULL for defaults)
void tim4_init(const tim4_config_t *config);

// Deinitialize TIM4
void tim4_deinit(void);

// Set PWM duty cycle for a channel
// channel: TIM4_CH1 to TIM4_CH4
// duty: Duty cycle 0.0 to 1.0 (0% to 100%)
void tim4_set_duty(tim4_channel_t channel, float duty);

// Set PWM duty cycle using raw value
// channel: TIM4_CH1 to TIM4_CH4
// value: Raw compare value (0 to TIM4_PWM_RESOLUTION-1)
void tim4_set_raw(tim4_channel_t channel, uint16_t value);

// Set all 4 channels at once (more efficient)
// duties: Array of 4 duty cycles (0.0 to 1.0)
void tim4_set_all(const float duties[4]);

// Set all 4 channels using raw values
// values: Array of 4 raw compare values
void tim4_set_all_raw(const uint16_t values[4]);

// Enable PWM output on all configured channels
void tim4_enable(void);

// Disable PWM output (all channels go low)
void tim4_disable(void);

// Check if PWM output is enabled
bool tim4_is_enabled(void);

// Get current PWM frequency
uint32_t tim4_get_frequency(void);

// Get PWM resolution (max compare value + 1)
uint16_t tim4_get_resolution(void);

#endif // TIM4_H

// Motor PWM Driver Implementation for Crazyflie 2.1+
//
// Uses TIM2 for PWM generation on PA0-PA3.
// Reference: STM32F405 Reference Manual, Crazyflie firmware (motors.c)

#include "motors.h"
#include "stm32f4xx.h"

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// PWM resolution (8-bit for compatibility with Crazyflie firmware)
#define PWM_RESOLUTION  255

// TIM2 runs at APB1*2 = 84 MHz (assuming 168 MHz system clock)
// For 328 kHz PWM: 84 MHz / 256 = 328.125 kHz
// For 50 kHz PWM: 84 MHz / 1680 = 50 kHz

// Motor to channel mapping
#define MOTOR_M1_CHANNEL  TIM_CHANNEL_1  // PA0
#define MOTOR_M2_CHANNEL  TIM_CHANNEL_2  // PA1
#define MOTOR_M3_CHANNEL  TIM_CHANNEL_3  // PA2
#define MOTOR_M4_CHANNEL  TIM_CHANNEL_4  // PA3

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;
static bool s_armed = false;
static motors_config_t s_config;
static uint16_t s_pwm[MOTORS_COUNT] = {0, 0, 0, 0};

// CCR register pointers for direct access
static volatile uint32_t *s_ccr[MOTORS_COUNT];

// ----------------------------------------------------------------------------
// Helper Functions
// ----------------------------------------------------------------------------

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline uint16_t float_to_pwm(float value) {
    value = clampf(value, 0.0f, 1.0f);
    uint16_t range = s_config.max_pulse - s_config.min_pulse;
    return s_config.min_pulse + (uint16_t)(value * range);
}

// ----------------------------------------------------------------------------
// GPIO and Timer Initialization
// ----------------------------------------------------------------------------

static void gpio_init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA0-PA3 as alternate function (AF1 = TIM2)
    // MODER: 10 = alternate function
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
                      GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 |
                     GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);

    // High speed output
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 |
                       GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3);

    // No pull-up/pull-down
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 |
                      GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);

    // Set alternate function to AF1 (TIM2) for PA0-PA3
    // AFRL handles pins 0-7
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL0 | GPIO_AFRL_AFRL1 |
                       GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= (1 << (0 * 4)) | (1 << (1 * 4)) |
                     (1 << (2 * 4)) | (1 << (3 * 4));
}

static void timer_init(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Stop timer during configuration
    TIM2->CR1 = 0;

    // Calculate prescaler and period for desired frequency
    // APB1 timer clock = 84 MHz (assuming 168 MHz system, APB1 prescaler = 4)
    uint32_t prescaler;
    uint32_t period = PWM_RESOLUTION;

    if (s_config.frequency == MOTORS_PWM_328KHZ) {
        // 84 MHz / 1 / 256 = 328.125 kHz
        prescaler = 0;
    } else {
        // 84 MHz / 7 / 240 = 50 kHz (approximate)
        prescaler = 6;
        period = 239;
    }

    TIM2->PSC = prescaler;
    TIM2->ARR = period;

    // Configure all 4 channels for PWM mode 1
    // OC1M = 110 (PWM mode 1), OC1PE = 1 (preload enable)
    TIM2->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 = (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    // Enable outputs (CC1E, CC2E, CC3E, CC4E)
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E |
                 TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Initialize all compare values to 0 (motors off)
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    // Set up CCR pointers for direct access
    s_ccr[0] = &TIM2->CCR1;
    s_ccr[1] = &TIM2->CCR2;
    s_ccr[2] = &TIM2->CCR3;
    s_ccr[3] = &TIM2->CCR4;

    // Auto-reload preload enable
    TIM2->CR1 = TIM_CR1_ARPE;

    // Generate update event to load prescaler
    TIM2->EGR = TIM_EGR_UG;
}

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

bool motors_init(const motors_config_t *config) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (motors_config_t)MOTORS_CONFIG_DEFAULT;
    }

    s_armed = false;

    // Initialize all PWM values to zero
    for (int i = 0; i < MOTORS_COUNT; i++) {
        s_pwm[i] = 0;
    }

    // Initialize hardware
    gpio_init();
    timer_init();

    s_initialized = true;
    return true;
}

void motors_arm(void) {
    if (!s_initialized) {
        return;
    }

    if (!s_armed) {
        // Ensure motors are at zero before arming
        motors_stop();

        // Enable timer
        TIM2->CR1 |= TIM_CR1_CEN;

        s_armed = true;
    }
}

void motors_disarm(void) {
    if (s_armed) {
        motors_stop();

        // Disable timer
        TIM2->CR1 &= ~TIM_CR1_CEN;

        s_armed = false;
    }
}

bool motors_is_armed(void) {
    return s_armed;
}

void motors_set(const motors_cmd_t *cmd) {
    if (!s_armed) {
        return;
    }

    // Convert and set all channels
    for (int i = 0; i < MOTORS_COUNT; i++) {
        float value = clampf(cmd->motor[i], 0.0f, 1.0f);
        s_pwm[i] = float_to_pwm(value);
        *s_ccr[i] = s_pwm[i];
    }
}

void motors_set_single(uint8_t motor, float value) {
    if (!s_armed || motor >= MOTORS_COUNT) {
        return;
    }

    value = clampf(value, 0.0f, 1.0f);
    s_pwm[motor] = float_to_pwm(value);
    *s_ccr[motor] = s_pwm[motor];
}

void motors_stop(void) {
    for (int i = 0; i < MOTORS_COUNT; i++) {
        s_pwm[i] = 0;
        *s_ccr[i] = 0;
    }
}

void motors_emergency_stop(void) {
    // Immediately stop all motors
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    for (int i = 0; i < MOTORS_COUNT; i++) {
        s_pwm[i] = 0;
    }

    // Disable timer
    TIM2->CR1 &= ~TIM_CR1_CEN;
    s_armed = false;
}

void motors_get_pwm(uint16_t pwm[MOTORS_COUNT]) {
    for (int i = 0; i < MOTORS_COUNT; i++) {
        pwm[i] = s_pwm[i];
    }
}

void motors_set_ratio(uint8_t motor, uint16_t ratio) {
    if (!s_armed || motor >= MOTORS_COUNT) {
        return;
    }

    s_pwm[motor] = ratio;
    *s_ccr[motor] = ratio;
}

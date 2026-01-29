// Motor PWM Driver Implementation for Crazyflie 2.1+
//
// Uses TIM2 and TIM4 for PWM generation matching Bitcraze reference implementation.
// Reference: https://github.com/bitcraze/crazyflie-firmware/blob/master/src/drivers/src/motors_def.c
//
// Motor pin mapping (Crazyflie 2.1+ hardware):
//   M1: PA1,  TIM2_CH2
//   M2: PB11, TIM2_CH4
//   M3: PA15, TIM2_CH1
//   M4: PB9,  TIM4_CH4

#include "motors.h"
#include "stm32f4xx.h"

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// PWM resolution (8-bit for compatibility with Crazyflie firmware)
#define PWM_RESOLUTION 255

// TIM2/TIM4 run at APB1*2 = 84 MHz (assuming 168 MHz system clock)
// For 328 kHz PWM: 84 MHz / 256 = 328.125 kHz
// For 50 kHz PWM: 84 MHz / 1680 = 50 kHz

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;
static bool s_armed = false;
static motors_config_t s_config;
static uint16_t s_pwm[MOTORS_COUNT] = {0, 0, 0, 0};

// CCR register pointers for direct access
// Index: 0=M1, 1=M2, 2=M3, 3=M4
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
// GPIO Initialization
// ----------------------------------------------------------------------------

static void gpio_init(void) {
    // Enable GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // ========================================================================
    // PA1 - M1 (TIM2_CH2, AF1)
    // ========================================================================
    GPIOA->MODER &= ~GPIO_MODER_MODER1;
    GPIOA->MODER |= GPIO_MODER_MODER1_1;      // Alternate function
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1; // High speed
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;       // No pull
    GPIOA->AFR[0] &= ~(0xFU << (1 * 4));
    GPIOA->AFR[0] |= (1U << (1 * 4)); // AF1 = TIM2

    // ========================================================================
    // PA15 - M3 (TIM2_CH1, AF1)
    // ========================================================================
    GPIOA->MODER &= ~GPIO_MODER_MODER15;
    GPIOA->MODER |= GPIO_MODER_MODER15_1;      // Alternate function
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15; // High speed
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR15;       // No pull
    // PA15 uses AFRH (AFR[1]), pin 15 is index 7 in AFRH
    GPIOA->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
    GPIOA->AFR[1] |= (1U << ((15 - 8) * 4)); // AF1 = TIM2

    // ========================================================================
    // PB11 - M2 (TIM2_CH4, AF1)
    // ========================================================================
    GPIOB->MODER &= ~GPIO_MODER_MODER11;
    GPIOB->MODER |= GPIO_MODER_MODER11_1;      // Alternate function
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11; // High speed
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR11;       // No pull
    // PB11 uses AFRH (AFR[1]), pin 11 is index 3 in AFRH
    GPIOB->AFR[1] &= ~(0xFU << ((11 - 8) * 4));
    GPIOB->AFR[1] |= (1U << ((11 - 8) * 4)); // AF1 = TIM2

    // ========================================================================
    // PB9 - M4 (TIM4_CH4, AF2)
    // ========================================================================
    GPIOB->MODER &= ~GPIO_MODER_MODER9;
    GPIOB->MODER |= GPIO_MODER_MODER9_1;      // Alternate function
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9; // High speed
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR9;       // No pull
    // PB9 uses AFRH (AFR[1]), pin 9 is index 1 in AFRH
    GPIOB->AFR[1] &= ~(0xFU << ((9 - 8) * 4));
    GPIOB->AFR[1] |= (2U << ((9 - 8) * 4)); // AF2 = TIM4
}

// ----------------------------------------------------------------------------
// Timer Initialization
// ----------------------------------------------------------------------------

static void timer_init(void) {
    // Enable timer clocks
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;

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

    // ========================================================================
    // TIM2 Configuration (M1=CH2, M2=CH4, M3=CH1)
    // ========================================================================
    TIM2->CR1 = 0; // Stop timer during configuration
    TIM2->PSC = prescaler;
    TIM2->ARR = period;

    // Configure channels for PWM mode 1 with preload
    // CH1 (M3), CH2 (M1): CCMR1
    // OC1M = 110 (PWM mode 1), OC1PE = 1 (preload enable)
    TIM2->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | // CH1 = M3
                  (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;  // CH2 = M1

    // CH4 (M2): CCMR2
    TIM2->CCMR2 = (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; // CH4 = M2

    // Enable outputs (CC1E, CC2E, CC4E)
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC4E;

    // Initialize compare values to 0 (motors off)
    TIM2->CCR1 = 0; // M3
    TIM2->CCR2 = 0; // M1
    TIM2->CCR4 = 0; // M2

    // Auto-reload preload enable
    TIM2->CR1 = TIM_CR1_ARPE;

    // Generate update event to load prescaler
    TIM2->EGR = TIM_EGR_UG;

    // ========================================================================
    // TIM4 Configuration (M4=CH4)
    // ========================================================================
    TIM4->CR1 = 0; // Stop timer during configuration
    TIM4->PSC = prescaler;
    TIM4->ARR = period;

    // Configure CH4 for PWM mode 1 with preload
    TIM4->CCMR2 = (6 << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; // CH4 = M4

    // Enable output (CC4E)
    TIM4->CCER = TIM_CCER_CC4E;

    // Initialize compare value to 0 (motor off)
    TIM4->CCR4 = 0; // M4

    // Auto-reload preload enable
    TIM4->CR1 = TIM_CR1_ARPE;

    // Generate update event to load prescaler
    TIM4->EGR = TIM_EGR_UG;

    // ========================================================================
    // Set up CCR pointers for direct access
    // Motor index: 0=M1, 1=M2, 2=M3, 3=M4
    // ========================================================================
    s_ccr[0] = &TIM2->CCR2; // M1 = TIM2_CH2
    s_ccr[1] = &TIM2->CCR4; // M2 = TIM2_CH4
    s_ccr[2] = &TIM2->CCR1; // M3 = TIM2_CH1
    s_ccr[3] = &TIM4->CCR4; // M4 = TIM4_CH4
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

        // Enable both timers
        TIM2->CR1 |= TIM_CR1_CEN;
        TIM4->CR1 |= TIM_CR1_CEN;

        s_armed = true;
    }
}

void motors_disarm(void) {
    if (s_armed) {
        motors_stop();

        // Disable both timers
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM4->CR1 &= ~TIM_CR1_CEN;

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
    TIM2->CCR1 = 0; // M3
    TIM2->CCR2 = 0; // M1
    TIM2->CCR4 = 0; // M2
    TIM4->CCR4 = 0; // M4

    for (int i = 0; i < MOTORS_COUNT; i++) {
        s_pwm[i] = 0;
    }

    // Disable both timers
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;

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

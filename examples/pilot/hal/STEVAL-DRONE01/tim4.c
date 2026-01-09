// TIM4 PWM driver for STM32F401 (STEVAL-DRONE01)
//
// Direct register access implementation for motor PWM control.
// Uses center-aligned PWM mode for symmetric switching.

#include "tim4.h"
#include "system_config.h"
#include "gpio_config.h"

// ----------------------------------------------------------------------------
// TIM4 Register Definitions
// ----------------------------------------------------------------------------

#define TIM4_BASE           0x40000800U

#define TIM4_CR1            (*(volatile uint32_t *)(TIM4_BASE + 0x00))
#define TIM4_CR2            (*(volatile uint32_t *)(TIM4_BASE + 0x04))
#define TIM4_SMCR           (*(volatile uint32_t *)(TIM4_BASE + 0x08))
#define TIM4_DIER           (*(volatile uint32_t *)(TIM4_BASE + 0x0C))
#define TIM4_SR             (*(volatile uint32_t *)(TIM4_BASE + 0x10))
#define TIM4_EGR            (*(volatile uint32_t *)(TIM4_BASE + 0x14))
#define TIM4_CCMR1          (*(volatile uint32_t *)(TIM4_BASE + 0x18))
#define TIM4_CCMR2          (*(volatile uint32_t *)(TIM4_BASE + 0x1C))
#define TIM4_CCER           (*(volatile uint32_t *)(TIM4_BASE + 0x20))
#define TIM4_CNT            (*(volatile uint32_t *)(TIM4_BASE + 0x24))
#define TIM4_PSC            (*(volatile uint32_t *)(TIM4_BASE + 0x28))
#define TIM4_ARR            (*(volatile uint32_t *)(TIM4_BASE + 0x2C))
#define TIM4_CCR1           (*(volatile uint32_t *)(TIM4_BASE + 0x34))
#define TIM4_CCR2           (*(volatile uint32_t *)(TIM4_BASE + 0x38))
#define TIM4_CCR3           (*(volatile uint32_t *)(TIM4_BASE + 0x3C))
#define TIM4_CCR4           (*(volatile uint32_t *)(TIM4_BASE + 0x40))
#define TIM4_DCR            (*(volatile uint32_t *)(TIM4_BASE + 0x48))
#define TIM4_DMAR           (*(volatile uint32_t *)(TIM4_BASE + 0x4C))

// TIM_CR1 bits
#define TIM_CR1_CEN         (1U << 0)   // Counter enable
#define TIM_CR1_UDIS        (1U << 1)   // Update disable
#define TIM_CR1_URS         (1U << 2)   // Update request source
#define TIM_CR1_OPM         (1U << 3)   // One pulse mode
#define TIM_CR1_DIR         (1U << 4)   // Direction (0=up, 1=down)
#define TIM_CR1_CMS_MASK    (3U << 5)   // Center-aligned mode
#define TIM_CR1_CMS_EDGE    (0U << 5)   // Edge-aligned mode
#define TIM_CR1_CMS_CENTER1 (1U << 5)   // Center-aligned mode 1
#define TIM_CR1_CMS_CENTER2 (2U << 5)   // Center-aligned mode 2
#define TIM_CR1_CMS_CENTER3 (3U << 5)   // Center-aligned mode 3
#define TIM_CR1_ARPE        (1U << 7)   // Auto-reload preload enable

// TIM_EGR bits
#define TIM_EGR_UG          (1U << 0)   // Update generation

// TIM_CCMR1/2 bits (output compare mode)
#define TIM_CCMR_OC1M_MASK  (7U << 4)
#define TIM_CCMR_OC1M_PWM1  (6U << 4)   // PWM mode 1 (active when CNT < CCR)
#define TIM_CCMR_OC1M_PWM2  (7U << 4)   // PWM mode 2 (active when CNT > CCR)
#define TIM_CCMR_OC1PE      (1U << 3)   // Output compare 1 preload enable
#define TIM_CCMR_OC1FE      (1U << 2)   // Output compare 1 fast enable

#define TIM_CCMR_OC2M_MASK  (7U << 12)
#define TIM_CCMR_OC2M_PWM1  (6U << 12)
#define TIM_CCMR_OC2M_PWM2  (7U << 12)
#define TIM_CCMR_OC2PE      (1U << 11)
#define TIM_CCMR_OC2FE      (1U << 10)

// TIM_CCER bits
#define TIM_CCER_CC1E       (1U << 0)   // Capture/Compare 1 output enable
#define TIM_CCER_CC1P       (1U << 1)   // Capture/Compare 1 polarity (0=active high)
#define TIM_CCER_CC2E       (1U << 4)
#define TIM_CCER_CC2P       (1U << 5)
#define TIM_CCER_CC3E       (1U << 8)
#define TIM_CCER_CC3P       (1U << 9)
#define TIM_CCER_CC4E       (1U << 12)
#define TIM_CCER_CC4P       (1U << 13)

// GPIO register definitions for port D (alternative TIM4 pins)
#define GPIOD_BASE          0x40020C00U
#define GPIOD_MODER         (*(volatile uint32_t *)(GPIOD_BASE + 0x00))
#define GPIOD_OTYPER        (*(volatile uint32_t *)(GPIOD_BASE + 0x04))
#define GPIOD_OSPEEDR       (*(volatile uint32_t *)(GPIOD_BASE + 0x08))
#define GPIOD_PUPDR         (*(volatile uint32_t *)(GPIOD_BASE + 0x0C))
#define GPIOD_AFRL          (*(volatile uint32_t *)(GPIOD_BASE + 0x20))
#define GPIOD_AFRH          (*(volatile uint32_t *)(GPIOD_BASE + 0x24))

// ----------------------------------------------------------------------------
// Static state
// ----------------------------------------------------------------------------

static tim4_config_t s_config;
static bool s_enabled = false;
static uint32_t s_frequency = 0;

// Pointers to CCR registers for fast access
static volatile uint32_t *const s_ccr[4] = {
    &TIM4_CCR1, &TIM4_CCR2, &TIM4_CCR3, &TIM4_CCR4
};

// ----------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------

static void init_gpio_pb8_pb9(void) {
    // Initialize PB8 (TIM4_CH3) and PB9 (TIM4_CH4)
    // These are already configured by gpio_init_tim4_pwm() but we call it anyway
    system_enable_gpio('B');

    // PB8 - TIM4_CH3, AF2
    gpio_set_mode('B', 8, GPIO_MODE_AF);
    gpio_set_otype('B', 8, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed('B', 8, GPIO_SPEED_HIGH);
    gpio_set_pupd('B', 8, GPIO_PUPD_NONE);
    gpio_set_af('B', 8, 2);  // AF2 = TIM4

    // PB9 - TIM4_CH4, AF2
    gpio_set_mode('B', 9, GPIO_MODE_AF);
    gpio_set_otype('B', 9, GPIO_OTYPE_PUSHPULL);
    gpio_set_speed('B', 9, GPIO_SPEED_HIGH);
    gpio_set_pupd('B', 9, GPIO_PUPD_NONE);
    gpio_set_af('B', 9, 2);  // AF2 = TIM4
}

static void init_gpio_pd12_pd15(void) {
    // Initialize PD12-PD15 for TIM4 CH1-CH4 (alternative pins)
    system_enable_gpio('D');

    // All 4 pins: AF2 (TIM4), push-pull, high speed
    for (int pin = 12; pin <= 15; pin++) {
        gpio_set_mode('D', pin, GPIO_MODE_AF);
        gpio_set_otype('D', pin, GPIO_OTYPE_PUSHPULL);
        gpio_set_speed('D', pin, GPIO_SPEED_HIGH);
        gpio_set_pupd('D', pin, GPIO_PUPD_NONE);
        gpio_set_af('D', pin, 2);  // AF2 = TIM4
    }
}

static void init_gpio_pb6_pb9(void) {
    // Initialize PB6-PB9 for TIM4 CH1-CH4
    // WARNING: PB6/PB7 conflict with I2C1
    system_enable_gpio('B');

    for (int pin = 6; pin <= 9; pin++) {
        gpio_set_mode('B', pin, GPIO_MODE_AF);
        gpio_set_otype('B', pin, GPIO_OTYPE_PUSHPULL);
        gpio_set_speed('B', pin, GPIO_SPEED_HIGH);
        gpio_set_pupd('B', pin, GPIO_PUPD_NONE);
        gpio_set_af('B', pin, 2);  // AF2 = TIM4
    }
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

void tim4_init(const tim4_config_t *config) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (tim4_config_t)TIM4_CONFIG_DEFAULT;
    }

    // Enable TIM4 clock
    system_enable_tim4();

    // Configure GPIO pins based on configuration
    switch (s_config.pin_config) {
        case TIM4_PINS_PB6_PB9:
            init_gpio_pb6_pb9();
            break;
        case TIM4_PINS_PD12_PD15:
            init_gpio_pd12_pd15();
            break;
        case TIM4_PINS_PB8_PB9_ONLY:
        default:
            init_gpio_pb8_pb9();
            break;
    }

    // Disable timer during configuration
    TIM4_CR1 = 0;

    // -------------------------------------------------------------------------
    // Calculate prescaler and ARR for desired frequency
    // -------------------------------------------------------------------------
    // TIM4 is on APB1 (42MHz), but timer clock is 2x APB1 = 84MHz when APB1 prescaler > 1
    // PWM frequency = TIM_CLK / ((PSC + 1) * (ARR + 1))
    // For 20kHz with 1024 resolution: PSC = (84MHz / (20kHz * 1024)) - 1 = 3.1 ≈ 4

    uint32_t tim_clk = PCLK1_FREQ * 2;  // Timer clock = 84MHz (APB1 * 2)
    uint32_t prescaler = (tim_clk / ((uint32_t)s_config.frequency * TIM4_PWM_RESOLUTION)) - 1;

    // Clamp prescaler to valid range
    if (prescaler > 65535) prescaler = 65535;

    // Calculate actual frequency achieved
    s_frequency = tim_clk / ((prescaler + 1) * TIM4_PWM_RESOLUTION);

    // Set prescaler and auto-reload value
    TIM4_PSC = prescaler;
    TIM4_ARR = TIM4_PWM_RESOLUTION - 1;

    // -------------------------------------------------------------------------
    // Configure PWM mode for each channel
    // -------------------------------------------------------------------------
    // PWM mode 1: Output is active when CNT < CCR
    // Preload enabled: CCR update takes effect at next update event

    uint32_t ccmr1 = 0;
    uint32_t ccmr2 = 0;

    // Channel 1 (OC1)
    if (s_config.ch1_enable) {
        ccmr1 |= TIM_CCMR_OC1M_PWM1 | TIM_CCMR_OC1PE;
    }

    // Channel 2 (OC2)
    if (s_config.ch2_enable) {
        ccmr1 |= TIM_CCMR_OC2M_PWM1 | TIM_CCMR_OC2PE;
    }

    // Channel 3 (OC3 uses same bit positions as OC1 in CCMR2)
    if (s_config.ch3_enable) {
        ccmr2 |= TIM_CCMR_OC1M_PWM1 | TIM_CCMR_OC1PE;
    }

    // Channel 4 (OC4 uses same bit positions as OC2 in CCMR2)
    if (s_config.ch4_enable) {
        ccmr2 |= TIM_CCMR_OC2M_PWM1 | TIM_CCMR_OC2PE;
    }

    TIM4_CCMR1 = ccmr1;
    TIM4_CCMR2 = ccmr2;

    // -------------------------------------------------------------------------
    // Initialize all CCR values to 0 (motors off)
    // -------------------------------------------------------------------------
    TIM4_CCR1 = 0;
    TIM4_CCR2 = 0;
    TIM4_CCR3 = 0;
    TIM4_CCR4 = 0;

    // -------------------------------------------------------------------------
    // Configure control register
    // -------------------------------------------------------------------------
    // Edge-aligned mode, auto-reload preload enabled
    TIM4_CR1 = TIM_CR1_ARPE;

    // Generate update event to load prescaler and ARR
    TIM4_EGR = TIM_EGR_UG;

    // Clear any pending flags
    TIM4_SR = 0;

    // PWM output is disabled until tim4_enable() is called
    s_enabled = false;
}

void tim4_deinit(void) {
    // Disable timer
    TIM4_CR1 = 0;

    // Disable all outputs
    TIM4_CCER = 0;

    // Reset all CCR values
    TIM4_CCR1 = 0;
    TIM4_CCR2 = 0;
    TIM4_CCR3 = 0;
    TIM4_CCR4 = 0;

    s_enabled = false;
}

void tim4_set_duty(tim4_channel_t channel, float duty) {
    // Clamp duty cycle to 0.0 - 1.0
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    // Convert to raw value
    uint16_t value = (uint16_t)(duty * (TIM4_PWM_RESOLUTION - 1));

    // Set CCR register
    if (channel <= TIM4_CH4) {
        *s_ccr[channel] = value;
    }
}

void tim4_set_raw(tim4_channel_t channel, uint16_t value) {
    // Clamp to valid range
    if (value >= TIM4_PWM_RESOLUTION) {
        value = TIM4_PWM_RESOLUTION - 1;
    }

    // Set CCR register
    if (channel <= TIM4_CH4) {
        *s_ccr[channel] = value;
    }
}

void tim4_set_all(const float duties[4]) {
    for (int i = 0; i < 4; i++) {
        float duty = duties[i];
        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;
        *s_ccr[i] = (uint16_t)(duty * (TIM4_PWM_RESOLUTION - 1));
    }
}

void tim4_set_all_raw(const uint16_t values[4]) {
    for (int i = 0; i < 4; i++) {
        uint16_t value = values[i];
        if (value >= TIM4_PWM_RESOLUTION) {
            value = TIM4_PWM_RESOLUTION - 1;
        }
        *s_ccr[i] = value;
    }
}

void tim4_enable(void) {
    // Enable output for configured channels
    uint32_t ccer = 0;

    if (s_config.ch1_enable) ccer |= TIM_CCER_CC1E;
    if (s_config.ch2_enable) ccer |= TIM_CCER_CC2E;
    if (s_config.ch3_enable) ccer |= TIM_CCER_CC3E;
    if (s_config.ch4_enable) ccer |= TIM_CCER_CC4E;

    TIM4_CCER = ccer;

    // Enable counter
    TIM4_CR1 |= TIM_CR1_CEN;

    s_enabled = true;
}

void tim4_disable(void) {
    // Disable counter
    TIM4_CR1 &= ~TIM_CR1_CEN;

    // Disable all outputs
    TIM4_CCER = 0;

    // Reset all CCR values to ensure outputs go low
    TIM4_CCR1 = 0;
    TIM4_CCR2 = 0;
    TIM4_CCR3 = 0;
    TIM4_CCR4 = 0;

    s_enabled = false;
}

bool tim4_is_enabled(void) {
    return s_enabled;
}

uint32_t tim4_get_frequency(void) {
    return s_frequency;
}

uint16_t tim4_get_resolution(void) {
    return TIM4_PWM_RESOLUTION;
}

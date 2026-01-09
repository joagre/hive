// Motor PWM driver for STEVAL-DRONE01
//
// TIM4 PWM output for 4 brushed DC motors.
// Note: Using CH3/CH4 only (PB8/PB9) since CH1/CH2 (PB6/PB7) conflict with I2C1.
// For full 4-motor support, use alternative pins (PD12-PD15) or separate timer.

#include "motors.h"
#include "tim4.h"

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Motor to TIM4 channel mapping
// With I2C1 using PB6/PB7, we only have TIM4 CH3/CH4 available on PB8/PB9
// For full quad support, would need alternative pins or timer
//
// Current mapping (2 motors on TIM4):
//   M3 (front-right): TIM4_CH3 (PB8)
//   M4 (rear-right):  TIM4_CH4 (PB9)
//
// Full mapping (if using PD12-PD15 or not using I2C1):
//   M1 (rear-left):   TIM4_CH1
//   M2 (front-left):  TIM4_CH2
//   M3 (front-right): TIM4_CH3
//   M4 (rear-right):  TIM4_CH4

static const tim4_channel_t motor_channel[MOTORS_COUNT] = {
    TIM4_CH1,   // M1 - rear-left
    TIM4_CH2,   // M2 - front-left
    TIM4_CH3,   // M3 - front-right
    TIM4_CH4    // M4 - rear-right
};

// ----------------------------------------------------------------------------
// Static state
// ----------------------------------------------------------------------------

static motors_config_t s_config;
static bool s_armed = false;
static uint16_t s_pwm[MOTORS_COUNT] = {0, 0, 0, 0};
static bool s_use_all_channels = false;

// ----------------------------------------------------------------------------
// Helper functions
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
// Public API
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

    // Determine TIM4 configuration based on motor config
    // Default: Only CH3/CH4 to avoid I2C1 conflict
    tim4_config_t tim_config = {
        .frequency = (tim4_pwm_freq_t)s_config.frequency_hz,
        .pin_config = TIM4_PINS_PB8_PB9_ONLY,
        .ch1_enable = false,
        .ch2_enable = false,
        .ch3_enable = true,
        .ch4_enable = true
    };

    // Update max_pulse to match TIM4 resolution if using default
    if (s_config.max_pulse == 1000) {
        s_config.max_pulse = TIM4_PWM_RESOLUTION - 1;
    }

    s_use_all_channels = false;

    // Initialize TIM4
    tim4_init(&tim_config);

    // Set all channels to zero
    for (int i = 0; i < MOTORS_COUNT; i++) {
        tim4_set_raw(motor_channel[i], 0);
    }

    return true;
}

bool motors_init_full(const motors_config_t *config, bool use_port_d) {
    // Full 4-motor initialization using alternative pins
    if (config) {
        s_config = *config;
    } else {
        s_config = (motors_config_t)MOTORS_CONFIG_DEFAULT;
    }

    s_armed = false;

    for (int i = 0; i < MOTORS_COUNT; i++) {
        s_pwm[i] = 0;
    }

    tim4_config_t tim_config = {
        .frequency = (tim4_pwm_freq_t)s_config.frequency_hz,
        .pin_config = use_port_d ? TIM4_PINS_PD12_PD15 : TIM4_PINS_PB6_PB9,
        .ch1_enable = true,
        .ch2_enable = true,
        .ch3_enable = true,
        .ch4_enable = true
    };

    if (s_config.max_pulse == 1000) {
        s_config.max_pulse = TIM4_PWM_RESOLUTION - 1;
    }

    s_use_all_channels = true;

    tim4_init(&tim_config);

    for (int i = 0; i < MOTORS_COUNT; i++) {
        tim4_set_raw(motor_channel[i], 0);
    }

    return true;
}

void motors_arm(void) {
    if (!s_armed) {
        // Ensure motors are at zero before arming
        motors_stop();
        tim4_enable();
        s_armed = true;
    }
}

void motors_disarm(void) {
    if (s_armed) {
        motors_stop();
        tim4_disable();
        s_armed = false;
    }
}

bool motors_is_armed(void) {
    return s_armed;
}

void motors_set(const motors_cmd_t *cmd) {
    if (!s_armed) {
        return;  // Ignore commands when disarmed
    }

    // Convert float commands to PWM values and set
    float duties[4];
    for (int i = 0; i < MOTORS_COUNT; i++) {
        float value = clampf(cmd->motor[i], 0.0f, 1.0f);
        duties[i] = value;
        s_pwm[i] = float_to_pwm(value);
    }

    // Use tim4_set_all for efficiency
    tim4_set_all(duties);
}

void motors_set_single(uint8_t motor, float value) {
    if (!s_armed || motor >= MOTORS_COUNT) {
        return;
    }

    value = clampf(value, 0.0f, 1.0f);
    s_pwm[motor] = float_to_pwm(value);
    tim4_set_duty(motor_channel[motor], value);
}

void motors_stop(void) {
    for (int i = 0; i < MOTORS_COUNT; i++) {
        s_pwm[i] = 0;
        tim4_set_raw(motor_channel[i], 0);
    }
}

void motors_emergency_stop(void) {
    // Immediately stop and disarm
    for (int i = 0; i < MOTORS_COUNT; i++) {
        s_pwm[i] = 0;
        tim4_set_raw(motor_channel[i], 0);
    }
    tim4_disable();
    s_armed = false;
}

void motors_get_pwm(uint16_t pwm[MOTORS_COUNT]) {
    for (int i = 0; i < MOTORS_COUNT; i++) {
        pwm[i] = s_pwm[i];
    }
}

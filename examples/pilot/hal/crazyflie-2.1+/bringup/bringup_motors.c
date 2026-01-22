// Crazyflie 2.1+ Bring-Up - Motor Tests

#include "bringup_motors.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"

// PWM frequency: 328 Hz (good for brushed motors)
// Timer clock = 84 MHz (APB1 x2 for timers when APB1 prescaler != 1)
// PWM period = 84 MHz / 328 Hz = 256097 -> use 256000 for round numbers
// Prescaler = 1, ARR = 256000 (too large for 16-bit)
// Use prescaler = 4, ARR = 64000 (fits in 32-bit TIM2)
#define MOTOR_PWM_PRESCALER 4
#define MOTOR_PWM_PERIOD 64000

static bool s_armed = false;

static const motor_info_t s_motor_info[MOTOR_COUNT] = {
    {"M1", "front-left", "CCW"},
    {"M2", "front-right", "CW"},
    {"M3", "rear-right", "CCW"},
    {"M4", "rear-left", "CW"},
};

// Simple delay
static void delay_ms(uint32_t ms) {
    volatile uint32_t count = ms * 42000;
    while (count--)
        ;
}

void motors_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure PA0-PA3 as alternate function (TIM2 CH1-CH4)
    GPIOA->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
                      GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 |
                     GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
    GPIOA->AFR[0] &= ~((0xFU << (0 * 4)) | (0xFU << (1 * 4)) |
                       (0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA->AFR[0] |= (1U << (0 * 4)) | (1U << (1 * 4)) | (1U << (2 * 4)) |
                     (1U << (3 * 4)); // AF1 = TIM2
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 |
                      GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3;

    // Configure TIM2 for PWM
    TIM2->CR1 = 0;
    TIM2->PSC = MOTOR_PWM_PRESCALER - 1;
    TIM2->ARR = MOTOR_PWM_PERIOD - 1;

    // PWM mode 1 on all channels, preload enabled
    TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE |
                  TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;

    // Enable outputs
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Initialize all channels to 0
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    // Enable timer
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    s_armed = false;
}

void motor_set(motor_id_t motor, float duty) {
    if (!s_armed || motor >= MOTOR_COUNT) {
        return;
    }

    // Clamp duty cycle
    if (duty < 0.0f)
        duty = 0.0f;
    if (duty > 1.0f)
        duty = 1.0f;

    uint32_t ccr = (uint32_t)(duty * (MOTOR_PWM_PERIOD - 1));

    switch (motor) {
    case MOTOR_M1:
        TIM2->CCR1 = ccr;
        break;
    case MOTOR_M2:
        TIM2->CCR2 = ccr;
        break;
    case MOTOR_M3:
        TIM2->CCR3 = ccr;
        break;
    case MOTOR_M4:
        TIM2->CCR4 = ccr;
        break;
    default:
        break;
    }
}

void motors_stop(void) {
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
}

const motor_info_t *motor_get_info(motor_id_t motor) {
    if (motor >= MOTOR_COUNT) {
        return NULL;
    }
    return &s_motor_info[motor];
}

void motors_arm(void) {
    s_armed = true;
}

void motors_disarm(void) {
    motors_stop();
    s_armed = false;
}

bool motors_is_armed(void) {
    return s_armed;
}

bool motors_run_test(void) {
    swo_puts("\n");
    swo_puts("[MOTOR] !!! WARNING: REMOVE PROPELLERS !!!\n");
    swo_puts("[MOTOR] Press ENTER to continue or 's' to skip...\n");

    // Wait for user input
    int c = swo_getc_timeout(30000); // 30 second timeout
    if (c < 0 || c == 's' || c == 'S') {
        swo_puts("[MOTOR] Motor test skipped\n");
        return false;
    }

    swo_puts("[MOTOR] Starting motor test...\n");

    motors_arm();

    // Test each motor
    for (int i = 0; i < MOTOR_COUNT; i++) {
        const motor_info_t *info = motor_get_info((motor_id_t)i);

        swo_printf("[MOTOR] Spinning %s (%s, %s) at 10%%...", info->name,
                   info->position, info->rotation);

        motor_set((motor_id_t)i, 0.10f);
        delay_ms(1000);
        motor_set((motor_id_t)i, 0.0f);
        delay_ms(500);

        swo_puts(" OK\n");
    }

    // Test all motors together at low power
    swo_puts("[MOTOR] Spinning all motors at 8%...");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set((motor_id_t)i, 0.08f);
    }
    delay_ms(1500);
    motors_stop();
    swo_puts(" OK\n");

    motors_disarm();

    swo_puts("[MOTOR] All motors tested... OK\n");
    return true;
}

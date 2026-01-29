// Crazyflie 2.1+ Bring-Up - Motor Tests
//
// Motor pin mapping (Bitcraze reference):
//   M1: PA1,  TIM2_CH2
//   M2: PB11, TIM2_CH4
//   M3: PA15, TIM2_CH1
//   M4: PB9,  TIM4_CH4

#include "bringup_motors.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"

// PWM frequency: 328 Hz (good for brushed motors)
// Timer clock = 84 MHz (APB1 x2 for timers when APB1 prescaler != 1)
// PWM period = 84 MHz / 328 Hz = 256097 -> use 256000 for round numbers
// Prescaler = 4, ARR = 64000 (fits in 32-bit TIM2 and 16-bit TIM4)
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
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;

    // ========================================================================
    // GPIO Configuration
    // ========================================================================

    // PA1 - M1 (TIM2_CH2, AF1)
    GPIOA->MODER &= ~GPIO_MODER_MODER1;
    GPIOA->MODER |= GPIO_MODER_MODER1_1; // Alternate function
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;
    GPIOA->AFR[0] &= ~(0xFU << (1 * 4));
    GPIOA->AFR[0] |= (1U << (1 * 4)); // AF1 = TIM2

    // PA15 - M3 (TIM2_CH1, AF1)
    GPIOA->MODER &= ~GPIO_MODER_MODER15;
    GPIOA->MODER |= GPIO_MODER_MODER15_1; // Alternate function
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;
    GPIOA->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
    GPIOA->AFR[1] |= (1U << ((15 - 8) * 4)); // AF1 = TIM2

    // PB11 - M2 (TIM2_CH4, AF1)
    GPIOB->MODER &= ~GPIO_MODER_MODER11;
    GPIOB->MODER |= GPIO_MODER_MODER11_1; // Alternate function
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11;
    GPIOB->AFR[1] &= ~(0xFU << ((11 - 8) * 4));
    GPIOB->AFR[1] |= (1U << ((11 - 8) * 4)); // AF1 = TIM2

    // PB9 - M4 (TIM4_CH4, AF2)
    GPIOB->MODER &= ~GPIO_MODER_MODER9;
    GPIOB->MODER |= GPIO_MODER_MODER9_1; // Alternate function
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;
    GPIOB->AFR[1] &= ~(0xFU << ((9 - 8) * 4));
    GPIOB->AFR[1] |= (2U << ((9 - 8) * 4)); // AF2 = TIM4

    // ========================================================================
    // TIM2 Configuration (M1=CH2, M2=CH4, M3=CH1)
    // ========================================================================
    TIM2->CR1 = 0;
    TIM2->PSC = MOTOR_PWM_PRESCALER - 1;
    TIM2->ARR = MOTOR_PWM_PERIOD - 1;

    // PWM mode 1 on CH1 (M3) and CH2 (M1), preload enabled
    TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;

    // PWM mode 1 on CH4 (M2), preload enabled
    TIM2->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;

    // Enable outputs (CH1, CH2, CH4)
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC4E;

    // Initialize channels to 0
    TIM2->CCR1 = 0; // M3
    TIM2->CCR2 = 0; // M1
    TIM2->CCR4 = 0; // M2

    // Enable timer
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    // ========================================================================
    // TIM4 Configuration (M4=CH4)
    // ========================================================================
    TIM4->CR1 = 0;
    TIM4->PSC = MOTOR_PWM_PRESCALER - 1;
    TIM4->ARR = MOTOR_PWM_PERIOD - 1;

    // PWM mode 1 on CH4 (M4), preload enabled
    TIM4->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;

    // Enable output (CH4)
    TIM4->CCER = TIM_CCER_CC4E;

    // Initialize channel to 0
    TIM4->CCR4 = 0; // M4

    // Enable timer
    TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

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
        TIM2->CCR2 = ccr; // M1 = TIM2_CH2
        break;
    case MOTOR_M2:
        TIM2->CCR4 = ccr; // M2 = TIM2_CH4
        break;
    case MOTOR_M3:
        TIM2->CCR1 = ccr; // M3 = TIM2_CH1
        break;
    case MOTOR_M4:
        TIM4->CCR4 = ccr; // M4 = TIM4_CH4
        break;
    default:
        break;
    }
}

void motors_stop(void) {
    TIM2->CCR1 = 0; // M3
    TIM2->CCR2 = 0; // M1
    TIM2->CCR4 = 0; // M2
    TIM4->CCR4 = 0; // M4
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
    swo_puts("\n");
    swo_puts("[MOTOR] Motor layout (X-config, viewed from above):\n");
    swo_puts("[MOTOR]\n");
    swo_puts("[MOTOR]           FRONT\n");
    swo_puts("[MOTOR]       M1(CCW)  M2(CW)\n");
    swo_puts("[MOTOR]           +--+\n");
    swo_puts("[MOTOR]           |  |\n");
    swo_puts("[MOTOR]           +--+\n");
    swo_puts("[MOTOR]       M4(CW)  M3(CCW)\n");
    swo_puts("[MOTOR]           REAR\n");
    swo_puts("[MOTOR]\n");
    swo_puts("[MOTOR] CCW = Counter-clockwise (viewed from above)\n");
    swo_puts("[MOTOR] CW  = Clockwise (viewed from above)\n");
    swo_puts("\n");

    motors_arm();

    // Test each motor individually with rotation verification
    for (int i = 0; i < MOTOR_COUNT; i++) {
        const motor_info_t *info = motor_get_info((motor_id_t)i);

        swo_puts("----------------------------------------\n");
        swo_printf("[MOTOR] Testing %s (%s)\n", info->name, info->position);
        swo_printf("[MOTOR] Expected rotation: %s\n", info->rotation);
        swo_puts("[MOTOR] Press ENTER to spin, 's' to skip this motor...\n");

        c = swo_getc_timeout(30000);
        if (c == 's' || c == 'S') {
            swo_printf("[MOTOR] %s skipped\n", info->name);
            continue;
        }
        if (c < 0) {
            swo_puts("[MOTOR] Timeout - aborting test\n");
            motors_disarm();
            return false;
        }

        // Slow ramp up so rotation direction is clearly visible
        swo_printf("[MOTOR] Ramping up %s...\n", info->name);
        for (int pct = 5; pct <= 15; pct += 2) {
            motor_set((motor_id_t)i, pct / 100.0f);
            delay_ms(300);
        }

        swo_printf("[MOTOR] %s spinning at 15%% - verify %s rotation\n",
                   info->name, info->rotation);
        swo_puts("[MOTOR] Press 'y' if correct, 'n' if wrong, ENTER to "
                 "continue...\n");

        c = swo_getc_timeout(10000);

        // Ramp down
        for (int pct = 15; pct >= 0; pct -= 3) {
            motor_set((motor_id_t)i, pct / 100.0f);
            delay_ms(100);
        }
        motor_set((motor_id_t)i, 0.0f);

        if (c == 'n' || c == 'N') {
            swo_printf("[MOTOR] !!! %s rotation INCORRECT - check wiring !!!\n",
                       info->name);
        } else if (c == 'y' || c == 'Y') {
            swo_printf("[MOTOR] %s rotation confirmed OK\n", info->name);
        } else {
            swo_printf("[MOTOR] %s test complete\n", info->name);
        }

        delay_ms(500);
    }

    swo_puts("----------------------------------------\n");
    swo_puts("[MOTOR] Individual tests complete\n");
    swo_puts("\n");

    // Test all motors together
    swo_puts(
        "[MOTOR] Press ENTER to spin all motors together, 's' to skip...\n");
    c = swo_getc_timeout(30000);

    if (c != 's' && c != 'S' && c >= 0) {
        swo_puts("[MOTOR] Spinning all motors at 10%...\n");
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motor_set((motor_id_t)i, 0.10f);
        }
        delay_ms(2000);
        motors_stop();
        swo_puts("[MOTOR] All motors stopped\n");
    }

    motors_disarm();

    swo_puts("\n");
    swo_puts("[MOTOR] Motor test complete\n");
    return true;
}

/**
 * Thrust calibration test for STEVAL-FCU001V1
 *
 * Runs all 4 motors at equal thrust for 5 seconds to calibrate hover thrust.
 *
 * Usage:
 *   1. REMOVE PROPELLERS! (or secure drone in test rig)
 *   2. Build: make TEST=thrust_test
 *   3. Flash: make TEST=thrust_test flash
 *   4. Connect serial at 115200 baud to see output
 *   5. Press reset to run test
 *   6. Increase TEST_THRUST, reflash, repeat until drone lifts
 *   7. Set HAL_BASE_THRUST in hal_config.h to ~90% of liftoff thrust
 *
 * LED feedback:
 *   2 blinks = Starting test
 *   Fast blink during test = Motors running
 *   LED off = Test complete (motors stopped)
 */

#include "stm32f4xx_hal.h"
#include "steval_fcu001_v1.h"
#include "usart1.h"

#include <stdbool.h>

// ============================================================================
// CALIBRATION VALUE - EDIT THIS AND REFLASH
// ============================================================================
// Start at 0.20 (20%), increase in steps of 0.05 until drone lifts.
// Values: 0.0 = off, 0.5 = 50%, 1.0 = full power
// DANGER: High values will cause drone to take off!
#define TEST_THRUST  0.30f

// Test duration in seconds
#define TEST_DURATION_SEC  5

// ============================================================================
// Motor PWM (TIM4 CH1-4 on PB6, PB7, PB8, PB9)
// ============================================================================

// TIM4 registers
#define TIM4_BASE           0x40000800UL
#define TIM4_CR1            (*(volatile uint32_t *)(TIM4_BASE + 0x00))
#define TIM4_CCMR1          (*(volatile uint32_t *)(TIM4_BASE + 0x18))
#define TIM4_CCMR2          (*(volatile uint32_t *)(TIM4_BASE + 0x1C))
#define TIM4_CCER           (*(volatile uint32_t *)(TIM4_BASE + 0x20))
#define TIM4_PSC            (*(volatile uint32_t *)(TIM4_BASE + 0x28))
#define TIM4_ARR            (*(volatile uint32_t *)(TIM4_BASE + 0x2C))
#define TIM4_CCR1           (*(volatile uint32_t *)(TIM4_BASE + 0x34))
#define TIM4_CCR2           (*(volatile uint32_t *)(TIM4_BASE + 0x38))
#define TIM4_CCR3           (*(volatile uint32_t *)(TIM4_BASE + 0x3C))
#define TIM4_CCR4           (*(volatile uint32_t *)(TIM4_BASE + 0x40))

// RCC registers
#define RCC_BASE            0x40023800UL
#define RCC_AHB1ENR         (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR         (*(volatile uint32_t *)(RCC_BASE + 0x40))

// GPIOB registers
#define GPIOB_BASE          0x40020400UL
#define GPIOB_MODER         (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OSPEEDR       (*(volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOB_AFRL          (*(volatile uint32_t *)(GPIOB_BASE + 0x20))
#define GPIOB_AFRH          (*(volatile uint32_t *)(GPIOB_BASE + 0x24))

// PWM: 16MHz / 16 / 1000 = 1kHz
#define PWM_PRESCALER       15
#define PWM_PERIOD          999

/* Blink LED n times */
static void blink_n(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        BSP_LED_On(LED1);
        HAL_Delay(on_ms);
        BSP_LED_Off(LED1);
        HAL_Delay(off_ms);
    }
    HAL_Delay(500);
}

/* Initialize motor PWM using direct register access */
static bool motors_init(void) {
    // Enable GPIOB clock
    RCC_AHB1ENR |= (1 << 1);  // GPIOBEN

    // Enable TIM4 clock
    RCC_APB1ENR |= (1 << 2);  // TIM4EN

    // Small delay for clock to stabilize
    for (volatile int i = 0; i < 100; i++);

    // Configure PB6, PB7, PB8, PB9 as alternate function (AF2 = TIM4)
    // MODER: 10 = alternate function
    GPIOB_MODER &= ~((3 << 12) | (3 << 14) | (3 << 16) | (3 << 18));  // Clear
    GPIOB_MODER |=  ((2 << 12) | (2 << 14) | (2 << 16) | (2 << 18));  // Set AF

    // High speed
    GPIOB_OSPEEDR |= ((3 << 12) | (3 << 14) | (3 << 16) | (3 << 18));

    // AF2 for TIM4 on PB6, PB7 (AFRL bits 24-31)
    GPIOB_AFRL &= ~((0xF << 24) | (0xF << 28));
    GPIOB_AFRL |=  ((2 << 24) | (2 << 28));  // AF2

    // AF2 for TIM4 on PB8, PB9 (AFRH bits 0-7)
    GPIOB_AFRH &= ~((0xF << 0) | (0xF << 4));
    GPIOB_AFRH |=  ((2 << 0) | (2 << 4));  // AF2

    // Configure TIM4
    TIM4_PSC = PWM_PRESCALER;
    TIM4_ARR = PWM_PERIOD;

    // PWM mode 1 on all channels (OC1M = 110, OC2M = 110, etc.)
    // Also enable preload (OC1PE, OC2PE)
    TIM4_CCMR1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);  // CH1, CH2
    TIM4_CCMR2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);  // CH3, CH4

    // Enable outputs (CC1E, CC2E, CC3E, CC4E)
    TIM4_CCER = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);

    // Start with 0% duty cycle
    TIM4_CCR1 = 0;
    TIM4_CCR2 = 0;
    TIM4_CCR3 = 0;
    TIM4_CCR4 = 0;

    // Enable counter
    TIM4_CR1 = 1;

    return true;
}

/* Set all motors to same PWM value (0-1000) */
static void motors_set_all(uint16_t speed) {
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    TIM4_CCR1 = speed;
    TIM4_CCR2 = speed;
    TIM4_CCR3 = speed;
    TIM4_CCR4 = speed;
}

/* Convert thrust (0.0-1.0) to PWM value (0-1000) */
static uint16_t thrust_to_pwm(float thrust) {
    if (thrust < 0.0f) thrust = 0.0f;
    if (thrust > 1.0f) thrust = 1.0f;
    return (uint16_t)(thrust * PWM_PERIOD);
}

int main(void) {
    // Set clock before HAL_Init
    SystemCoreClock = 16000000;

    // Initialize HAL
    HAL_Init();

    // Initialize LED
    BSP_LED_Init(LED1);
    BSP_LED_Off(LED1);

    // Initialize USART1 for debug output (115200 baud)
    usart1_init(NULL);

    usart1_puts("\r\n");
    usart1_puts("========================================\r\n");
    usart1_puts("STEVAL-FCU001V1 Thrust Calibration Test\r\n");
    usart1_puts("========================================\r\n\r\n");

    // Print configuration
    int thrust_pct = (int)(TEST_THRUST * 100);
    usart1_printf("Thrust:   %d.%02d (%d%%)\r\n",
                  (int)TEST_THRUST, (int)((TEST_THRUST - (int)TEST_THRUST) * 100),
                  thrust_pct);
    usart1_printf("Duration: %d seconds\r\n", TEST_DURATION_SEC);
    usart1_printf("PWM:      %d / %d\r\n\r\n", thrust_to_pwm(TEST_THRUST), PWM_PERIOD);

    usart1_puts("*** WARNING: SECURE DRONE OR REMOVE PROPELLERS! ***\r\n\r\n");

    // 2 blinks = Starting
    usart1_puts("Starting in 2 seconds... (2 blinks)\r\n");
    blink_n(2, 200, 200);

    // Initialize motors
    usart1_puts("Initializing motor PWM...\r\n");
    if (!motors_init()) {
        usart1_puts("FATAL: Motor init failed!\r\n");
        while (1) {
            BSP_LED_Toggle(LED1);
            HAL_Delay(1000);
        }
    }
    usart1_puts("Motors OK\r\n\r\n");

    // Start motors
    uint16_t pwm = thrust_to_pwm(TEST_THRUST);
    usart1_printf("*** MOTORS ON at %d%% ***\r\n", thrust_pct);
    motors_set_all(pwm);

    // Run for TEST_DURATION_SEC seconds with countdown
    for (int sec = TEST_DURATION_SEC; sec > 0; sec--) {
        usart1_printf("  %d...\r\n", sec);
        // Blink LED while running (toggle every 100ms = 5 Hz)
        for (int i = 0; i < 10; i++) {
            BSP_LED_Toggle(LED1);
            HAL_Delay(100);
        }
    }

    // Stop motors
    motors_set_all(0);
    BSP_LED_Off(LED1);

    usart1_puts("\r\n*** MOTORS OFF ***\r\n\r\n");
    usart1_puts("========================================\r\n");
    usart1_puts("Test complete. Press RESET to run again.\r\n");
    usart1_puts("========================================\r\n\r\n");

    usart1_puts("To increase thrust:\r\n");
    usart1_puts("  1. Edit TEST_THRUST in thrust_test.c\r\n");
    usart1_puts("  2. make TEST=thrust_test\r\n");
    usart1_puts("  3. make TEST=thrust_test flash\r\n");
    usart1_puts("  4. Press RESET\r\n\r\n");

    usart1_puts("When drone lifts, note the thrust value.\r\n");
    usart1_puts("Set HAL_BASE_THRUST to ~90%% of that value.\r\n");

    // Stay stopped forever (user must reset to run again)
    while (1) {
        HAL_Delay(1000);
    }
}

/* Called by HAL_Init */
void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

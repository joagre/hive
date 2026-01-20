/**
 * Thrust calibration test for Crazyflie 2.1+
 *
 * Runs all 4 motors at equal thrust for 5 seconds to calibrate hover thrust.
 *
 * Usage:
 *   1. REMOVE PROPELLERS! (or secure drone in test rig)
 *   2. Build: make -C tests thrust_test
 *   3. Flash: make -C tests flash-thrust
 *   4. Watch LED feedback
 *   5. Press reset to run test again
 *   6. Increase TEST_THRUST, reflash, repeat until drone lifts
 *   7. Set HAL_BASE_THRUST in hal_config.h to ~90% of liftoff thrust
 *
 * LED feedback (blue LED on PC4):
 *   2 slow blinks = Starting test (get ready!)
 *   Fast blink during test = Motors running
 *   LED off = Test complete (motors stopped)
 *   Continuous fast blink = Error
 *
 * Motor layout (X-configuration):
 *          Front
 *      M1(CCW)  M2(CW)
 *          +--+
 *          |  |
 *          +--+
 *      M4(CW)  M3(CCW)
 *          Rear
 *
 * TIM2 PWM: PA0=M1, PA1=M2, PA2=M3, PA3=M4
 */

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// CALIBRATION VALUE - EDIT THIS AND REFLASH
// ============================================================================
// Start at 0.15 (15%), increase in steps of 0.05 until drone lifts.
// Values: 0.0 = off, 0.5 = 50%, 1.0 = full power
// DANGER: High values will cause drone to take off!
#define TEST_THRUST 0.20f

// Test duration in seconds
#define TEST_DURATION_SEC 5

// ============================================================================
// Hardware Addresses
// ============================================================================

// Peripheral bases
#define PERIPH_BASE 0x40000000UL
#define APB1PERIPH_BASE PERIPH_BASE
#define AHB1PERIPH_BASE (PERIPH_BASE + 0x00020000UL)

// GPIO
#define GPIOA_BASE (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOC_BASE (AHB1PERIPH_BASE + 0x0800UL)

#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OSPEEDR (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_PUPDR (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))
#define GPIOA_AFR0 (*(volatile uint32_t *)(GPIOA_BASE + 0x20))

#define GPIOC_MODER (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_OSPEEDR (*(volatile uint32_t *)(GPIOC_BASE + 0x08))
#define GPIOC_ODR (*(volatile uint32_t *)(GPIOC_BASE + 0x14))

// RCC
#define RCC_BASE (AHB1PERIPH_BASE + 0x3800UL)
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40))

// TIM2
#define TIM2_BASE (APB1PERIPH_BASE + 0x0000UL)
#define TIM2_CR1 (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_CCMR1 (*(volatile uint32_t *)(TIM2_BASE + 0x18))
#define TIM2_CCMR2 (*(volatile uint32_t *)(TIM2_BASE + 0x1C))
#define TIM2_CCER (*(volatile uint32_t *)(TIM2_BASE + 0x20))
#define TIM2_PSC (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR (*(volatile uint32_t *)(TIM2_BASE + 0x2C))
#define TIM2_CCR1 (*(volatile uint32_t *)(TIM2_BASE + 0x34))
#define TIM2_CCR2 (*(volatile uint32_t *)(TIM2_BASE + 0x38))
#define TIM2_CCR3 (*(volatile uint32_t *)(TIM2_BASE + 0x3C))
#define TIM2_CCR4 (*(volatile uint32_t *)(TIM2_BASE + 0x40))
#define TIM2_EGR (*(volatile uint32_t *)(TIM2_BASE + 0x14))

// SysTick (for delays)
#define SYSTICK_BASE 0xE000E010UL
#define SYSTICK_CTRL (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))

// Flash
#define FLASH_BASE (AHB1PERIPH_BASE + 0x3C00UL)
#define FLASH_ACR (*(volatile uint32_t *)(FLASH_BASE + 0x00))

// PWM configuration
// TIM2 runs at APB1*2 = 84 MHz (168 MHz system / 2)
// For ~328 kHz PWM: 84 MHz / 1 / 256 = 328 kHz
#define PWM_PRESCALER 0
#define PWM_PERIOD 255

// LED (PC4)
#define LED_PIN (1 << 4)

// ============================================================================
// Global State
// ============================================================================

static volatile uint32_t g_ticks = 0;

// ============================================================================
// SysTick Handler
// ============================================================================

void SysTick_Handler(void) {
    g_ticks++;
}

// ============================================================================
// Helper Functions
// ============================================================================

static void delay_ms(uint32_t ms) {
    uint32_t start = g_ticks;
    while ((g_ticks - start) < ms)
        ;
}

static void led_on(void) {
    GPIOC_ODR |= LED_PIN;
}
static void led_off(void) {
    GPIOC_ODR &= ~LED_PIN;
}
static void led_toggle(void) {
    GPIOC_ODR ^= LED_PIN;
}

static void blink_n(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        led_on();
        delay_ms(on_ms);
        led_off();
        delay_ms(off_ms);
    }
    delay_ms(300);
}

// ============================================================================
// System Initialization
// ============================================================================

static void clock_init(void) {
    // Configure flash latency for 168 MHz (5 wait states)
    FLASH_ACR = (5 << 0) | (1 << 8) | (1 << 9) | (1 << 10);

    // Enable HSE
    volatile uint32_t *RCC_CR = (volatile uint32_t *)(RCC_BASE + 0x00);
    volatile uint32_t *RCC_PLLCFGR = (volatile uint32_t *)(RCC_BASE + 0x04);
    volatile uint32_t *RCC_CFGR = (volatile uint32_t *)(RCC_BASE + 0x08);

    *RCC_CR |= (1 << 16); // HSEON
    while (!(*RCC_CR & (1 << 17)))
        ; // Wait for HSERDY

    // Configure PLL: HSE=8MHz, PLLM=4, PLLN=168, PLLP=2, PLLQ=7
    // VCO = 8/4 * 168 = 336 MHz, SYSCLK = 336/2 = 168 MHz
    *RCC_PLLCFGR = (4 << 0) | (168 << 6) | (0 << 16) | (1 << 22) | (7 << 24);

    // Enable PLL
    *RCC_CR |= (1 << 24); // PLLON
    while (!(*RCC_CR & (1 << 25)))
        ; // Wait for PLLRDY

    // Configure prescalers: AHB=1, APB1=4, APB2=2
    *RCC_CFGR = (0 << 4) | (5 << 10) | (4 << 13);

    // Switch to PLL
    *RCC_CFGR |= (2 << 0); // SW = PLL
    while (((*RCC_CFGR >> 2) & 0x3) != 2)
        ; // Wait for SWS = PLL
}

static void systick_init(void) {
    // Configure SysTick for 1ms ticks (168 MHz / 168000 = 1 kHz)
    SYSTICK_LOAD = 168000 - 1;
    SYSTICK_VAL = 0;
    SYSTICK_CTRL =
        (1 << 2) | (1 << 1) | (1 << 0); // CLKSOURCE | TICKINT | ENABLE
}

static void gpio_init(void) {
    // Enable GPIOA and GPIOC clocks
    RCC_AHB1ENR |= (1 << 0) | (1 << 2); // GPIOAEN | GPIOCEN

    // Small delay for clock to stabilize
    for (volatile int i = 0; i < 100; i++)
        ;

    // Configure PC4 as output (LED)
    GPIOC_MODER &= ~(3 << 8);  // Clear
    GPIOC_MODER |= (1 << 8);   // Output mode
    GPIOC_OSPEEDR |= (3 << 8); // High speed
    GPIOC_ODR &= ~LED_PIN;     // LED off
}

static bool motors_init(void) {
    // Enable TIM2 clock
    RCC_APB1ENR |= (1 << 0); // TIM2EN

    // Small delay for clock to stabilize
    for (volatile int i = 0; i < 100; i++)
        ;

    // Configure PA0-PA3 as alternate function (AF1 = TIM2)
    // MODER: 10 = alternate function
    GPIOA_MODER &= ~((3 << 0) | (3 << 2) | (3 << 4) | (3 << 6));
    GPIOA_MODER |= ((2 << 0) | (2 << 2) | (2 << 4) | (2 << 6));

    // High speed
    GPIOA_OSPEEDR |= ((3 << 0) | (3 << 2) | (3 << 4) | (3 << 6));

    // No pull-up/pull-down
    GPIOA_PUPDR &= ~((3 << 0) | (3 << 2) | (3 << 4) | (3 << 6));

    // AF1 for TIM2 on PA0-PA3
    GPIOA_AFR0 &= ~0xFFFF;
    GPIOA_AFR0 |= (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);

    // Configure TIM2
    TIM2_CR1 = 0; // Stop timer
    TIM2_PSC = PWM_PRESCALER;
    TIM2_ARR = PWM_PERIOD;

    // PWM mode 1 on all channels (OCxM = 110), preload enable
    TIM2_CCMR1 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);
    TIM2_CCMR2 = (6 << 4) | (1 << 3) | (6 << 12) | (1 << 11);

    // Enable outputs
    TIM2_CCER = (1 << 0) | (1 << 4) | (1 << 8) | (1 << 12);

    // Start with 0% duty cycle
    TIM2_CCR1 = 0;
    TIM2_CCR2 = 0;
    TIM2_CCR3 = 0;
    TIM2_CCR4 = 0;

    // Generate update event to load registers
    TIM2_EGR = 1;

    // Enable counter with auto-reload preload
    TIM2_CR1 = (1 << 7) | (1 << 0); // ARPE | CEN

    return true;
}

static void motors_set_all(uint16_t speed) {
    if (speed > PWM_PERIOD)
        speed = PWM_PERIOD;
    TIM2_CCR1 = speed;
    TIM2_CCR2 = speed;
    TIM2_CCR3 = speed;
    TIM2_CCR4 = speed;
}

static uint16_t thrust_to_pwm(float thrust) {
    if (thrust < 0.0f)
        thrust = 0.0f;
    if (thrust > 1.0f)
        thrust = 1.0f;
    return (uint16_t)(thrust * PWM_PERIOD);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    // Initialize system
    clock_init();
    systick_init();
    gpio_init();

    // 2 slow blinks = Starting
    blink_n(2, 300, 300);
    delay_ms(1000);

    // Initialize motors
    if (!motors_init()) {
        // Error: continuous fast blink
        while (1) {
            led_toggle();
            delay_ms(100);
        }
    }

    // 3 quick blinks = Motors initialized, starting test
    blink_n(3, 100, 100);
    delay_ms(500);

    // Start motors at test thrust
    uint16_t pwm = thrust_to_pwm(TEST_THRUST);
    motors_set_all(pwm);

    // Run for TEST_DURATION_SEC seconds with fast LED blink
    for (int sec = 0; sec < TEST_DURATION_SEC; sec++) {
        // Blink LED while running (toggle every 100ms = 5 Hz)
        for (int i = 0; i < 10; i++) {
            led_toggle();
            delay_ms(100);
        }
    }

    // Stop motors
    motors_set_all(0);
    led_off();

    // 5 slow blinks = Test complete
    delay_ms(500);
    blink_n(5, 200, 200);

    // Stay stopped forever (user must reset to run again)
    while (1) {
        delay_ms(1000);
    }
}

// ============================================================================
// Startup and Vector Table
// ============================================================================

extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata;
extern uint32_t _sbss, _ebss;

void Reset_Handler(void) {
    // Copy .data from flash to RAM
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata)
        *dst++ = *src++;

    // Zero .bss
    dst = &_sbss;
    while (dst < &_ebss)
        *dst++ = 0;

    // Call main
    main();
    while (1)
        ;
}

void Default_Handler(void) {
    while (1)
        ;
}

// Weak aliases for handlers
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));

// Vector table
__attribute__((section(".isr_vector"))) const uint32_t g_vectors[] = {
    (uint32_t)&_estack,
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    (uint32_t)MemManage_Handler,
    (uint32_t)BusFault_Handler,
    (uint32_t)UsageFault_Handler,
    0,
    0,
    0,
    0,
    (uint32_t)SVC_Handler,
    (uint32_t)DebugMon_Handler,
    0,
    (uint32_t)PendSV_Handler,
    (uint32_t)SysTick_Handler,
};

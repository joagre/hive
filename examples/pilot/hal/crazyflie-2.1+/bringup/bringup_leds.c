// Crazyflie 2.1+ Bring-Up - LED Test
//
// Tests all controllable LEDs:
// - Blue LED (PD2): Status/activity indicator
// - Red LEDs (PC0-PC3): Motor position indicators (M1-M4)
//
// Test sequence:
// 1. Initialize all LED GPIOs
// 2. Blink blue LED
// 3. Sequence through red LEDs (M1 -> M2 -> M3 -> M4)
// 4. All on, all off pattern

#include "bringup_leds.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"

// LED pin definitions
// Blue LED: PD2
#define LED_BLUE_PORT GPIOD
#define LED_BLUE_PIN 2

// Red LEDs: PC0-PC3 (M1-M4)
#define LED_RED_PORT GPIOC
#define LED_M1_PIN 0
#define LED_M2_PIN 1
#define LED_M3_PIN 2
#define LED_M4_PIN 3

// LED timing
#define BLINK_ON_MS 150
#define BLINK_OFF_MS 150
#define SEQUENCE_DELAY_MS 100

// Timing
static uint32_t get_ticks(void) {
    return swo_get_ticks();
}

static void delay_ms(uint32_t ms) {
    uint32_t start = get_ticks();
    while ((get_ticks() - start) < ms)
        ;
}

// ----------------------------------------------------------------------------
// LED Control
// ----------------------------------------------------------------------------

void led_test_init(void) {
    // Enable GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

    // Configure PD2 (Blue LED) as output
    LED_BLUE_PORT->MODER &= ~(3UL << (LED_BLUE_PIN * 2));
    LED_BLUE_PORT->MODER |= (1UL << (LED_BLUE_PIN * 2));
    LED_BLUE_PORT->OSPEEDR |= (3UL << (LED_BLUE_PIN * 2));
    LED_BLUE_PORT->ODR &= ~(1UL << LED_BLUE_PIN); // Off

    // Configure PC0-PC3 (Red LEDs) as outputs
    for (int pin = LED_M1_PIN; pin <= LED_M4_PIN; pin++) {
        LED_RED_PORT->MODER &= ~(3UL << (pin * 2));
        LED_RED_PORT->MODER |= (1UL << (pin * 2));
        LED_RED_PORT->OSPEEDR |= (3UL << (pin * 2));
        LED_RED_PORT->ODR &= ~(1UL << pin); // Off
    }
}

void led_set(led_id_t led, bool on) {
    switch (led) {
    case LED_BLUE:
        if (on) {
            LED_BLUE_PORT->ODR |= (1UL << LED_BLUE_PIN);
        } else {
            LED_BLUE_PORT->ODR &= ~(1UL << LED_BLUE_PIN);
        }
        break;

    case LED_M1_RED:
        if (on) {
            LED_RED_PORT->ODR |= (1UL << LED_M1_PIN);
        } else {
            LED_RED_PORT->ODR &= ~(1UL << LED_M1_PIN);
        }
        break;

    case LED_M2_RED:
        if (on) {
            LED_RED_PORT->ODR |= (1UL << LED_M2_PIN);
        } else {
            LED_RED_PORT->ODR &= ~(1UL << LED_M2_PIN);
        }
        break;

    case LED_M3_RED:
        if (on) {
            LED_RED_PORT->ODR |= (1UL << LED_M3_PIN);
        } else {
            LED_RED_PORT->ODR &= ~(1UL << LED_M3_PIN);
        }
        break;

    case LED_M4_RED:
        if (on) {
            LED_RED_PORT->ODR |= (1UL << LED_M4_PIN);
        } else {
            LED_RED_PORT->ODR &= ~(1UL << LED_M4_PIN);
        }
        break;

    default:
        break;
    }
}

void led_toggle(led_id_t led) {
    switch (led) {
    case LED_BLUE:
        LED_BLUE_PORT->ODR ^= (1UL << LED_BLUE_PIN);
        break;
    case LED_M1_RED:
        LED_RED_PORT->ODR ^= (1UL << LED_M1_PIN);
        break;
    case LED_M2_RED:
        LED_RED_PORT->ODR ^= (1UL << LED_M2_PIN);
        break;
    case LED_M3_RED:
        LED_RED_PORT->ODR ^= (1UL << LED_M3_PIN);
        break;
    case LED_M4_RED:
        LED_RED_PORT->ODR ^= (1UL << LED_M4_PIN);
        break;
    default:
        break;
    }
}

static void led_all_off(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        led_set((led_id_t)i, false);
    }
}

static void led_all_on(void) {
    for (int i = 0; i < LED_COUNT; i++) {
        led_set((led_id_t)i, true);
    }
}

static void led_blink(led_id_t led, int count) {
    for (int i = 0; i < count; i++) {
        led_set(led, true);
        delay_ms(BLINK_ON_MS);
        led_set(led, false);
        delay_ms(BLINK_OFF_MS);
    }
}

// ----------------------------------------------------------------------------
// LED Test Implementation
// ----------------------------------------------------------------------------

bool led_run_test(led_test_results_t *results) {
    results->gpio_init_ok = false;
    results->blue_tested = false;
    results->red_m1_tested = false;
    results->red_m2_tested = false;
    results->red_m3_tested = false;
    results->red_m4_tested = false;

    swo_puts("[LED] Testing all controllable LEDs\n");
    swo_puts("[LED] Watch the Crazyflie for LED activity\n");

    // Initialize
    swo_puts("[LED] Initializing GPIO... ");
    led_test_init();
    results->gpio_init_ok = true;
    swo_puts("OK\n");

    // Ensure all off
    led_all_off();
    delay_ms(200);

    // Blue LED test
    swo_puts("[LED] Blue LED (PD2) - 3 blinks... ");
    led_blink(LED_BLUE, 3);
    results->blue_tested = true;
    swo_puts("OK\n");

    delay_ms(SEQUENCE_DELAY_MS);

    // Red LED sequence (motor positions)
    swo_puts("[LED] Red LED M1 (PC0, front-left) - 2 blinks... ");
    led_blink(LED_M1_RED, 2);
    results->red_m1_tested = true;
    swo_puts("OK\n");

    delay_ms(SEQUENCE_DELAY_MS);

    swo_puts("[LED] Red LED M2 (PC1, front-right) - 2 blinks... ");
    led_blink(LED_M2_RED, 2);
    results->red_m2_tested = true;
    swo_puts("OK\n");

    delay_ms(SEQUENCE_DELAY_MS);

    swo_puts("[LED] Red LED M3 (PC2, rear-right) - 2 blinks... ");
    led_blink(LED_M3_RED, 2);
    results->red_m3_tested = true;
    swo_puts("OK\n");

    delay_ms(SEQUENCE_DELAY_MS);

    swo_puts("[LED] Red LED M4 (PC3, rear-left) - 2 blinks... ");
    led_blink(LED_M4_RED, 2);
    results->red_m4_tested = true;
    swo_puts("OK\n");

    delay_ms(SEQUENCE_DELAY_MS);

    // All on/off pattern
    swo_puts("[LED] All LEDs - on/off pattern... ");
    for (int i = 0; i < 2; i++) {
        led_all_on();
        delay_ms(200);
        led_all_off();
        delay_ms(200);
    }
    swo_puts("OK\n");

    // Rotating pattern
    swo_puts("[LED] Rotating pattern (M1->M2->M3->M4)... ");
    for (int cycle = 0; cycle < 2; cycle++) {
        for (int i = LED_M1_RED; i <= LED_M4_RED; i++) {
            led_set((led_id_t)i, true);
            delay_ms(100);
            led_set((led_id_t)i, false);
        }
    }
    swo_puts("OK\n");

    // Leave blue LED on to indicate test complete
    led_set(LED_BLUE, true);

    swo_puts("[LED] All LED tests complete!\n");
    swo_puts("[LED] Note: Red/green charge LEDs are controlled by nRF51\n");

    return true;
}

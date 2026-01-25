// Crazyflie 2.1+ Bring-Up - LED Test
//
// Tests all controllable LEDs on the Crazyflie 2.1+:
// - Blue LED (PD2) - Status LED
// - Red LEDs M1-M4 (PC0-PC3) - Motor position indicators
//
// Note: The red/green charge LEDs are controlled by the nRF51
// and cannot be tested directly from the STM32.

#ifndef BRINGUP_LEDS_H
#define BRINGUP_LEDS_H

#include <stdbool.h>
#include <stdint.h>

// LED identifiers
typedef enum {
    LED_BLUE = 0, // PD2 - Status LED
    LED_M1_RED,   // PC0 - Front-left motor
    LED_M2_RED,   // PC1 - Front-right motor
    LED_M3_RED,   // PC2 - Rear-right motor
    LED_M4_RED,   // PC3 - Rear-left motor
    LED_COUNT
} led_id_t;

// LED test results
typedef struct {
    bool gpio_init_ok;  // GPIO configured
    bool blue_tested;   // Blue LED blinked
    bool red_m1_tested; // M1 red LED blinked
    bool red_m2_tested; // M2 red LED blinked
    bool red_m3_tested; // M3 red LED blinked
    bool red_m4_tested; // M4 red LED blinked
} led_test_results_t;

// Initialize LED test (configures GPIO)
void led_test_init(void);

// Individual LED control
void led_set(led_id_t led, bool on);
void led_toggle(led_id_t led);

// Run LED test sequence
// Blinks each LED in sequence for visual verification
bool led_run_test(led_test_results_t *results);

#endif // BRINGUP_LEDS_H

// Crazyflie 2.1+ Bring-Up Test
//
// Systematically tests each hardware component and reports via SWO.
// Connect ST-Link V3 to debug adapter (20-pin). SWO on pin 13.
//
// View output: st-trace -c 168

#include "bringup_deck.h"
#include "bringup_eeprom.h"
#include "bringup_flash.h"
#include "bringup_i2c3.h"
#include "bringup_leds.h"
#include "bringup_motors.h"
#include "bringup_radio.h"
#include "bringup_sd.h"
#include "bringup_sensors.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"

// CMSIS required global - actual value set by system_clock_init()
uint32_t SystemCoreClock = 16000000; // Default to HSI

// Required by startup code - do nothing, we init clocks in main()
void SystemInit(void) {
}

// Newlib stubs
void _init(void) {
}
void _fini(void) {
}

// LED on PD2
#define LED_PIN 2

// Test results
static bool s_boot_ok = false;
static bool s_i2c_ok = false;
static bool s_sensors_ok = false;
static bool s_motors_ok = false;
static bool s_radio_ok = false;
static bool s_flash_ok = false;
static bool s_sd_ok = false;
static bool s_eeprom_ok = false;
static bool s_deck_ok = false;
static bool s_leds_ok = false;
static bool s_flow_deck = false;
static bool s_sd_deck = false;
static int s_deck_count = 0;

// System clock configuration (168 MHz from 8 MHz HSE)
static void system_clock_init(void) {
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    // Configure flash latency (5 wait states for 168 MHz)
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |
                 FLASH_ACR_DCEN;

    // Configure PLL: HSE/8 * 336 / 2 = 168 MHz
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (8 << RCC_PLLCFGR_PLLM_Pos) |
                   (336 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) | // PLLP = 2
                   (7 << RCC_PLLCFGR_PLLQ_Pos);

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Configure bus clocks: AHB = 168 MHz, APB1 = 42 MHz, APB2 = 84 MHz
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    // Switch to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    // Update SystemCoreClock variable
    SystemCoreClock = 168000000;
}

static void led_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= GPIO_MODER_MODER2_0; // Output
}

static void led_on(void) {
    GPIOD->BSRR = GPIO_BSRR_BS_2;
}

static void led_off(void) {
    GPIOD->BSRR = GPIO_BSRR_BR_2;
}

static void blue_led_toggle(void) {
    GPIOD->ODR ^= GPIO_ODR_OD2;
}

static void delay_ms(uint32_t ms) {
    volatile uint32_t count = ms * 42000;
    while (count--)
        ;
}

static void led_blink(int count, int period_ms) {
    for (int i = 0; i < count; i++) {
        led_on();
        delay_ms(period_ms / 2);
        led_off();
        delay_ms(period_ms / 2);
    }
}

// ----------------------------------------------------------------------------
// Test Phases (ordered from primitive to complex)
// ----------------------------------------------------------------------------

// Phase 1: Boot & Clock - Most fundamental system initialization
static bool test_boot(void) {
    swo_puts("\n=== Phase 1: Boot & Clock ===\n");

    // LED blink test
    swo_puts("[BOOT] LED blink test... ");
    led_blink(3, 200);
    swo_puts("OK\n");

    // Clock test
    swo_printf("[BOOT] SystemCoreClock = %u Hz... ", SystemCoreClock);
    if (SystemCoreClock == 168000000) {
        swo_puts("OK\n");
        return true;
    } else {
        swo_puts("FAIL\n");
        return false;
    }
}

// Phase 2: LEDs - Simple GPIO output test
static bool test_leds(void) {
    swo_puts("\n=== Phase 2: LED Test ===\n");

    led_test_results_t results;
    return led_run_test(&results);
}

// Phase 3: I2C Bus Scan - Basic I2C protocol (on-board sensors only)
static bool test_i2c_scan(void) {
    swo_puts("\n=== Phase 3: I2C Bus Scan ===\n");

    swo_puts("[I2C] Initializing I2C3 (on-board sensors)...\n");
    i2c3_init();

    swo_puts("[I2C] Scanning I2C3 bus...\n");

    uint8_t found[16];
    int count = i2c3_scan(found, 16);

    bool accel_found = false;
    bool gyro_found = false;
    bool baro_found = false;

    for (int i = 0; i < count; i++) {
        const char *name = i2c3_device_name(found[i]);
        swo_printf("[I2C] Found device at 0x%02X (%s)\n", found[i], name);

        if (found[i] == I2C3_ADDR_BMI088_ACCEL)
            accel_found = true;
        if (found[i] == I2C3_ADDR_BMI088_GYRO)
            gyro_found = true;
        if (found[i] == I2C3_ADDR_BMP388)
            baro_found = true;
    }

    swo_printf("[I2C] Scan complete: %d devices found\n", count);

    // Check required devices (on-board only - VL53L1x is on I2C1)
    bool ok = accel_found && gyro_found && baro_found;
    swo_printf("[I2C] Required sensors (IMU, Baro): %s\n", ok ? "OK" : "FAIL");

    return ok;
}

// Phase 4: Sensors - I2C/SPI device communication
static bool test_sensors(void) {
    swo_puts("\n=== Phase 4: Sensors ===\n");

    spi_init(); // For PMW3901
    sensors_init();

    sensor_test_results_t results;
    sensor_run_all_tests(&results);

    s_flow_deck = results.flow_deck_present;

    // Required: accel, gyro, baro
    bool ok = results.accel_id_ok && results.gyro_id_ok && results.baro_id_ok &&
              results.accel_data_ok && results.gyro_data_ok &&
              results.baro_data_ok;

    return ok;
}

// Phase 5: EEPROM - I2C read/write on separate bus
static bool test_eeprom(void) {
    swo_puts("\n=== Phase 5: EEPROM Test ===\n");

    eeprom_test_results_t results;
    return eeprom_run_test(&results);
}

// Phase 6: Deck Detection - 1-Wire bit-banged protocol
static bool test_deck(void) {
    swo_puts("\n=== Phase 6: Deck Detection ===\n");

    deck_test_results_t results;
    bool ok = deck_run_test(&results);

    s_deck_count = results.deck_count;
    return ok;
}

// Phase 7: Motors - Timer-based PWM output
static bool test_motors(void) {
    swo_puts("\n=== Phase 7: Motor Test ===\n");

    motors_init();
    return motors_run_test();
}

// Phase 8: Radio - USART serial protocol (syslink)
static bool test_radio(void) {
    swo_puts("\n=== Phase 8: Radio Test ===\n");

    return radio_run_test(5000); // 5 second timeout
}

// Phase 9: Flash Storage - Internal flash programming (complex, runs from RAM)
static bool test_flash(void) {
    swo_puts("\n=== Phase 9: Flash Storage Test ===\n");

    flash_test_results_t results;
    return flash_run_test(&results);
}

// Phase 10: SD Card - SPI + SD protocol (most complex, optional)
static bool test_sd(void) {
    swo_puts("\n=== Phase 10: SD Card Test ===\n");

    sd_test_results_t results;
    bool ok = sd_run_test(&results);

    s_sd_deck = results.card_present;
    return ok;
}

// Print summary (in order of test execution: primitive to complex)
static void print_summary(void) {
    swo_puts("\n");
    swo_puts("========================================\n");
    swo_puts("  Bring-Up Test Summary\n");
    swo_puts("========================================\n");
    swo_printf("1. Boot & Clock:    %s\n", s_boot_ok ? "OK" : "FAIL");
    swo_printf("2. LEDs:            %s\n", s_leds_ok ? "OK" : "FAIL");
    swo_printf("3. I2C Bus Scan:    %s\n", s_i2c_ok ? "OK" : "FAIL");
    swo_printf("4. Sensors:         %s\n", s_sensors_ok ? "OK" : "FAIL");
    swo_printf("5. EEPROM:          %s\n", s_eeprom_ok ? "OK" : "TODO");
    swo_printf("6. Deck Detection:  %s\n", s_deck_ok ? "OK" : "TODO");
    swo_printf("7. Motors:          %s\n", s_motors_ok ? "OK" : "TODO");
    swo_printf("8. Radio:           %s\n", s_radio_ok ? "OK" : "TODO");
    swo_printf("9. Flash Storage:   %s\n", s_flash_ok ? "OK" : "TODO");
    swo_printf("10. SD Card:        %s\n",
               s_sd_deck ? (s_sd_ok ? "OK" : "FAIL") : "TODO");
    swo_puts("\n");
    swo_printf("Flow deck:          %s\n",
               s_flow_deck ? "DETECTED" : "NOT DETECTED");
    if (s_deck_count > 0) {
        swo_printf("Expansion decks:    %d detected\n", s_deck_count);
    }
    swo_puts("\n");

    // Check only the phases we've tested so far
    bool all_tested_ok = s_boot_ok && s_leds_ok && s_i2c_ok && s_sensors_ok;
    if (all_tested_ok) {
        swo_puts("All tested phases passed!\n");
        // Steady LED on
        led_on();
    } else {
        swo_puts("Some tests FAILED - check output above\n");
        // Fast blink to indicate failure
        while (1) {
            blue_led_toggle();
            delay_ms(100);
        }
    }
}

int main(void) {
    // Initialize system
    system_clock_init();
    led_init();
    swo_init();

    swo_puts("\n\n");
    swo_puts("========================================\n");
    swo_puts("  Crazyflie 2.1+ Hardware Bring-Up\n");
    swo_puts("========================================\n");

    // Run test phases in order (primitive to complex)
    // Phase 1: Boot & Clock
    s_boot_ok = test_boot();

    // Phase 2: LEDs
    s_leds_ok = test_leds();

    // Phase 3: I2C Bus Scan
    s_i2c_ok = test_i2c_scan();

    // Phase 4: Sensors (requires I2C)
    if (s_i2c_ok) {
        s_sensors_ok = test_sensors();
    } else {
        swo_puts("\n=== Phase 4: Sensors ===\n");
        swo_puts("[SENSOR] SKIPPED - I2C not working\n");
    }

    // Phase 5: EEPROM (separate I2C1 bus)
    s_eeprom_ok = test_eeprom();

    // Phase 6: Deck Detection (1-Wire on PC11)
    s_deck_ok = test_deck();

    // Phase 7: Motors
    s_motors_ok = test_motors();

    // Phase 8: Radio (Syslink to nRF51)
    s_radio_ok = test_radio();

    // Phase 9: Flash Storage (sector 8)
    s_flash_ok = test_flash();

    // Phase 10: SD Card (requires SD card deck)
    s_sd_ok = test_sd();

    // Print summary
    print_summary();

    // Busy-wait loop (don't use WFI - it breaks ST-Link debug connection)
    while (1) {
        __NOP();
    }

    return 0;
}

// Default interrupt handlers
void NMI_Handler(void) {
    while (1)
        ;
}
void HardFault_Handler(void) {
    while (1)
        ;
}
void MemManage_Handler(void) {
    while (1)
        ;
}
void BusFault_Handler(void) {
    while (1)
        ;
}
void UsageFault_Handler(void) {
    while (1)
        ;
}
void SVC_Handler(void) {
}
void DebugMon_Handler(void) {
}
void PendSV_Handler(void) {
}

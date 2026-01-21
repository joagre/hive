// Crazyflie 2.1+ Bring-Up Test
//
// Systematically tests each hardware component and reports via UART.
// Connect ST-Link V3 VCP to USART1 (PA9/PA10) via debug adapter.
//
// Run: screen /dev/ttyACM0 115200

#include "bringup_i2c.h"
#include "bringup_motors.h"
#include "bringup_radio.h"
#include "bringup_sensors.h"
#include "bringup_uart.h"
#include "stm32f4xx.h"

// CMSIS required global - actual value set by system_clock_init()
uint32_t SystemCoreClock = 16000000; // Default to HSI

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
static bool s_flow_deck = false;

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

static void led_toggle(void) {
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

// Phase 1: Boot & Clock
static bool test_boot(void) {
    uart_puts("\n=== Phase 1: Boot & Clock ===\n");

    // LED blink test
    uart_puts("[BOOT] LED blink test... ");
    led_blink(3, 200);
    uart_puts("OK\n");

    // Clock test
    uart_printf("[BOOT] SystemCoreClock = %u Hz... ", SystemCoreClock);
    if (SystemCoreClock == 168000000) {
        uart_puts("OK\n");
        return true;
    } else {
        uart_puts("FAIL\n");
        return false;
    }
}

// Phase 2: I2C Bus Scan
static bool test_i2c_scan(void) {
    uart_puts("\n=== Phase 2: I2C Bus Scan ===\n");

    uart_puts("[I2C] Initializing I2C3...\n");
    i2c_init();

    uart_puts("[I2C] Scanning I2C3 bus...\n");

    uint8_t found[16];
    int count = i2c_scan(found, 16);

    bool accel_found = false;
    bool gyro_found = false;
    bool baro_found = false;
    bool tof_found = false;

    for (int i = 0; i < count; i++) {
        const char *name = i2c_device_name(found[i]);
        uart_printf("[I2C] Found device at 0x%02X (%s)\n", found[i], name);

        if (found[i] == I2C_ADDR_BMI088_ACCEL)
            accel_found = true;
        if (found[i] == I2C_ADDR_BMI088_GYRO)
            gyro_found = true;
        if (found[i] == I2C_ADDR_BMP388)
            baro_found = true;
        if (found[i] == I2C_ADDR_VL53L1X)
            tof_found = true;
    }

    uart_printf("[I2C] Scan complete: %d devices found\n", count);

    // Check required devices
    bool ok = accel_found && gyro_found && baro_found;
    uart_printf("[I2C] Required sensors (IMU, Baro): %s\n", ok ? "OK" : "FAIL");

    if (tof_found) {
        uart_puts("[I2C] Flow deck detected\n");
        s_flow_deck = true;
    }

    return ok;
}

// Phase 3 & 4: Sensor Tests
static bool test_sensors(void) {
    uart_puts("\n=== Phase 3: Sensor Chip IDs ===\n");
    uart_puts("\n=== Phase 4: Sensor Data ===\n");

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

// Phase 5: Motor Test
static bool test_motors(void) {
    uart_puts("\n=== Phase 5: Motor Test ===\n");

    motors_init();
    return motors_run_test();
}

// Phase 6: Radio Test
static bool test_radio(void) {
    uart_puts("\n=== Phase 6: Radio Test ===\n");

    return radio_run_test(5000); // 5 second timeout
}

// Print summary
static void print_summary(void) {
    uart_puts("\n");
    uart_puts("=== Bring-Up Test Summary ===\n");
    uart_printf("Boot & Clock:     %s\n", s_boot_ok ? "OK" : "FAIL");
    uart_printf("I2C Bus Scan:     %s\n", s_i2c_ok ? "OK" : "FAIL");
    uart_printf("Sensors:          %s\n", s_sensors_ok ? "OK" : "FAIL");
    uart_printf("Motors:           %s\n", s_motors_ok ? "OK" : "SKIP/FAIL");
    uart_printf("Radio:            %s\n", s_radio_ok ? "OK" : "FAIL");
    uart_puts("\n");
    uart_printf("Flow deck:        %s\n",
                s_flow_deck ? "DETECTED" : "NOT DETECTED");
    uart_puts("\n");

    bool all_required = s_boot_ok && s_i2c_ok && s_sensors_ok;
    if (all_required) {
        uart_puts("All required tests passed!\n");
        // Steady LED on
        led_on();
    } else {
        uart_puts("Some tests FAILED - check output above\n");
        // Fast blink to indicate failure
        while (1) {
            led_toggle();
            delay_ms(100);
        }
    }
}

int main(void) {
    // Initialize system
    system_clock_init();
    led_init();
    uart_init();

    // Welcome message
    uart_puts("\n\n");
    uart_puts("========================================\n");
    uart_puts("  Crazyflie 2.1+ Bring-Up Test\n");
    uart_puts("========================================\n");
    uart_puts("\n");
    uart_puts("Press ENTER to start tests...\n");

    // Wait for keypress or timeout
    int c = uart_getc_timeout(5000);
    if (c < 0) {
        uart_puts("(timeout - starting automatically)\n");
    }

    // Run tests
    s_boot_ok = test_boot();
    if (!s_boot_ok) {
        uart_puts("\n!!! Boot test failed - cannot continue !!!\n");
        goto done;
    }

    s_i2c_ok = test_i2c_scan();
    if (!s_i2c_ok) {
        uart_puts("\n!!! I2C test failed - cannot test sensors !!!\n");
        // Continue anyway to test motors/radio
    } else {
        s_sensors_ok = test_sensors();
    }

    s_motors_ok = test_motors();
    s_radio_ok = test_radio();

done:
    print_summary();

    // Idle loop
    while (1) {
        __WFI();
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

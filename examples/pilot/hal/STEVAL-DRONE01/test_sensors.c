// Sensor test program for STEVAL-DRONE01
//
// Reads all sensors and prints values over USART1 (115200 baud).
// Connect ST-Link virtual COM port to see output.
//
// Build: make -f Makefile.test
// Flash: make -f Makefile.test flash

#include "system_config.h"
#include "gpio_config.h"
#include "spi1.h"
#include "i2c1.h"
#include "usart1.h"
#include "lsm6dsl.h"
#include "lis2mdl.h"
#include "lps22hd.h"
#include "motors.h"

#include <stdbool.h>

// Test mode selection
#define TEST_SENSORS    1   // Print sensor values
#define TEST_MOTORS     0   // Spin motors (BE CAREFUL - remove propellers!)

// Motor test parameters (only if TEST_MOTORS enabled)
#define MOTOR_TEST_POWER    0.1f    // 10% power - just enough to spin
#define MOTOR_TEST_DURATION 2000    // ms per motor

static void test_sensors(void);
static void test_motors(void);

int main(void) {
    // Initialize system clocks (84MHz), SysTick
    if (!system_init()) {
        while (1) {}  // Halt on failure
    }

    // Initialize debug UART
    usart1_init(NULL);
    usart1_puts("\r\n\r\n");
    usart1_puts("========================================\r\n");
    usart1_puts("STEVAL-DRONE01 Hardware Test\r\n");
    usart1_puts("========================================\r\n\r\n");

    // Initialize I2C1 for magnetometer and barometer
    usart1_puts("Initializing I2C1... ");
    i2c1_init(I2C1_SPEED_400KHZ);
    usart1_puts("OK\r\n");

    // Initialize LSM6DSL (IMU via SPI1)
    usart1_puts("Initializing LSM6DSL (IMU)... ");
    if (!lsm6dsl_init(NULL)) {
        usart1_puts("FAILED\r\n");
        while (1) {}
    }
    usart1_puts("OK\r\n");

    // Initialize LIS2MDL (magnetometer)
    usart1_puts("Initializing LIS2MDL (mag)... ");
    if (!lis2mdl_init(NULL)) {
        usart1_puts("FAILED\r\n");
        while (1) {}
    }
    usart1_puts("OK\r\n");

    // Initialize LPS22HD (barometer)
    usart1_puts("Initializing LPS22HD (baro)... ");
    if (!lps22hd_init(NULL)) {
        usart1_puts("FAILED\r\n");
        while (1) {}
    }
    usart1_puts("OK\r\n");

#if TEST_MOTORS
    // Initialize motors
    usart1_puts("Initializing motors... ");
    if (!motors_init(NULL)) {
        usart1_puts("FAILED\r\n");
        while (1) {}
    }
    usart1_puts("OK\r\n");
#endif

    usart1_puts("\r\nAll peripherals initialized.\r\n\r\n");

#if TEST_SENSORS
    test_sensors();
#endif

#if TEST_MOTORS
    test_motors();
#endif

    usart1_puts("Test complete.\r\n");
    while (1) {
        system_delay_ms(1000);
    }
}

static void test_sensors(void) {
    usart1_puts("--- Sensor Test (Ctrl+C to stop) ---\r\n\r\n");

    while (1) {
        // Read IMU
        lsm6dsl_data_t accel, gyro;
        lsm6dsl_read_all(&accel, &gyro);

        // Read magnetometer
        lis2mdl_data_t mag;
        lis2mdl_read(&mag);

        // Read barometer
        float pressure = lps22hd_read_pressure();
        float temp = lps22hd_read_temp();

        // Print values
        usart1_printf("Accel: %7.3f %7.3f %7.3f m/s2  ",
                      accel.x, accel.y, accel.z);
        usart1_printf("Gyro: %7.3f %7.3f %7.3f rad/s\r\n",
                      gyro.x, gyro.y, gyro.z);
        usart1_printf("Mag:   %7.1f %7.1f %7.1f uT    ",
                      mag.x, mag.y, mag.z);
        usart1_printf("Baro: %7.2f hPa  Temp: %5.1f C\r\n\r\n",
                      pressure, temp);

        system_delay_ms(500);  // 2 Hz update
    }
}

static void test_motors(void) {
    usart1_puts("--- Motor Test ---\r\n");
    usart1_puts("WARNING: Remove propellers before running!\r\n\r\n");

    // Wait for user to read warning
    usart1_puts("Starting in 3 seconds...\r\n");
    system_delay_ms(3000);

    motors_arm();

    motors_cmd_t cmd = {.motor = {0.0f, 0.0f, 0.0f, 0.0f}};

    for (int m = 0; m < 4; m++) {
        usart1_printf("Motor %d: spinning at %.0f%%\r\n", m + 1, MOTOR_TEST_POWER * 100);

        cmd.motor[m] = MOTOR_TEST_POWER;
        motors_set(&cmd);

        system_delay_ms(MOTOR_TEST_DURATION);

        cmd.motor[m] = 0.0f;
        motors_set(&cmd);

        usart1_printf("Motor %d: stopped\r\n\r\n", m + 1);
        system_delay_ms(500);
    }

    motors_disarm();
    usart1_puts("Motor test complete.\r\n\r\n");
}

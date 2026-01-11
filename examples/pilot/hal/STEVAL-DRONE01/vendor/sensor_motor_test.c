/**
 * Sensor + Motor test for STEVAL-FCU001V1
 *
 * Tests:
 *   1. All sensors initialized and reading data
 *   2. Motors spinning briefly (REMOVE PROPS!)
 *
 * LED feedback:
 *   1 blink  = Starting
 *   2 blinks = Sensors initialized
 *   3 blinks = Reading sensor data (loop)
 *   4 blinks = Motors test starting (DANGER!)
 *   Fast blink = Success, all tests passed
 *   Slow blink = Failure
 */

#include "stm32f4xx_hal.h"
#include "steval_fcu001_v1.h"
#include "steval_fcu001_v1_accelero.h"
#include "steval_fcu001_v1_gyro.h"
#include "steval_fcu001_v1_magneto.h"
#include "steval_fcu001_v1_pressure.h"

#include <stdbool.h>

// Motor PWM uses TIM4 CH1-4 on PB6, PB7, PB8, PB9
// Direct register access (no HAL TIM to avoid version issues)

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

// Sensor handles
static void *accel_handle = NULL;
static void *gyro_handle = NULL;
static void *mag_handle = NULL;
static void *press_handle = NULL;

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

/* Set motor speed (0-1000) */
static void motor_set(int motor, uint16_t speed) {
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    switch (motor) {
        case 0: TIM4_CCR1 = speed; break;
        case 1: TIM4_CCR2 = speed; break;
        case 2: TIM4_CCR3 = speed; break;
        case 3: TIM4_CCR4 = speed; break;
    }
}

/* Set all motors */
static void motors_set_all(uint16_t speed) {
    motor_set(0, speed);
    motor_set(1, speed);
    motor_set(2, speed);
    motor_set(3, speed);
}

/* Initialize all sensors */
static bool sensors_init(void) {
    // Initialize SPI bus
    if (Sensor_IO_SPI_Init() != COMPONENT_OK) {
        return false;
    }
    Sensor_IO_SPI_CS_Init_All();

    // Initialize accelerometer
    if (BSP_ACCELERO_Init(LSM6DSL_X_0, &accel_handle) != COMPONENT_OK) {
        return false;
    }
    BSP_ACCELERO_Sensor_Enable(accel_handle);

    // Initialize gyroscope
    if (BSP_GYRO_Init(LSM6DSL_G_0, &gyro_handle) != COMPONENT_OK) {
        return false;
    }
    BSP_GYRO_Sensor_Enable(gyro_handle);

    // Initialize magnetometer
    if (BSP_MAGNETO_Init(LIS2MDL_M_0, &mag_handle) != COMPONENT_OK) {
        return false;
    }
    BSP_MAGNETO_Sensor_Enable(mag_handle);

    // Initialize pressure sensor
    if (BSP_PRESSURE_Init(LPS22HB_P_0, &press_handle) != COMPONENT_OK) {
        return false;
    }
    BSP_PRESSURE_Sensor_Enable(press_handle);

    return true;
}

/* Read and validate sensor data */
static bool sensors_read_test(void) {
    SensorAxes_t accel, gyro, mag;
    float pressure;
    bool ok = true;

    // Read accelerometer (expect ~1g on one axis when level)
    if (BSP_ACCELERO_Get_Axes(accel_handle, &accel) != COMPONENT_OK) {
        ok = false;
    }

    // Read gyroscope (expect near-zero when stationary)
    if (BSP_GYRO_Get_Axes(gyro_handle, &gyro) != COMPONENT_OK) {
        ok = false;
    }

    // Read magnetometer
    if (BSP_MAGNETO_Get_Axes(mag_handle, &mag) != COMPONENT_OK) {
        ok = false;
    }

    // Read pressure (expect ~1000 hPa at sea level)
    if (BSP_PRESSURE_Get_Press(press_handle, &pressure) != COMPONENT_OK) {
        ok = false;
    }

    // Basic sanity checks
    // Accelerometer: should have some non-zero reading (gravity)
    int32_t accel_mag = accel.AXIS_X * accel.AXIS_X +
                        accel.AXIS_Y * accel.AXIS_Y +
                        accel.AXIS_Z * accel.AXIS_Z;
    if (accel_mag < 500000) {  // Should be ~1000000 (1g = 1000mg squared)
        ok = false;
    }

    // Pressure: should be in reasonable range (800-1200 hPa)
    if (pressure < 800.0f || pressure > 1200.0f) {
        ok = false;
    }

    return ok;
}

int main(void) {
    // Set clock before HAL_Init
    SystemCoreClock = 16000000;

    // Initialize HAL
    HAL_Init();

    // Initialize LED
    BSP_LED_Init(LED1);
    BSP_LED_Off(LED1);

    // 1 blink = Starting
    blink_n(1, 200, 200);

    // Initialize sensors
    if (!sensors_init()) {
        // Sensor init failed - slow blink
        while (1) {
            BSP_LED_Toggle(LED1);
            HAL_Delay(1000);
        }
    }

    // 2 blinks = Sensors initialized
    blink_n(2, 200, 200);

    // Read sensor data a few times
    bool sensor_ok = true;
    for (int i = 0; i < 10; i++) {
        if (!sensors_read_test()) {
            sensor_ok = false;
        }
        BSP_LED_Toggle(LED1);
        HAL_Delay(100);
    }
    BSP_LED_Off(LED1);
    HAL_Delay(300);

    if (!sensor_ok) {
        // Sensor read failed - slow blink
        while (1) {
            BSP_LED_Toggle(LED1);
            HAL_Delay(1000);
        }
    }

    // 3 blinks = Sensor data OK
    blink_n(3, 200, 200);

    // Initialize motors
    if (!motors_init()) {
        // Motor init failed - slow blink
        while (1) {
            BSP_LED_Toggle(LED1);
            HAL_Delay(1000);
        }
    }

    // 4 blinks = Motors test starting (WARNING!)
    blink_n(4, 200, 200);

    // Brief motor test - spin each motor at low speed
    // *** REMOVE PROPELLERS BEFORE RUNNING! ***
    for (int m = 0; m < 4; m++) {
        motor_set(m, 100);  // 10% duty cycle
        HAL_Delay(500);
        motor_set(m, 0);
        HAL_Delay(200);
    }

    // All motors together briefly
    motors_set_all(100);
    HAL_Delay(500);
    motors_set_all(0);

    // Fast blink = All tests passed!
    while (1) {
        BSP_LED_Toggle(LED1);
        HAL_Delay(100);
    }
}

/* Called by HAL_Init */
void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

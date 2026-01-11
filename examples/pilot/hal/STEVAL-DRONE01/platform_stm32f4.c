// STM32F4 Platform Layer for Pilot Example
//
// Implements the platform interface using STM32 HAL + BSP for STEVAL-FCU001V1.
// Sensors are accessed via SPI using the official ST BSP drivers.

#include "platform_stm32f4.h"
#include "stm32f4xx_hal.h"
#include "steval_fcu001_v1.h"
#include "steval_fcu001_v1_accelero.h"
#include "steval_fcu001_v1_gyro.h"
#include "steval_fcu001_v1_magneto.h"
#include "steval_fcu001_v1_pressure.h"
#include "steval_fcu001_v1_temperature.h"
#include "tim4.h"
#include "motors.h"
#include "usart1.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

#define CALIBRATION_SAMPLES     500     // Gyro calibration samples
#define BARO_CALIBRATION_SAMPLES 50     // Barometer calibration samples

// Conversion constants
#define GRAVITY             9.80665f    // m/s²
#ifndef M_PI
#define M_PI                3.14159265358979323846f
#endif
#define DEG_TO_RAD          (M_PI / 180.0f)
#define MGAUSS_TO_UT        0.1f        // 1 mGauss = 0.1 µT

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;
static bool s_calibrated = false;
static bool s_armed = false;

// Sensor handles
static void *s_accel_handle = NULL;
static void *s_gyro_handle = NULL;
static void *s_mag_handle = NULL;
static void *s_press_handle = NULL;
static void *s_temp_handle = NULL;

// Gyro bias (rad/s) - determined during calibration
static float s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};

// Barometer reference pressure (hPa)
static float s_ref_pressure = 0.0f;

// ----------------------------------------------------------------------------
// HAL MSP Initialization (called by HAL_Init)
// ----------------------------------------------------------------------------

void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

// ----------------------------------------------------------------------------
// Platform Interface
// ----------------------------------------------------------------------------

int platform_init(void) {
    // Set system clock before HAL_Init
    SystemCoreClock = 16000000;  // 16 MHz HSI

    // Initialize HAL
    if (HAL_Init() != HAL_OK) {
        return -1;
    }

    // Initialize debug serial early (before sensors)
    platform_debug_init();

    // Initialize LED for status indication
    BSP_LED_Init(LED1);
    BSP_LED_Off(LED1);

    // Initialize sensor SPI bus
    if (Sensor_IO_SPI_Init() != COMPONENT_OK) {
        return -1;
    }
    Sensor_IO_SPI_CS_Init_All();

    // Initialize accelerometer (LSM6DSL)
    if (BSP_ACCELERO_Init(LSM6DSL_X_0, &s_accel_handle) != COMPONENT_OK) {
        return -1;
    }
    BSP_ACCELERO_Sensor_Enable(s_accel_handle);

    // Initialize gyroscope (LSM6DSL - same chip as accelerometer)
    if (BSP_GYRO_Init(LSM6DSL_G_0, &s_gyro_handle) != COMPONENT_OK) {
        return -1;
    }
    BSP_GYRO_Sensor_Enable(s_gyro_handle);

    // Initialize magnetometer (LIS2MDL)
    if (BSP_MAGNETO_Init(LIS2MDL_M_0, &s_mag_handle) != COMPONENT_OK) {
        return -1;
    }
    BSP_MAGNETO_Sensor_Enable(s_mag_handle);

    // Initialize pressure sensor (LPS22HB)
    if (BSP_PRESSURE_Init(LPS22HB_P_0, &s_press_handle) != COMPONENT_OK) {
        return -1;
    }
    BSP_PRESSURE_Sensor_Enable(s_press_handle);

    // Initialize temperature sensor (LPS22HB - same chip as pressure)
    if (BSP_TEMPERATURE_Init(LPS22HB_T_0, &s_temp_handle) != COMPONENT_OK) {
        return -1;
    }
    BSP_TEMPERATURE_Sensor_Enable(s_temp_handle);

    // Initialize motors (TIM4 PWM)
    if (!motors_init(NULL)) {
        return -1;
    }

    s_initialized = true;
    s_calibrated = false;
    s_armed = false;

    // Blink LED to indicate successful init
    for (int i = 0; i < 3; i++) {
        BSP_LED_On(LED1);
        HAL_Delay(100);
        BSP_LED_Off(LED1);
        HAL_Delay(100);
    }

    return 0;
}

int platform_calibrate(void) {
    if (!s_initialized) {
        return -1;
    }

    // -------------------------------------------------------------------------
    // Gyro bias calibration - average readings while stationary
    // -------------------------------------------------------------------------
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    SensorAxes_t gyro_axes;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (BSP_GYRO_Get_Axes(s_gyro_handle, &gyro_axes) == COMPONENT_OK) {
            // Convert mdps to rad/s for bias accumulation
            gyro_sum[0] += (float)gyro_axes.AXIS_X * 0.001f * DEG_TO_RAD;
            gyro_sum[1] += (float)gyro_axes.AXIS_Y * 0.001f * DEG_TO_RAD;
            gyro_sum[2] += (float)gyro_axes.AXIS_Z * 0.001f * DEG_TO_RAD;
        }
        HAL_Delay(2);  // ~500Hz
    }

    s_gyro_bias[0] = gyro_sum[0] / CALIBRATION_SAMPLES;
    s_gyro_bias[1] = gyro_sum[1] / CALIBRATION_SAMPLES;
    s_gyro_bias[2] = gyro_sum[2] / CALIBRATION_SAMPLES;

    // -------------------------------------------------------------------------
    // Barometer reference calibration
    // -------------------------------------------------------------------------
    float pressure_sum = 0.0f;
    float pressure;

    for (int i = 0; i < BARO_CALIBRATION_SAMPLES; i++) {
        if (BSP_PRESSURE_Get_Press(s_press_handle, &pressure) == COMPONENT_OK) {
            pressure_sum += pressure;
        }
        HAL_Delay(20);
    }

    s_ref_pressure = pressure_sum / BARO_CALIBRATION_SAMPLES;

    s_calibrated = true;
    return 0;
}

void platform_read_sensors(sensor_data_t *sensors) {
    SensorAxes_t axes;

    // -------------------------------------------------------------------------
    // Accelerometer (BSP returns mg, convert to m/s²)
    // -------------------------------------------------------------------------
    if (BSP_ACCELERO_Get_Axes(s_accel_handle, &axes) == COMPONENT_OK) {
        // mg to m/s²: divide by 1000 to get g, multiply by 9.81
        sensors->accel[0] = (float)axes.AXIS_X * 0.001f * GRAVITY;
        sensors->accel[1] = (float)axes.AXIS_Y * 0.001f * GRAVITY;
        sensors->accel[2] = (float)axes.AXIS_Z * 0.001f * GRAVITY;
    }

    // -------------------------------------------------------------------------
    // Gyroscope (BSP returns mdps, convert to rad/s)
    // -------------------------------------------------------------------------
    if (BSP_GYRO_Get_Axes(s_gyro_handle, &axes) == COMPONENT_OK) {
        // mdps to rad/s: divide by 1000 to get dps, multiply by pi/180
        sensors->gyro[0] = (float)axes.AXIS_X * 0.001f * DEG_TO_RAD - s_gyro_bias[0];
        sensors->gyro[1] = (float)axes.AXIS_Y * 0.001f * DEG_TO_RAD - s_gyro_bias[1];
        sensors->gyro[2] = (float)axes.AXIS_Z * 0.001f * DEG_TO_RAD - s_gyro_bias[2];
    }

    // -------------------------------------------------------------------------
    // Magnetometer (BSP returns mGauss, convert to µT)
    // -------------------------------------------------------------------------
    if (BSP_MAGNETO_Get_Axes(s_mag_handle, &axes) == COMPONENT_OK) {
        // mGauss to µT: 1 Gauss = 100 µT, so 1 mGauss = 0.1 µT
        sensors->mag[0] = (float)axes.AXIS_X * MGAUSS_TO_UT;
        sensors->mag[1] = (float)axes.AXIS_Y * MGAUSS_TO_UT;
        sensors->mag[2] = (float)axes.AXIS_Z * MGAUSS_TO_UT;
        sensors->mag_valid = true;
    } else {
        sensors->mag_valid = false;
    }

    // -------------------------------------------------------------------------
    // Barometer
    // -------------------------------------------------------------------------
    float pressure, temperature;
    if (BSP_PRESSURE_Get_Press(s_press_handle, &pressure) == COMPONENT_OK) {
        sensors->pressure_hpa = pressure;
        sensors->baro_valid = true;
    } else {
        sensors->baro_valid = false;
    }

    if (BSP_TEMPERATURE_Get_Temp(s_temp_handle, &temperature) == COMPONENT_OK) {
        sensors->baro_temp_c = temperature;
    }

    // No GPS on this platform
    sensors->gps_x = 0.0f;
    sensors->gps_y = 0.0f;
    sensors->gps_z = 0.0f;
    sensors->gps_valid = false;
}

void platform_write_motors(const motor_cmd_t *cmd) {
    if (!s_armed) {
        return;
    }

    motors_cmd_t motor_cmd;
    for (int i = 0; i < 4; i++) {
        motor_cmd.motor[i] = cmd->motor[i];
    }
    motors_set(&motor_cmd);
}

// ----------------------------------------------------------------------------
// Extended Platform Interface
// ----------------------------------------------------------------------------

void platform_arm(void) {
    if (s_initialized && s_calibrated) {
        motors_arm();
        s_armed = true;
        BSP_LED_On(LED1);  // LED on when armed
    }
}

void platform_disarm(void) {
    motors_disarm();
    s_armed = false;
    BSP_LED_Off(LED1);  // LED off when disarmed
}

uint32_t platform_get_time_ms(void) {
    return HAL_GetTick();
}

uint32_t platform_get_time_us(void) {
    // HAL doesn't provide microsecond resolution by default
    // For now, return milliseconds * 1000
    return HAL_GetTick() * 1000;
}

void platform_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void platform_delay_us(uint32_t us) {
    // Simple busy-wait for microseconds
    uint32_t start = platform_get_time_us();
    while ((platform_get_time_us() - start) < us) {
        __NOP();
    }
}

void platform_debug_init(void) {
    // Initialize USART1 for debug output (115200 baud, TX only)
    usart1_init(NULL);  // NULL = use defaults
}

void platform_debug_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    // usart1_printf doesn't support va_list, use fixed buffer
    char buf[128];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        usart1_write(buf, len < (int)sizeof(buf) ? len : (int)sizeof(buf) - 1);
    }
}

void platform_emergency_stop(void) {
    motors_emergency_stop();
    s_armed = false;

    // Fast blink LED to indicate emergency
    for (int i = 0; i < 10; i++) {
        BSP_LED_Toggle(LED1);
        HAL_Delay(50);
    }
    BSP_LED_Off(LED1);
}

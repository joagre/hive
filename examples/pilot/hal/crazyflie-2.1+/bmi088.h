// BMI088 IMU Driver for Crazyflie 2.1+
//
// The BMI088 has separate accelerometer and gyroscope with independent
// SPI interfaces (separate chip selects). This driver handles both.
//
// Accelerometer: 16-bit, +/-3/6/12/24g
// Gyroscope: 16-bit, +/-125/250/500/1000/2000 dps
//
// Reference: Bosch BMI088 Datasheet (BST-BMI088-DS001)

#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Accelerometer range options
typedef enum {
    BMI088_ACC_RANGE_3G = 0x00,
    BMI088_ACC_RANGE_6G = 0x01,
    BMI088_ACC_RANGE_12G = 0x02,
    BMI088_ACC_RANGE_24G = 0x03
} bmi088_acc_range_t;

// Gyroscope range options
typedef enum {
    BMI088_GYRO_RANGE_2000DPS = 0x00,
    BMI088_GYRO_RANGE_1000DPS = 0x01,
    BMI088_GYRO_RANGE_500DPS = 0x02,
    BMI088_GYRO_RANGE_250DPS = 0x03,
    BMI088_GYRO_RANGE_125DPS = 0x04
} bmi088_gyro_range_t;

// Accelerometer ODR/bandwidth options
typedef enum {
    BMI088_ACC_ODR_12_5HZ = 0x05,
    BMI088_ACC_ODR_25HZ = 0x06,
    BMI088_ACC_ODR_50HZ = 0x07,
    BMI088_ACC_ODR_100HZ = 0x08,
    BMI088_ACC_ODR_200HZ = 0x09,
    BMI088_ACC_ODR_400HZ = 0x0A,
    BMI088_ACC_ODR_800HZ = 0x0B,
    BMI088_ACC_ODR_1600HZ = 0x0C
} bmi088_acc_odr_t;

// Gyroscope ODR/bandwidth options
typedef enum {
    BMI088_GYRO_ODR_2000HZ_BW_532HZ = 0x00,
    BMI088_GYRO_ODR_2000HZ_BW_230HZ = 0x01,
    BMI088_GYRO_ODR_1000HZ_BW_116HZ = 0x02,
    BMI088_GYRO_ODR_400HZ_BW_47HZ = 0x03,
    BMI088_GYRO_ODR_200HZ_BW_23HZ = 0x04,
    BMI088_GYRO_ODR_100HZ_BW_12HZ = 0x05,
    BMI088_GYRO_ODR_200HZ_BW_64HZ = 0x06,
    BMI088_GYRO_ODR_100HZ_BW_32HZ = 0x07
} bmi088_gyro_odr_t;

// Configuration structure
typedef struct {
    bmi088_acc_range_t acc_range;
    bmi088_acc_odr_t acc_odr;
    bmi088_gyro_range_t gyro_range;
    bmi088_gyro_odr_t gyro_odr;
} bmi088_config_t;

// Default configuration (good for flight control)
#define BMI088_CONFIG_DEFAULT                                              \
    {                                                                      \
        .acc_range = BMI088_ACC_RANGE_6G, .acc_odr = BMI088_ACC_ODR_400HZ, \
        .gyro_range = BMI088_GYRO_RANGE_1000DPS,                           \
        .gyro_odr = BMI088_GYRO_ODR_400HZ_BW_47HZ                          \
    }

// ----------------------------------------------------------------------------
// Data Structures
// ----------------------------------------------------------------------------

// Raw sensor data (16-bit signed)
typedef struct {
    int16_t x, y, z;
} bmi088_raw_t;

// Scaled sensor data (SI units)
typedef struct {
    float x, y, z; // Accel: m/s^2, Gyro: rad/s
} bmi088_data_t;

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

// Initialize BMI088 (both accelerometer and gyroscope)
// config: Configuration structure (or NULL for defaults)
// Returns: true on success, false on error
bool bmi088_init(const bmi088_config_t *config);

// Check if sensor is initialized and responding
bool bmi088_is_ready(void);

// Read accelerometer data (m/s^2)
bool bmi088_read_accel(bmi088_data_t *data);

// Read gyroscope data (rad/s)
bool bmi088_read_gyro(bmi088_data_t *data);

// Read both accel and gyro in one call
bool bmi088_read_all(bmi088_data_t *accel, bmi088_data_t *gyro);

// Read raw accelerometer data (16-bit signed)
bool bmi088_read_accel_raw(bmi088_raw_t *data);

// Read raw gyroscope data (16-bit signed)
bool bmi088_read_gyro_raw(bmi088_raw_t *data);

// Read temperature (degC) from accelerometer
bool bmi088_read_temp(float *temp_c);

// Perform self-test (returns true if passed)
bool bmi088_self_test(void);

// Software reset (both sensors)
void bmi088_reset(void);

// ----------------------------------------------------------------------------
// Low-Level SPI Interface (to be implemented by platform)
// ----------------------------------------------------------------------------

// These functions must be implemented by the platform layer
extern void bmi088_acc_cs_low(void);
extern void bmi088_acc_cs_high(void);
extern void bmi088_gyro_cs_low(void);
extern void bmi088_gyro_cs_high(void);
extern uint8_t bmi088_spi_transfer(uint8_t data);
extern void bmi088_delay_us(uint32_t us);
extern void bmi088_delay_ms(uint32_t ms);

#endif // BMI088_H

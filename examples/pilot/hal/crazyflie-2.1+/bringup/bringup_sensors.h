// Crazyflie 2.1+ Bring-Up - Sensor Tests
//
// Tests for BMI088 IMU, BMP388 barometer, and Flow deck sensors

#ifndef BRINGUP_SENSORS_H
#define BRINGUP_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

// Expected chip IDs
#define BMI088_ACCEL_CHIP_ID 0x1E
#define BMI088_GYRO_CHIP_ID 0x0F
#define BMP388_CHIP_ID 0x50
#define VL53L1X_MODEL_ID 0xEACC
#define PMW3901_PRODUCT_ID 0x49

// Sensor data structures
typedef struct {
    float x, y, z; // g
} accel_data_t;

typedef struct {
    float x, y, z; // deg/s
} gyro_data_t;

typedef struct {
    float pressure;    // Pa
    float temperature; // C
} baro_data_t;

typedef struct {
    int16_t delta_x;
    int16_t delta_y;
    uint8_t squal; // Surface quality
} flow_data_t;

// Test results
typedef struct {
    bool accel_id_ok;
    bool gyro_id_ok;
    bool baro_id_ok;
    bool tof_id_ok;
    bool flow_id_ok;
    bool accel_data_ok;
    bool gyro_data_ok;
    bool baro_data_ok;
    bool tof_data_ok;
    bool flow_deck_present;
} sensor_test_results_t;

// Initialize sensors (call after i2c_init and spi_init)
void sensors_init(void);

// Chip ID tests (Phase 3)
bool sensor_test_accel_id(uint8_t *chip_id);
bool sensor_test_gyro_id(uint8_t *chip_id);
bool sensor_test_baro_id(uint8_t *chip_id);
bool sensor_test_tof_id(uint16_t *model_id);
bool sensor_test_flow_id(uint8_t *product_id);

// Data readout tests (Phase 4)
bool sensor_read_accel(accel_data_t *data);
bool sensor_read_gyro(gyro_data_t *data);
bool sensor_read_baro(baro_data_t *data);
bool sensor_read_tof(uint16_t *range_mm);
bool sensor_read_flow(flow_data_t *data);

// Sanity checks for sensor data
bool sensor_check_accel(const accel_data_t *data); // X,Y near 0, Z near 1g
bool sensor_check_gyro(const gyro_data_t *data);   // All near 0
bool sensor_check_baro(const baro_data_t *data);   // 95-106 kPa, 15-35 C

// Run all sensor tests
void sensor_run_all_tests(sensor_test_results_t *results);

// SPI initialization for PMW3901 (separate from I2C)
void spi_init(void);

// Check if Flow deck is present (based on VL53L1x detection)
bool sensor_flow_deck_present(void);

#endif // BRINGUP_SENSORS_H

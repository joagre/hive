// BMP388 Barometer Driver for Crazyflie 2.1+
//
// The BMP388 is a high-precision pressure sensor with temperature
// compensation. Uses I2C interface.
//
// Range: 300-1250 hPa
// Accuracy: ±50 Pa absolute, ±8 Pa relative
//
// Reference: Bosch BMP388 Datasheet (BST-BMP388-DS001)

#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Oversampling options (higher = less noise, slower)
typedef enum {
    BMP388_OSR_X1   = 0x00,   // No oversampling (single)
    BMP388_OSR_X2   = 0x01,
    BMP388_OSR_X4   = 0x02,
    BMP388_OSR_X8   = 0x03,
    BMP388_OSR_X16  = 0x04,
    BMP388_OSR_X32  = 0x05
} bmp388_osr_t;

// Output data rate options
typedef enum {
    BMP388_ODR_200HZ   = 0x00,
    BMP388_ODR_100HZ   = 0x01,
    BMP388_ODR_50HZ    = 0x02,
    BMP388_ODR_25HZ    = 0x03,
    BMP388_ODR_12_5HZ  = 0x04,
    BMP388_ODR_6_25HZ  = 0x05,
    BMP388_ODR_3_1HZ   = 0x06,
    BMP388_ODR_1_5HZ   = 0x07,
    BMP388_ODR_0_78HZ  = 0x08,
    BMP388_ODR_0_39HZ  = 0x09,
    BMP388_ODR_0_2HZ   = 0x0A,
    BMP388_ODR_0_1HZ   = 0x0B,
    BMP388_ODR_0_05HZ  = 0x0C,
    BMP388_ODR_0_02HZ  = 0x0D,
    BMP388_ODR_0_01HZ  = 0x0E,
    BMP388_ODR_0_006HZ = 0x0F,
    BMP388_ODR_0_003HZ = 0x10,
    BMP388_ODR_0_0015HZ = 0x11
} bmp388_odr_t;

// IIR filter coefficient (reduces noise, adds latency)
typedef enum {
    BMP388_IIR_COEF_0   = 0x00,   // Disabled
    BMP388_IIR_COEF_1   = 0x01,
    BMP388_IIR_COEF_3   = 0x02,
    BMP388_IIR_COEF_7   = 0x03,
    BMP388_IIR_COEF_15  = 0x04,
    BMP388_IIR_COEF_31  = 0x05,
    BMP388_IIR_COEF_63  = 0x06,
    BMP388_IIR_COEF_127 = 0x07
} bmp388_iir_t;

// Power modes
typedef enum {
    BMP388_MODE_SLEEP  = 0x00,
    BMP388_MODE_FORCED = 0x01,   // Single measurement
    BMP388_MODE_NORMAL = 0x03    // Continuous
} bmp388_mode_t;

// Configuration structure
typedef struct {
    bmp388_osr_t  press_osr;    // Pressure oversampling
    bmp388_osr_t  temp_osr;     // Temperature oversampling
    bmp388_odr_t  odr;          // Output data rate
    bmp388_iir_t  iir_coef;     // IIR filter coefficient
} bmp388_config_t;

// Default configuration (good for altitude measurement)
#define BMP388_CONFIG_DEFAULT { \
    .press_osr = BMP388_OSR_X8, \
    .temp_osr  = BMP388_OSR_X1, \
    .odr       = BMP388_ODR_50HZ, \
    .iir_coef  = BMP388_IIR_COEF_3 \
}

// High-precision configuration (slower but more accurate)
#define BMP388_CONFIG_HIGHRES { \
    .press_osr = BMP388_OSR_X32, \
    .temp_osr  = BMP388_OSR_X2, \
    .odr       = BMP388_ODR_12_5HZ, \
    .iir_coef  = BMP388_IIR_COEF_7 \
}

// ----------------------------------------------------------------------------
// Data Structures
// ----------------------------------------------------------------------------

// Sensor data
typedef struct {
    float pressure_pa;      // Pressure in Pascals
    float temperature_c;    // Temperature in Celsius
} bmp388_data_t;

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

// Initialize BMP388
// config: Configuration structure (or NULL for defaults)
// Returns: true on success, false on error
bool bmp388_init(const bmp388_config_t *config);

// Check if sensor is initialized and responding
bool bmp388_is_ready(void);

// Read pressure and temperature
bool bmp388_read(bmp388_data_t *data);

// Read pressure only (hPa)
bool bmp388_read_pressure(float *pressure_hpa);

// Read temperature only (°C)
bool bmp388_read_temperature(float *temp_c);

// Convert pressure to altitude (meters) relative to reference
// Uses standard barometric formula
float bmp388_pressure_to_altitude(float pressure_pa, float ref_pressure_pa);

// Trigger single measurement (forced mode)
bool bmp388_trigger(void);

// Check if data is ready
bool bmp388_data_ready(void);

// Software reset
void bmp388_reset(void);

// ----------------------------------------------------------------------------
// Low-Level I2C Interface (to be implemented by platform)
// ----------------------------------------------------------------------------

// I2C address (default 0x77, can be 0x76 if SDO pin grounded)
#define BMP388_I2C_ADDR_DEFAULT 0x77
#define BMP388_I2C_ADDR_ALT     0x76

// These functions must be implemented by the platform layer
extern bool bmp388_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
extern bool bmp388_i2c_write(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
extern void bmp388_delay_ms(uint32_t ms);

#endif // BMP388_H

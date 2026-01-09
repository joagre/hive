// LPS22HD Barometer driver for STEVAL-DRONE01
//
// I2C interface to the LPS22HD pressure/temperature sensor.

#include "lps22hd.h"
#include "i2c1.h"
#include "system_config.h"
#include <math.h>

// ----------------------------------------------------------------------------
// Register addresses
// ----------------------------------------------------------------------------

#define LPS22HD_I2C_ADDR        0x5D  // 7-bit address (SA0 = 1 on STEVAL)

#define LPS22HD_INTERRUPT_CFG   0x0B
#define LPS22HD_THS_P_L         0x0C
#define LPS22HD_THS_P_H         0x0D
#define LPS22HD_WHO_AM_I        0x0F
#define LPS22HD_CTRL_REG1       0x10
#define LPS22HD_CTRL_REG2       0x11
#define LPS22HD_CTRL_REG3       0x12
#define LPS22HD_FIFO_CTRL       0x14
#define LPS22HD_REF_P_XL        0x15
#define LPS22HD_REF_P_L         0x16
#define LPS22HD_REF_P_H         0x17
#define LPS22HD_RPDS_L          0x18
#define LPS22HD_RPDS_H          0x19
#define LPS22HD_RES_CONF        0x1A
#define LPS22HD_INT_SOURCE      0x25
#define LPS22HD_FIFO_STATUS     0x26
#define LPS22HD_STATUS          0x27
#define LPS22HD_PRESS_OUT_XL    0x28
#define LPS22HD_PRESS_OUT_L     0x29
#define LPS22HD_PRESS_OUT_H     0x2A
#define LPS22HD_TEMP_OUT_L      0x2B
#define LPS22HD_TEMP_OUT_H      0x2C
#define LPS22HD_LPFP_RES        0x33

#define LPS22HD_WHO_AM_I_VALUE  0xB1  // Expected WHO_AM_I response

// Status register bits
#define LPS22HD_STATUS_P_DA     0x01  // Pressure data available
#define LPS22HD_STATUS_T_DA     0x02  // Temperature data available

// CTRL_REG2 bits
#define LPS22HD_CTRL2_ONE_SHOT  0x01  // One-shot trigger
#define LPS22HD_CTRL2_SWRESET   0x04  // Software reset
#define LPS22HD_CTRL2_BOOT      0x80  // Reboot memory

// ----------------------------------------------------------------------------
// Conversion constants
// ----------------------------------------------------------------------------

// Pressure sensitivity: 4096 LSB/hPa
#define LPS22HD_PRESS_SENSITIVITY  4096.0f

// Temperature sensitivity: 100 LSB/°C
#define LPS22HD_TEMP_SENSITIVITY   100.0f

// Standard sea level pressure (hPa)
#define STD_SEA_LEVEL_PRESSURE     1013.25f

// Barometric formula constant
#define BARO_ALTITUDE_CONST        44330.0f
#define BARO_ALTITUDE_EXP          0.1903f

// ----------------------------------------------------------------------------
// Static state
// ----------------------------------------------------------------------------

static lps22hd_config_t s_config;
static float s_reference_pressure = STD_SEA_LEVEL_PRESSURE;

// ----------------------------------------------------------------------------
// I2C low-level
// ----------------------------------------------------------------------------

static void i2c_write_reg(uint8_t reg, uint8_t value) {
    i2c1_write_reg(LPS22HD_I2C_ADDR, reg, value);
}

static uint8_t i2c_read_reg(uint8_t reg) {
    uint8_t value = 0;
    i2c1_read_reg(LPS22HD_I2C_ADDR, reg, &value);
    return value;
}

static void i2c_read_burst(uint8_t reg, uint8_t *buf, uint8_t len) {
    // For LPS22HD, set MSB for auto-increment
    i2c1_read_regs(LPS22HD_I2C_ADDR, reg | 0x80, buf, len);
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

bool lps22hd_init(const lps22hd_config_t *config) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (lps22hd_config_t)LPS22HD_CONFIG_DEFAULT;
    }

    // Check WHO_AM_I
    if (!lps22hd_is_ready()) {
        return false;
    }

    // Software reset
    i2c_write_reg(LPS22HD_CTRL_REG2, LPS22HD_CTRL2_SWRESET);
    system_delay_ms(10);

    // Wait for reset to complete
    while (i2c_read_reg(LPS22HD_CTRL_REG2) & LPS22HD_CTRL2_SWRESET);

    // Reboot memory content
    i2c_write_reg(LPS22HD_CTRL_REG2, LPS22HD_CTRL2_BOOT);
    system_delay_ms(10);

    // Configure CTRL_REG1: ODR, LPF, BDU
    uint8_t ctrl1 = (s_config.odr << 4);
    if (s_config.lpf != LPS22HD_LPF_DISABLED) {
        ctrl1 |= 0x08;  // EN_LPFP
        ctrl1 |= (s_config.lpf & 0x03) << 2;  // LPFP_CFG
    }
    if (s_config.bdu) {
        ctrl1 |= 0x02;  // BDU
    }
    i2c_write_reg(LPS22HD_CTRL_REG1, ctrl1);

    // Set default reference to current sea level
    s_reference_pressure = STD_SEA_LEVEL_PRESSURE;

    return true;
}

bool lps22hd_is_ready(void) {
    uint8_t who = i2c_read_reg(LPS22HD_WHO_AM_I);
    return (who == LPS22HD_WHO_AM_I_VALUE);
}

bool lps22hd_pressure_ready(void) {
    uint8_t status = i2c_read_reg(LPS22HD_STATUS);
    return (status & LPS22HD_STATUS_P_DA) != 0;
}

bool lps22hd_temp_ready(void) {
    uint8_t status = i2c_read_reg(LPS22HD_STATUS);
    return (status & LPS22HD_STATUS_T_DA) != 0;
}

void lps22hd_read_raw(lps22hd_raw_t *data) {
    uint8_t buf[5];
    // Read pressure (3 bytes) and temperature (2 bytes) in one burst
    i2c_read_burst(LPS22HD_PRESS_OUT_XL, buf, 5);

    // Pressure: 24-bit unsigned, stored in 3 bytes (XL, L, H)
    data->pressure = (int32_t)(buf[2] << 16 | buf[1] << 8 | buf[0]);

    // Temperature: 16-bit signed
    data->temperature = (int16_t)(buf[4] << 8 | buf[3]);
}

void lps22hd_read(lps22hd_data_t *data) {
    lps22hd_raw_t raw;
    lps22hd_read_raw(&raw);

    // Convert pressure: raw / 4096 = hPa
    data->pressure_hpa = (float)raw.pressure / LPS22HD_PRESS_SENSITIVITY;

    // Convert temperature: raw / 100 = °C
    data->temp_c = (float)raw.temperature / LPS22HD_TEMP_SENSITIVITY;
}

float lps22hd_read_pressure(void) {
    uint8_t buf[3];
    i2c_read_burst(LPS22HD_PRESS_OUT_XL, buf, 3);

    int32_t raw = (int32_t)(buf[2] << 16 | buf[1] << 8 | buf[0]);
    return (float)raw / LPS22HD_PRESS_SENSITIVITY;
}

float lps22hd_read_temp(void) {
    uint8_t buf[2];
    i2c_read_burst(LPS22HD_TEMP_OUT_L, buf, 2);

    int16_t raw = (int16_t)(buf[1] << 8 | buf[0]);
    return (float)raw / LPS22HD_TEMP_SENSITIVITY;
}

void lps22hd_set_reference(float pressure_hpa) {
    s_reference_pressure = pressure_hpa;
}

float lps22hd_get_reference(void) {
    return s_reference_pressure;
}

float lps22hd_altitude(float pressure_hpa) {
    // International barometric formula:
    // altitude = 44330 * (1 - (P / P0)^0.1903)
    //
    // P  = current pressure
    // P0 = reference pressure at ground level
    //
    // This gives altitude in meters relative to reference.

    if (s_reference_pressure <= 0.0f || pressure_hpa <= 0.0f) {
        return 0.0f;
    }

    float ratio = pressure_hpa / s_reference_pressure;
    float altitude = BARO_ALTITUDE_CONST * (1.0f - powf(ratio, BARO_ALTITUDE_EXP));

    return altitude;
}

float lps22hd_read_altitude(void) {
    float pressure = lps22hd_read_pressure();
    return lps22hd_altitude(pressure);
}

void lps22hd_trigger_one_shot(void) {
    uint8_t ctrl2 = i2c_read_reg(LPS22HD_CTRL_REG2);
    ctrl2 |= LPS22HD_CTRL2_ONE_SHOT;
    i2c_write_reg(LPS22HD_CTRL_REG2, ctrl2);
}

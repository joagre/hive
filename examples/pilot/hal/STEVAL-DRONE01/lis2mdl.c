// LIS2MDL Magnetometer driver for STEVAL-DRONE01
//
// I2C interface to the LIS2MDL 3-axis magnetometer.

#include "lis2mdl.h"
#include "i2c1.h"
#include "system_config.h"
#include <math.h>

// ----------------------------------------------------------------------------
// Register addresses
// ----------------------------------------------------------------------------

#define LIS2MDL_I2C_ADDR        0x1E  // 7-bit address (0x3C write, 0x3D read)

#define LIS2MDL_OFFSET_X_REG_L  0x45
#define LIS2MDL_OFFSET_X_REG_H  0x46
#define LIS2MDL_OFFSET_Y_REG_L  0x47
#define LIS2MDL_OFFSET_Y_REG_H  0x48
#define LIS2MDL_OFFSET_Z_REG_L  0x49
#define LIS2MDL_OFFSET_Z_REG_H  0x4A
#define LIS2MDL_WHO_AM_I        0x4F
#define LIS2MDL_CFG_REG_A       0x60
#define LIS2MDL_CFG_REG_B       0x61
#define LIS2MDL_CFG_REG_C       0x62
#define LIS2MDL_INT_CRTL_REG    0x63
#define LIS2MDL_INT_SOURCE_REG  0x64
#define LIS2MDL_INT_THS_L_REG   0x65
#define LIS2MDL_INT_THS_H_REG   0x66
#define LIS2MDL_STATUS_REG      0x67
#define LIS2MDL_OUTX_L_REG      0x68
#define LIS2MDL_OUTX_H_REG      0x69
#define LIS2MDL_OUTY_L_REG      0x6A
#define LIS2MDL_OUTY_H_REG      0x6B
#define LIS2MDL_OUTZ_L_REG      0x6C
#define LIS2MDL_OUTZ_H_REG      0x6D
#define LIS2MDL_TEMP_OUT_L_REG  0x6E
#define LIS2MDL_TEMP_OUT_H_REG  0x6F

#define LIS2MDL_WHO_AM_I_VALUE  0x40  // Expected WHO_AM_I response

// Status register bits
#define LIS2MDL_STATUS_ZYXDA    0x08  // XYZ data available

// ----------------------------------------------------------------------------
// Conversion constants
// ----------------------------------------------------------------------------

// Magnetometer sensitivity: 1.5 mG/LSB = 0.15 µT/LSB
#define LIS2MDL_SENSITIVITY     0.15f  // µT per LSB

// ----------------------------------------------------------------------------
// Static state
// ----------------------------------------------------------------------------

static lis2mdl_config_t s_config;

// ----------------------------------------------------------------------------
// I2C low-level
// ----------------------------------------------------------------------------

static void i2c_write_reg(uint8_t reg, uint8_t value) {
    i2c1_write_reg(LIS2MDL_I2C_ADDR, reg, value);
}

static uint8_t i2c_read_reg(uint8_t reg) {
    uint8_t value = 0;
    i2c1_read_reg(LIS2MDL_I2C_ADDR, reg, &value);
    return value;
}

static void i2c_read_burst(uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c1_read_regs(LIS2MDL_I2C_ADDR, reg, buf, len);
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

bool lis2mdl_init(const lis2mdl_config_t *config) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (lis2mdl_config_t)LIS2MDL_CONFIG_DEFAULT;
    }

    // Check WHO_AM_I
    if (!lis2mdl_is_ready()) {
        return false;
    }

    // Software reset
    i2c_write_reg(LIS2MDL_CFG_REG_A, 0x20);  // SOFT_RST = 1
    system_delay_ms(10);

    // Reboot memory content
    i2c_write_reg(LIS2MDL_CFG_REG_A, 0x40);  // REBOOT = 1
    system_delay_ms(20);

    // Configure CFG_REG_A: ODR, mode, temp compensation
    uint8_t cfg_a = (s_config.odr << 2) | (s_config.mode);
    if (s_config.temp_comp) {
        cfg_a |= 0x80;  // COMP_TEMP_EN
    }
    i2c_write_reg(LIS2MDL_CFG_REG_A, cfg_a);

    // Configure CFG_REG_B: Low-pass filter
    uint8_t cfg_b = 0;
    if (s_config.low_pass_filter) {
        cfg_b |= 0x01;  // LPF
    }
    cfg_b |= 0x02;  // OFF_CANC - enable offset cancellation
    i2c_write_reg(LIS2MDL_CFG_REG_B, cfg_b);

    // Configure CFG_REG_C: BDU (block data update)
    i2c_write_reg(LIS2MDL_CFG_REG_C, 0x10);  // BDU = 1

    return true;
}

bool lis2mdl_is_ready(void) {
    uint8_t who = i2c_read_reg(LIS2MDL_WHO_AM_I);
    return (who == LIS2MDL_WHO_AM_I_VALUE);
}

bool lis2mdl_data_ready(void) {
    uint8_t status = i2c_read_reg(LIS2MDL_STATUS_REG);
    return (status & LIS2MDL_STATUS_ZYXDA) != 0;
}

void lis2mdl_read_raw(lis2mdl_raw_t *data) {
    uint8_t buf[6];
    i2c_read_burst(LIS2MDL_OUTX_L_REG, buf, 6);
    data->x = (int16_t)(buf[1] << 8 | buf[0]);
    data->y = (int16_t)(buf[3] << 8 | buf[2]);
    data->z = (int16_t)(buf[5] << 8 | buf[4]);
}

void lis2mdl_read(lis2mdl_data_t *data) {
    lis2mdl_raw_t raw;
    lis2mdl_read_raw(&raw);
    data->x = raw.x * LIS2MDL_SENSITIVITY;
    data->y = raw.y * LIS2MDL_SENSITIVITY;
    data->z = raw.z * LIS2MDL_SENSITIVITY;
}

void lis2mdl_read_calibrated(lis2mdl_data_t *data, const lis2mdl_offset_t *offset) {
    lis2mdl_read(data);
    data->x -= offset->x;
    data->y -= offset->y;
    data->z -= offset->z;
}

float lis2mdl_read_temp(void) {
    uint8_t buf[2];
    i2c_read_burst(LIS2MDL_TEMP_OUT_L_REG, buf, 2);
    int16_t raw = (int16_t)(buf[1] << 8 | buf[0]);
    // Temperature sensitivity: 8 LSB/°C, offset at 25°C
    return 25.0f + (raw / 8.0f);
}

void lis2mdl_set_offset(int16_t x, int16_t y, int16_t z) {
    i2c_write_reg(LIS2MDL_OFFSET_X_REG_L, x & 0xFF);
    i2c_write_reg(LIS2MDL_OFFSET_X_REG_H, (x >> 8) & 0xFF);
    i2c_write_reg(LIS2MDL_OFFSET_Y_REG_L, y & 0xFF);
    i2c_write_reg(LIS2MDL_OFFSET_Y_REG_H, (y >> 8) & 0xFF);
    i2c_write_reg(LIS2MDL_OFFSET_Z_REG_L, z & 0xFF);
    i2c_write_reg(LIS2MDL_OFFSET_Z_REG_H, (z >> 8) & 0xFF);
}

float lis2mdl_heading(const lis2mdl_data_t *mag) {
    // Simple 2D heading (only valid when level)
    // atan2 returns -π to +π, with 0 pointing along +X axis
    // Adjust so 0 = magnetic north (assuming +X = north)
    float heading = atan2f(mag->y, mag->x);

    // Normalize to 0 to 2π
    if (heading < 0) {
        heading += 2.0f * (float)M_PI;
    }

    return heading;
}

float lis2mdl_heading_tilt_compensated(const lis2mdl_data_t *mag,
                                        float roll, float pitch) {
    // Tilt compensation using rotation matrix
    // Rotate magnetometer readings into horizontal plane

    float cos_roll = cosf(roll);
    float sin_roll = sinf(roll);
    float cos_pitch = cosf(pitch);
    float sin_pitch = sinf(pitch);

    // Compensate magnetometer readings for tilt
    // X_h = X * cos(pitch) + Y * sin(roll) * sin(pitch) + Z * cos(roll) * sin(pitch)
    // Y_h = Y * cos(roll) - Z * sin(roll)
    float mag_x_h = mag->x * cos_pitch +
                    mag->y * sin_roll * sin_pitch +
                    mag->z * cos_roll * sin_pitch;

    float mag_y_h = mag->y * cos_roll - mag->z * sin_roll;

    // Calculate heading from horizontal components
    float heading = atan2f(mag_y_h, mag_x_h);

    // Normalize to 0 to 2π
    if (heading < 0) {
        heading += 2.0f * (float)M_PI;
    }

    return heading;
}

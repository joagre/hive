// BMP388 Barometer Driver Implementation
//
// Reference: Bosch BMP388 Datasheet (BST-BMP388-DS001)
// Reference: Bosch BMP3 API (bmp3.c)

#include "bmp388.h"
#include <math.h>

// ----------------------------------------------------------------------------
// Register Definitions
// ----------------------------------------------------------------------------

#define BMP388_REG_CHIP_ID      0x00
#define BMP388_REG_ERR_REG      0x02
#define BMP388_REG_STATUS       0x03
#define BMP388_REG_DATA_0       0x04    // Pressure XLSB
#define BMP388_REG_DATA_1       0x05    // Pressure LSB
#define BMP388_REG_DATA_2       0x06    // Pressure MSB
#define BMP388_REG_DATA_3       0x07    // Temperature XLSB
#define BMP388_REG_DATA_4       0x08    // Temperature LSB
#define BMP388_REG_DATA_5       0x09    // Temperature MSB
#define BMP388_REG_SENSORTIME_0 0x0C
#define BMP388_REG_SENSORTIME_1 0x0D
#define BMP388_REG_SENSORTIME_2 0x0E
#define BMP388_REG_EVENT        0x10
#define BMP388_REG_INT_STATUS   0x11
#define BMP388_REG_FIFO_LENGTH_0 0x12
#define BMP388_REG_FIFO_LENGTH_1 0x13
#define BMP388_REG_FIFO_DATA    0x14
#define BMP388_REG_FIFO_WTM_0   0x15
#define BMP388_REG_FIFO_WTM_1   0x16
#define BMP388_REG_FIFO_CONFIG_1 0x17
#define BMP388_REG_FIFO_CONFIG_2 0x18
#define BMP388_REG_INT_CTRL     0x19
#define BMP388_REG_IF_CONF      0x1A
#define BMP388_REG_PWR_CTRL     0x1B
#define BMP388_REG_OSR          0x1C
#define BMP388_REG_ODR          0x1D
#define BMP388_REG_CONFIG       0x1F
#define BMP388_REG_CALIB_DATA   0x31    // Start of calibration data (21 bytes)
#define BMP388_REG_CMD          0x7E

// Expected chip ID
#define BMP388_CHIP_ID_VALUE    0x50

// Commands
#define BMP388_CMD_SOFTRESET    0xB6

// Status bits
#define BMP388_STATUS_DRDY_PRESS 0x20
#define BMP388_STATUS_DRDY_TEMP  0x40
#define BMP388_STATUS_CMD_RDY    0x10

// ----------------------------------------------------------------------------
// Calibration Data Structure (from NVM)
// ----------------------------------------------------------------------------

typedef struct {
    // Temperature coefficients
    uint16_t t1;
    uint16_t t2;
    int8_t   t3;

    // Pressure coefficients
    int16_t  p1;
    int16_t  p2;
    int8_t   p3;
    int8_t   p4;
    uint16_t p5;
    uint16_t p6;
    int8_t   p7;
    int8_t   p8;
    int16_t  p9;
    int8_t   p10;
    int8_t   p11;
} bmp388_calib_t;

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;
static uint8_t s_i2c_addr = BMP388_I2C_ADDR_DEFAULT;
static bmp388_config_t s_config;
static bmp388_calib_t s_calib;

// Compensation intermediate value
static float s_t_lin = 0.0f;

// ----------------------------------------------------------------------------
// Low-Level Register Access
// ----------------------------------------------------------------------------

static bool read_reg(uint8_t reg, uint8_t *value) {
    return bmp388_i2c_read(s_i2c_addr, reg, value, 1);
}

static bool write_reg(uint8_t reg, uint8_t value) {
    return bmp388_i2c_write(s_i2c_addr, reg, &value, 1);
}

static bool read_regs(uint8_t reg, uint8_t *data, uint8_t len) {
    return bmp388_i2c_read(s_i2c_addr, reg, data, len);
}

// ----------------------------------------------------------------------------
// Calibration Data
// ----------------------------------------------------------------------------

static bool read_calibration_data(void) {
    uint8_t buf[21];

    if (!read_regs(BMP388_REG_CALIB_DATA, buf, 21)) {
        return false;
    }

    // Parse calibration coefficients (little-endian)
    s_calib.t1 = (uint16_t)(buf[1] << 8) | buf[0];
    s_calib.t2 = (uint16_t)(buf[3] << 8) | buf[2];
    s_calib.t3 = (int8_t)buf[4];

    s_calib.p1 = (int16_t)((buf[6] << 8) | buf[5]);
    s_calib.p2 = (int16_t)((buf[8] << 8) | buf[7]);
    s_calib.p3 = (int8_t)buf[9];
    s_calib.p4 = (int8_t)buf[10];
    s_calib.p5 = (uint16_t)(buf[12] << 8) | buf[11];
    s_calib.p6 = (uint16_t)(buf[14] << 8) | buf[13];
    s_calib.p7 = (int8_t)buf[15];
    s_calib.p8 = (int8_t)buf[16];
    s_calib.p9 = (int16_t)((buf[18] << 8) | buf[17]);
    s_calib.p10 = (int8_t)buf[19];
    s_calib.p11 = (int8_t)buf[20];

    return true;
}

// ----------------------------------------------------------------------------
// Compensation Functions (from Bosch API)
// ----------------------------------------------------------------------------

static float compensate_temperature(uint32_t raw_temp) {
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(raw_temp - (256.0f * s_calib.t1));
    partial_data2 = s_calib.t2 * (1.0f / 1073741824.0f);

    s_t_lin = partial_data1 * partial_data2 +
              partial_data1 * partial_data1 * s_calib.t3 * (1.0f / 281474976710656.0f);

    return s_t_lin;
}

static float compensate_pressure(uint32_t raw_press) {
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    partial_data1 = s_calib.p6 * s_t_lin;
    partial_data2 = s_calib.p7 * (s_t_lin * s_t_lin);
    partial_data3 = s_calib.p8 * (s_t_lin * s_t_lin * s_t_lin);
    partial_out1 = s_calib.p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = s_calib.p2 * s_t_lin;
    partial_data2 = s_calib.p3 * (s_t_lin * s_t_lin);
    partial_data3 = s_calib.p4 * (s_t_lin * s_t_lin * s_t_lin);
    partial_out2 = (float)raw_press *
                   (s_calib.p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = s_calib.p9 + s_calib.p10 * s_t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)raw_press * (float)raw_press * (float)raw_press) * s_calib.p11;

    return partial_out1 + partial_out2 + partial_data4;
}

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

bool bmp388_init(const bmp388_config_t *config) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (bmp388_config_t)BMP388_CONFIG_DEFAULT;
    }

    // Soft reset
    write_reg(BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
    bmp388_delay_ms(10);

    // Verify chip ID
    uint8_t chip_id;
    if (!read_reg(BMP388_REG_CHIP_ID, &chip_id)) {
        // Try alternate address
        s_i2c_addr = BMP388_I2C_ADDR_ALT;
        if (!read_reg(BMP388_REG_CHIP_ID, &chip_id)) {
            return false;
        }
    }

    if (chip_id != BMP388_CHIP_ID_VALUE) {
        return false;
    }

    // Read calibration data
    if (!read_calibration_data()) {
        return false;
    }

    // Configure OSR (oversampling)
    uint8_t osr = (s_config.temp_osr << 3) | s_config.press_osr;
    if (!write_reg(BMP388_REG_OSR, osr)) {
        return false;
    }

    // Configure ODR
    if (!write_reg(BMP388_REG_ODR, s_config.odr)) {
        return false;
    }

    // Configure IIR filter
    uint8_t cfg = s_config.iir_coef << 1;
    if (!write_reg(BMP388_REG_CONFIG, cfg)) {
        return false;
    }

    // Enable pressure and temperature, normal mode
    uint8_t pwr = 0x03 | (0x01 << 4) | (0x01 << 5);  // press_en, temp_en, normal mode
    if (!write_reg(BMP388_REG_PWR_CTRL, pwr)) {
        return false;
    }

    bmp388_delay_ms(10);

    s_initialized = true;
    return true;
}

bool bmp388_is_ready(void) {
    if (!s_initialized) {
        return false;
    }

    uint8_t chip_id;
    if (!read_reg(BMP388_REG_CHIP_ID, &chip_id)) {
        return false;
    }

    return chip_id == BMP388_CHIP_ID_VALUE;
}

bool bmp388_data_ready(void) {
    uint8_t status;
    if (!read_reg(BMP388_REG_STATUS, &status)) {
        return false;
    }

    return (status & (BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP)) ==
           (BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP);
}

bool bmp388_read(bmp388_data_t *data) {
    if (!s_initialized) {
        return false;
    }

    uint8_t buf[6];
    if (!read_regs(BMP388_REG_DATA_0, buf, 6)) {
        return false;
    }

    // Assemble 24-bit raw values (LSB first)
    uint32_t raw_press = (uint32_t)(buf[2] << 16) | (buf[1] << 8) | buf[0];
    uint32_t raw_temp = (uint32_t)(buf[5] << 16) | (buf[4] << 8) | buf[3];

    // Compensate (temperature must be done first)
    data->temperature_c = compensate_temperature(raw_temp);
    data->pressure_pa = compensate_pressure(raw_press);

    return true;
}

bool bmp388_read_pressure(float *pressure_hpa) {
    bmp388_data_t data;
    if (!bmp388_read(&data)) {
        return false;
    }

    *pressure_hpa = data.pressure_pa / 100.0f;
    return true;
}

bool bmp388_read_temperature(float *temp_c) {
    bmp388_data_t data;
    if (!bmp388_read(&data)) {
        return false;
    }

    *temp_c = data.temperature_c;
    return true;
}

float bmp388_pressure_to_altitude(float pressure_pa, float ref_pressure_pa) {
    // Barometric formula: h = 44330 * (1 - (p/p0)^0.1903)
    // Where p0 is reference pressure (sea level or ground level)
    if (ref_pressure_pa <= 0.0f) {
        return 0.0f;
    }

    float ratio = pressure_pa / ref_pressure_pa;
    return 44330.0f * (1.0f - powf(ratio, 0.1903f));
}

bool bmp388_trigger(void) {
    if (!s_initialized) {
        return false;
    }

    // Set forced mode (single measurement)
    uint8_t pwr = 0x01 | (0x01 << 4) | (0x01 << 5);  // press_en, temp_en, forced mode
    return write_reg(BMP388_REG_PWR_CTRL, pwr);
}

void bmp388_reset(void) {
    write_reg(BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
    bmp388_delay_ms(10);
    s_initialized = false;
}

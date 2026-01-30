/*
 * VL53L1X Ultra Lite Driver (C port)
 *
 * Ported from ST/mbed ULD to plain C.
 * Original: https://github.com/VRaktion/mbed-VL53L1X-ULD
 *
 * Copyright (c) 2017, STMicroelectronics - All Rights Reserved
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 * See vl53l1x_uld.h for full license text.
 */

#include "vl53l1x_uld.h"

// ----------------------------------------------------------------------------
// Register addresses
// ----------------------------------------------------------------------------

#define SOFT_RESET                                      0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND    0x0008
#define VL53L1_IDENTIFICATION__MODEL_ID                 0x010F
#define VL53L1_FIRMWARE__SYSTEM_STATUS                  0x00E5
#define GPIO_HV_MUX__CTRL                               0x0030
#define GPIO__TIO_HV_STATUS                             0x0031
#define SYSTEM__INTERRUPT_CLEAR                         0x0086
#define SYSTEM__MODE_START                              0x0087
#define VL53L1_RESULT__RANGE_STATUS                     0x0089
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
#define VL53L1_RESULT__OSC_CALIBRATE_VAL                0x00DE
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD          0x006C
#define PHASECAL_CONFIG__TIMEOUT_MACROP                 0x004B
#define RANGE_CONFIG__VCSEL_PERIOD_A                    0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B                    0x0063
#define RANGE_CONFIG__VALID_PHASE_HIGH                  0x0069
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI               0x005E
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI               0x0061
#define SD_CONFIG__WOI_SD0                              0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0                    0x007A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM              0x001E
#define MM_CONFIG__INNER_OFFSET_MM                      0x0020
#define MM_CONFIG__OUTER_OFFSET_MM                      0x0022

// ----------------------------------------------------------------------------
// Default configuration (91 bytes starting at register 0x2D)
// ----------------------------------------------------------------------------

static const uint8_t VL53L1X_DEFAULT_CONFIGURATION[] = {
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C) */
    0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 */
    0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */
    0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */
    0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */
    0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0x20 = New sample ready */
    0x0b, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4a : not user-modifiable */
    0x0a, /* 0x4b : not user-modifiable */
    0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */
    0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */
    0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */
    0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */
    0x0f, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xf1, /* 0x62 : not user-modifiable */
    0x0d, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma threshold MSB (default 90mm) */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not user-modifiable */
    0xb8, /* 0x69 : not user-modifiable */
    0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x0f, /* 0x78 : not user-modifiable */
    0x0d, /* 0x79 : not user-modifiable */
    0x0e, /* 0x7a : not user-modifiable */
    0x0e, /* 0x7b : not user-modifiable */
    0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */
    0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : ROI center */
    0xff, /* 0x80 : XY ROI (X=Width, Y=Height) */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt */
    0x00  /* 0x87 : start ranging */
};

// Range status lookup table
static const uint8_t status_rtn[24] = {
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255,
    9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12
};

// ----------------------------------------------------------------------------
// Low-level I2C access
// ----------------------------------------------------------------------------

static int8_t wr_byte(vl53l1x_dev_t *dev, uint16_t reg, uint8_t data) {
    return dev->platform->i2c_write(dev->i2c_address, reg, &data, 1);
}

static int8_t rd_byte(vl53l1x_dev_t *dev, uint16_t reg, uint8_t *data) {
    return dev->platform->i2c_read(dev->i2c_address, reg, data, 1);
}

static int8_t wr_word(vl53l1x_dev_t *dev, uint16_t reg, uint16_t data) {
    uint8_t buffer[2];
    buffer[0] = data >> 8;
    buffer[1] = data & 0xFF;
    return dev->platform->i2c_write(dev->i2c_address, reg, buffer, 2);
}

static int8_t rd_word(vl53l1x_dev_t *dev, uint16_t reg, uint16_t *data) {
    uint8_t buffer[2];
    int8_t status = dev->platform->i2c_read(dev->i2c_address, reg, buffer, 2);
    if (status == 0) {
        *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    }
    return status;
}

static int8_t wr_dword(vl53l1x_dev_t *dev, uint16_t reg, uint32_t data) {
    uint8_t buffer[4];
    buffer[0] = (data >> 24) & 0xFF;
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >> 8) & 0xFF;
    buffer[3] = data & 0xFF;
    return dev->platform->i2c_write(dev->i2c_address, reg, buffer, 4);
}

static int8_t rd_multi(vl53l1x_dev_t *dev, uint16_t reg, uint8_t *data, uint16_t len) {
    return dev->platform->i2c_read(dev->i2c_address, reg, data, len);
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

int8_t vl53l1x_init(vl53l1x_dev_t *dev, const vl53l1x_platform_t *platform) {
    dev->platform = platform;
    dev->i2c_address = VL53L1X_DEFAULT_ADDRESS;
    return 0;
}

int8_t vl53l1x_reset(vl53l1x_dev_t *dev) {
    int8_t status;
    status = wr_byte(dev, SOFT_RESET, 0x00);
    dev->platform->delay_ms(1);
    status = wr_byte(dev, SOFT_RESET, 0x01);
    dev->platform->delay_ms(1);
    return status;
}

int8_t vl53l1x_boot_state(vl53l1x_dev_t *dev, uint8_t *state) {
    return rd_byte(dev, VL53L1_FIRMWARE__SYSTEM_STATUS, state);
}

int8_t vl53l1x_get_sensor_id(vl53l1x_dev_t *dev, uint16_t *sensor_id) {
    return rd_word(dev, VL53L1_IDENTIFICATION__MODEL_ID, sensor_id);
}

int8_t vl53l1x_sensor_init(vl53l1x_dev_t *dev) {
    int8_t status = 0;
    uint8_t addr;
    uint8_t tmp;

    // Load default configuration
    for (addr = 0x2D; addr <= 0x87; addr++) {
        status = wr_byte(dev, addr, VL53L1X_DEFAULT_CONFIGURATION[addr - 0x2D]);
        if (status != 0) return status;
    }

    // Post-init: start ranging once to validate
    status = vl53l1x_start_ranging(dev);
    if (status != 0) return status;

    tmp = 0;
    while (tmp == 0) {
        status = vl53l1x_check_data_ready(dev, &tmp);
        if (status != 0) return status;
    }

    status = vl53l1x_clear_interrupt(dev);
    status = vl53l1x_stop_ranging(dev);

    // Configure VHV
    status = wr_byte(dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    status = wr_byte(dev, 0x0B, 0x00);

    return status;
}

int8_t vl53l1x_start_ranging(vl53l1x_dev_t *dev) {
    return wr_byte(dev, SYSTEM__MODE_START, 0x40);
}

int8_t vl53l1x_stop_ranging(vl53l1x_dev_t *dev) {
    return wr_byte(dev, SYSTEM__MODE_START, 0x00);
}

int8_t vl53l1x_check_data_ready(vl53l1x_dev_t *dev, uint8_t *is_ready) {
    int8_t status;
    uint8_t temp;
    uint8_t int_pol;

    // Get interrupt polarity
    status = rd_byte(dev, GPIO_HV_MUX__CTRL, &temp);
    int_pol = (temp & 0x10) ? 0 : 1;

    // Check GPIO status
    status = rd_byte(dev, GPIO__TIO_HV_STATUS, &temp);
    if (status == 0) {
        *is_ready = ((temp & 1) == int_pol) ? 1 : 0;
    }
    return status;
}

int8_t vl53l1x_get_distance(vl53l1x_dev_t *dev, uint16_t *distance) {
    return rd_word(dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, distance);
}

int8_t vl53l1x_get_range_status(vl53l1x_dev_t *dev, uint8_t *status_out) {
    int8_t status;
    uint8_t rg_st;

    *status_out = 255;
    status = rd_byte(dev, VL53L1_RESULT__RANGE_STATUS, &rg_st);
    rg_st = rg_st & 0x1F;
    if (rg_st < 24) {
        *status_out = status_rtn[rg_st];
    }
    return status;
}

int8_t vl53l1x_get_result(vl53l1x_dev_t *dev, vl53l1x_result_t *result) {
    int8_t status;
    uint8_t temp[17];
    uint8_t rg_st;

    status = rd_multi(dev, VL53L1_RESULT__RANGE_STATUS, temp, 17);
    if (status != 0) return status;

    rg_st = temp[0] & 0x1F;
    if (rg_st < 24) {
        rg_st = status_rtn[rg_st];
    }

    result->status = rg_st;
    result->ambient = ((uint16_t)temp[7] << 8 | temp[8]) * 8;
    result->num_spads = temp[3];
    result->signal_per_spad = ((uint16_t)temp[15] << 8 | temp[16]) * 8;
    result->distance = (uint16_t)temp[13] << 8 | temp[14];

    return status;
}

int8_t vl53l1x_clear_interrupt(vl53l1x_dev_t *dev) {
    return wr_byte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
}

int8_t vl53l1x_set_distance_mode(vl53l1x_dev_t *dev, vl53l1x_distance_mode_t mode) {
    int8_t status = 0;

    switch (mode) {
    case VL53L1X_DISTANCE_MODE_SHORT:
        status = wr_byte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
        status = wr_byte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
        status = wr_byte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
        status = wr_byte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
        status = wr_word(dev, SD_CONFIG__WOI_SD0, 0x0705);
        status = wr_word(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
        break;

    case VL53L1X_DISTANCE_MODE_LONG:
        status = wr_byte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
        status = wr_byte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
        status = wr_byte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
        status = wr_byte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
        status = wr_word(dev, SD_CONFIG__WOI_SD0, 0x0F0D);
        status = wr_word(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
        break;

    default:
        status = 1;
        break;
    }

    return status;
}

int8_t vl53l1x_set_timing_budget(vl53l1x_dev_t *dev, vl53l1x_timing_budget_t budget_ms) {
    int8_t status = 0;

    // Default to long mode timing values
    switch (budget_ms) {
    case VL53L1X_TIMING_20MS:
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
        break;
    case VL53L1X_TIMING_33MS:
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
        break;
    case VL53L1X_TIMING_50MS:
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
        break;
    case VL53L1X_TIMING_100MS:
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
        break;
    case VL53L1X_TIMING_200MS:
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
        break;
    case VL53L1X_TIMING_500MS:
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
        status = wr_word(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
        break;
    default:
        status = 1;
        break;
    }

    return status;
}

int8_t vl53l1x_set_inter_measurement(vl53l1x_dev_t *dev, uint32_t period_ms) {
    int8_t status;
    uint16_t clock_pll;

    status = rd_word(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &clock_pll);
    if (status != 0) return status;

    clock_pll = clock_pll & 0x3FF;
    // Calculate period with 1.075 correction factor
    uint32_t period = (uint32_t)((float)clock_pll * (float)period_ms * 1.075f);
    status = wr_dword(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, period);

    return status;
}

int8_t vl53l1x_set_offset(vl53l1x_dev_t *dev, int16_t offset_mm) {
    int8_t status;
    int16_t temp = offset_mm * 4;

    status = wr_word(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)temp);
    status = wr_word(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
    status = wr_word(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);

    return status;
}

int8_t vl53l1x_get_offset(vl53l1x_dev_t *dev, int16_t *offset_mm) {
    int8_t status;
    uint16_t temp;

    status = rd_word(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, &temp);
    temp = temp << 3;
    temp = temp >> 5;
    *offset_mm = (int16_t)temp;

    return status;
}

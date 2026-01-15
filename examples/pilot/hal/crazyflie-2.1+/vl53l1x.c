// VL53L1x Time-of-Flight Distance Sensor Driver Implementation
//
// Based on ST VL53L1X Ultra Lite Driver (STSW-IMG009)
// Reference: ST VL53L1X API User Manual (UM2356)

#include "vl53l1x.h"

// ----------------------------------------------------------------------------
// Register Definitions (16-bit register addresses)
// ----------------------------------------------------------------------------

#define VL53L1X_SOFT_RESET                      0x0000
#define VL53L1X_I2C_SLAVE_DEVICE_ADDRESS        0x0001
#define VL53L1X_MODEL_ID                        0x010F
#define VL53L1X_MODULE_TYPE                     0x0110
#define VL53L1X_FIRMWARE_SYSTEM_STATUS          0x00E5
#define VL53L1X_GPIO_HV_MUX_CTRL                0x0030
#define VL53L1X_GPIO_TIO_HV_STATUS              0x0031
#define VL53L1X_SYSTEM_MODE_START               0x0087
#define VL53L1X_RESULT_RANGE_STATUS             0x0089
#define VL53L1X_RESULT_DSS_ACTUAL_EFFECTIVE_SPADS 0x008C
#define VL53L1X_RESULT_AMBIENT_COUNT_RATE_MCPS  0x0090
#define VL53L1X_RESULT_SIGNAL_COUNT_RATE_MCPS   0x0096
#define VL53L1X_RESULT_SIGMA                    0x009C
#define VL53L1X_RESULT_FINAL_RANGE_MM           0x0096
#define VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A     0x0060
#define VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A   0x005E
#define VL53L1X_SYSTEM_INTERRUPT_CLEAR          0x0086
#define VL53L1X_SYSTEM_THRESH_HIGH              0x0072
#define VL53L1X_SYSTEM_THRESH_LOW               0x0074

// Expected model ID
#define VL53L1X_MODEL_ID_VALUE  0xEACC

// ----------------------------------------------------------------------------
// Default Configuration (from ST ULD)
// ----------------------------------------------------------------------------

// Configuration blob from ST ULD (written to sensor at init)
static const uint8_t VL53L1X_DEFAULT_CONFIG[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00, 0x00, 0x02, 0xc7, 0xff,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40
};
#define VL53L1X_DEFAULT_CONFIG_SIZE  91

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;
static uint8_t s_i2c_addr = VL53L1X_I2C_ADDR_DEFAULT;
static vl53l1x_config_t s_config;

// ----------------------------------------------------------------------------
// Low-Level Register Access
// ----------------------------------------------------------------------------

static bool read_reg8(uint16_t reg, uint8_t *value) {
    return vl53l1x_i2c_read(s_i2c_addr, reg, value, 1);
}

static bool write_reg8(uint16_t reg, uint8_t value) {
    return vl53l1x_i2c_write(s_i2c_addr, reg, &value, 1);
}

static bool read_reg16(uint16_t reg, uint16_t *value) {
    uint8_t buf[2];
    if (!vl53l1x_i2c_read(s_i2c_addr, reg, buf, 2)) {
        return false;
    }
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

static bool write_reg16(uint16_t reg, uint16_t value) {
    uint8_t buf[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return vl53l1x_i2c_write(s_i2c_addr, reg, buf, 2);
}

static bool write_reg32(uint16_t reg, uint32_t value) {
    uint8_t buf[4] = {
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };
    return vl53l1x_i2c_write(s_i2c_addr, reg, buf, 4);
}

// ----------------------------------------------------------------------------
// Internal Functions
// ----------------------------------------------------------------------------

static bool wait_for_boot(void) {
    uint8_t status = 0;
    int timeout = 100;  // 1 second timeout

    while (timeout-- > 0) {
        if (read_reg8(VL53L1X_FIRMWARE_SYSTEM_STATUS, &status)) {
            if (status & 0x01) {
                return true;  // Boot complete
            }
        }
        vl53l1x_delay_ms(10);
    }

    return false;  // Timeout
}

static bool sensor_init(void) {
    // Write default configuration
    for (uint8_t i = 0; i < VL53L1X_DEFAULT_CONFIG_SIZE; i++) {
        if (!write_reg8(0x002D + i, VL53L1X_DEFAULT_CONFIG[i])) {
            return false;
        }
    }

    // Start VHV calibration
    if (!write_reg8(VL53L1X_SYSTEM_MODE_START, 0x40)) {
        return false;
    }

    // Wait for calibration to complete
    uint8_t status = 0;
    int timeout = 100;
    while (timeout-- > 0) {
        if (!read_reg8(VL53L1X_GPIO_TIO_HV_STATUS, &status)) {
            return false;
        }
        if ((status & 0x01) == 0) {
            break;
        }
        vl53l1x_delay_ms(10);
    }

    if (timeout <= 0) {
        return false;
    }

    // Clear interrupt
    if (!write_reg8(VL53L1X_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    // Stop ranging
    if (!write_reg8(VL53L1X_SYSTEM_MODE_START, 0x00)) {
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

bool vl53l1x_init(const vl53l1x_config_t *config) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (vl53l1x_config_t)VL53L1X_CONFIG_DEFAULT;
    }

    // Software reset
    write_reg8(VL53L1X_SOFT_RESET, 0x00);
    vl53l1x_delay_ms(1);
    write_reg8(VL53L1X_SOFT_RESET, 0x01);
    vl53l1x_delay_ms(1);

    // Wait for boot
    if (!wait_for_boot()) {
        return false;
    }

    // Verify model ID
    uint16_t model_id;
    if (!read_reg16(VL53L1X_MODEL_ID, &model_id)) {
        return false;
    }
    if (model_id != VL53L1X_MODEL_ID_VALUE) {
        return false;
    }

    // Initialize sensor
    if (!sensor_init()) {
        return false;
    }

    // Apply configuration
    if (!vl53l1x_set_distance_mode(s_config.distance_mode)) {
        return false;
    }
    if (!vl53l1x_set_timing_budget(s_config.timing_budget_ms)) {
        return false;
    }

    // Set inter-measurement period
    write_reg32(0x006C, (uint32_t)(s_config.inter_measurement_ms * 1.075f));

    s_initialized = true;
    return true;
}

bool vl53l1x_is_ready(void) {
    if (!s_initialized) {
        return false;
    }

    uint16_t model_id;
    if (!read_reg16(VL53L1X_MODEL_ID, &model_id)) {
        return false;
    }
    return model_id == VL53L1X_MODEL_ID_VALUE;
}

bool vl53l1x_start_ranging(void) {
    if (!s_initialized) {
        return false;
    }
    return write_reg8(VL53L1X_SYSTEM_MODE_START, 0x40);  // Continuous mode
}

bool vl53l1x_stop_ranging(void) {
    if (!s_initialized) {
        return false;
    }
    return write_reg8(VL53L1X_SYSTEM_MODE_START, 0x00);
}

bool vl53l1x_data_ready(void) {
    uint8_t status;
    if (!read_reg8(VL53L1X_GPIO_TIO_HV_STATUS, &status)) {
        return false;
    }
    return (status & 0x01) == 0;
}

uint16_t vl53l1x_read_distance(void) {
    vl53l1x_result_t result;
    if (!vl53l1x_read_result(&result)) {
        return 0;
    }
    if (result.status != VL53L1X_RANGE_VALID) {
        return 0;
    }
    return result.distance_mm;
}

bool vl53l1x_read_result(vl53l1x_result_t *result) {
    if (!s_initialized) {
        return false;
    }

    // Wait for data
    int timeout = 100;
    while (!vl53l1x_data_ready() && timeout-- > 0) {
        vl53l1x_delay_ms(1);
    }
    if (timeout <= 0) {
        return false;
    }

    // Read result registers
    uint8_t buf[17];
    if (!vl53l1x_i2c_read(s_i2c_addr, VL53L1X_RESULT_RANGE_STATUS, buf, 17)) {
        return false;
    }

    // Parse results
    uint8_t range_status = buf[0] & 0x1F;
    result->distance_mm = ((uint16_t)buf[13] << 8) | buf[14];
    result->ambient_rate = ((uint16_t)buf[7] << 8) | buf[8];
    result->signal_rate = ((uint16_t)buf[15] << 8) | buf[16];
    result->sigma_mm = ((uint16_t)buf[11] << 8) | buf[12];

    // Map status
    switch (range_status) {
        case 9:
            result->status = VL53L1X_RANGE_VALID;
            break;
        case 6:
            result->status = VL53L1X_RANGE_SIGMA_FAIL;
            break;
        case 4:
            result->status = VL53L1X_RANGE_SIGNAL_FAIL;
            break;
        case 8:
            result->status = VL53L1X_RANGE_OUT_OF_BOUNDS_FAIL;
            break;
        case 5:
            result->status = VL53L1X_RANGE_HARDWARE_FAIL;
            break;
        case 7:
            result->status = VL53L1X_RANGE_WRAP_TARGET_FAIL;
            break;
        default:
            result->status = VL53L1X_RANGE_NO_TARGET;
            break;
    }

    // Clear interrupt
    vl53l1x_clear_interrupt();

    return true;
}

void vl53l1x_clear_interrupt(void) {
    write_reg8(VL53L1X_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

uint16_t vl53l1x_measure_single(void) {
    if (!s_initialized) {
        return 0;
    }

    // Start single measurement
    write_reg8(VL53L1X_SYSTEM_MODE_START, 0x10);

    // Wait for result
    vl53l1x_result_t result;
    if (!vl53l1x_read_result(&result)) {
        return 0;
    }

    return (result.status == VL53L1X_RANGE_VALID) ? result.distance_mm : 0;
}

bool vl53l1x_set_distance_mode(vl53l1x_dist_mode_t mode) {
    uint8_t timing_a, timing_b;
    uint16_t period_a, period_b;

    if (mode == VL53L1X_DIST_SHORT) {
        // Short distance mode
        timing_a = 0x07;
        timing_b = 0x05;
        period_a = 0x0006;
        period_b = 0x0006;
    } else {
        // Long distance mode
        timing_a = 0x0F;
        timing_b = 0x0D;
        period_a = 0x000F;
        period_b = 0x000F;
    }

    if (!write_reg8(0x0060, timing_a)) return false;
    if (!write_reg8(0x0063, timing_b)) return false;
    if (!write_reg16(0x0069, period_a)) return false;
    if (!write_reg16(0x0071, period_b)) return false;

    s_config.distance_mode = mode;
    return true;
}

bool vl53l1x_set_timing_budget(vl53l1x_timing_t timing_ms) {
    // Timing budget configuration (simplified from ST ULD)
    uint16_t macro_period;
    uint32_t timeout;

    switch (timing_ms) {
        case VL53L1X_TIMING_15MS:
            macro_period = 0x001D;
            timeout = 0x0027;
            break;
        case VL53L1X_TIMING_20MS:
            macro_period = 0x0051;
            timeout = 0x006E;
            break;
        case VL53L1X_TIMING_33MS:
            macro_period = 0x00D6;
            timeout = 0x01AE;
            break;
        case VL53L1X_TIMING_50MS:
            macro_period = 0x01AE;
            timeout = 0x02E1;
            break;
        case VL53L1X_TIMING_100MS:
            macro_period = 0x02E1;
            timeout = 0x0591;
            break;
        case VL53L1X_TIMING_200MS:
            macro_period = 0x03E1;
            timeout = 0x0B31;
            break;
        case VL53L1X_TIMING_500MS:
            macro_period = 0x0591;
            timeout = 0x1C31;
            break;
        default:
            return false;
    }

    if (!write_reg16(VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A, macro_period)) return false;
    if (!write_reg16(0x0061, (uint16_t)timeout)) return false;

    s_config.timing_budget_ms = timing_ms;
    return true;
}

uint16_t vl53l1x_get_model_id(void) {
    uint16_t model_id;
    if (!read_reg16(VL53L1X_MODEL_ID, &model_id)) {
        return 0;
    }
    return model_id;
}

void vl53l1x_reset(void) {
    write_reg8(VL53L1X_SOFT_RESET, 0x00);
    vl53l1x_delay_ms(1);
    write_reg8(VL53L1X_SOFT_RESET, 0x01);
    vl53l1x_delay_ms(1);
    s_initialized = false;
}

// BMI088 IMU Driver Implementation
//
// Reference: Bosch BMI088 Datasheet (BST-BMI088-DS001)
// Reference: Crazyflie firmware (sensors_bmi088_bmp3xx.c)

#include "bmi088.h"
#include <math.h>

// ----------------------------------------------------------------------------
// Register Definitions
// ----------------------------------------------------------------------------

// Accelerometer registers
#define BMI088_ACC_CHIP_ID 0x00
#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACC_STATUS 0x03
#define BMI088_ACC_X_LSB 0x12
#define BMI088_ACC_X_MSB 0x13
#define BMI088_ACC_Y_LSB 0x14
#define BMI088_ACC_Y_MSB 0x15
#define BMI088_ACC_Z_LSB 0x16
#define BMI088_ACC_Z_MSB 0x17
#define BMI088_ACC_SENSORTIME_0 0x18
#define BMI088_ACC_SENSORTIME_1 0x19
#define BMI088_ACC_SENSORTIME_2 0x1A
#define BMI088_ACC_INT_STAT_1 0x1D
#define BMI088_ACC_TEMP_MSB 0x22
#define BMI088_ACC_TEMP_LSB 0x23
#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_INT1_IO_CONF 0x53
#define BMI088_ACC_INT2_IO_CONF 0x54
#define BMI088_ACC_INT1_INT2_MAP 0x58
#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_SOFTRESET 0x7E

// Gyroscope registers
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_X_LSB 0x02
#define BMI088_GYRO_X_MSB 0x03
#define BMI088_GYRO_Y_LSB 0x04
#define BMI088_GYRO_Y_MSB 0x05
#define BMI088_GYRO_Z_LSB 0x06
#define BMI088_GYRO_Z_MSB 0x07
#define BMI088_GYRO_INT_STAT_1 0x0A
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_INT_CTRL 0x15
#define BMI088_GYRO_INT3_INT4_IO 0x16
#define BMI088_GYRO_INT3_INT4_MAP 0x18
#define BMI088_GYRO_SELF_TEST 0x3C

// Expected chip IDs
#define BMI088_ACC_CHIP_ID_VALUE 0x1E
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

// Commands
#define BMI088_ACC_SOFTRESET_CMD 0xB6
#define BMI088_GYRO_SOFTRESET_CMD 0xB6

// SPI read/write flags
#define BMI088_SPI_READ 0x80
#define BMI088_SPI_WRITE 0x00

// Constants
#define GRAVITY_MSS 9.80665f
#define DEG_TO_RAD 0.017453292519943295f

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;
static bmi088_config_t s_config;

// Scale factors (set during init based on config)
static float s_acc_scale = 0.0f;  // LSB to m/s^2
static float s_gyro_scale = 0.0f; // LSB to rad/s

// ----------------------------------------------------------------------------
// Low-Level SPI Functions
// ----------------------------------------------------------------------------

static uint8_t acc_read_reg(uint8_t reg) {
    bmi088_acc_cs_low();
    bmi088_spi_transfer(reg | BMI088_SPI_READ);
    bmi088_spi_transfer(0x00); // Dummy byte for accelerometer
    uint8_t value = bmi088_spi_transfer(0x00);
    bmi088_acc_cs_high();
    bmi088_delay_us(2);
    return value;
}

static void acc_write_reg(uint8_t reg, uint8_t value) {
    bmi088_acc_cs_low();
    bmi088_spi_transfer(reg | BMI088_SPI_WRITE);
    bmi088_spi_transfer(value);
    bmi088_acc_cs_high();
    bmi088_delay_us(2);
}

static void acc_read_burst(uint8_t reg, uint8_t *data, uint8_t len) {
    bmi088_acc_cs_low();
    bmi088_spi_transfer(reg | BMI088_SPI_READ);
    bmi088_spi_transfer(0x00); // Dummy byte
    for (uint8_t i = 0; i < len; i++) {
        data[i] = bmi088_spi_transfer(0x00);
    }
    bmi088_acc_cs_high();
    bmi088_delay_us(2);
}

static uint8_t gyro_read_reg(uint8_t reg) {
    bmi088_gyro_cs_low();
    bmi088_spi_transfer(reg | BMI088_SPI_READ);
    uint8_t value = bmi088_spi_transfer(0x00);
    bmi088_gyro_cs_high();
    bmi088_delay_us(2);
    return value;
}

static void gyro_write_reg(uint8_t reg, uint8_t value) {
    bmi088_gyro_cs_low();
    bmi088_spi_transfer(reg | BMI088_SPI_WRITE);
    bmi088_spi_transfer(value);
    bmi088_gyro_cs_high();
    bmi088_delay_us(2);
}

static void gyro_read_burst(uint8_t reg, uint8_t *data, uint8_t len) {
    bmi088_gyro_cs_low();
    bmi088_spi_transfer(reg | BMI088_SPI_READ);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = bmi088_spi_transfer(0x00);
    }
    bmi088_gyro_cs_high();
    bmi088_delay_us(2);
}

// ----------------------------------------------------------------------------
// Scale Factor Calculation
// ----------------------------------------------------------------------------

static void calculate_scale_factors(void) {
    // Accelerometer scale: LSB to m/s^2
    // Sensitivity from datasheet (LSB/g at each range)
    switch (s_config.acc_range) {
    case BMI088_ACC_RANGE_3G:
        s_acc_scale = (3.0f * 2.0f * GRAVITY_MSS) / 65536.0f;
        break;
    case BMI088_ACC_RANGE_6G:
        s_acc_scale = (6.0f * 2.0f * GRAVITY_MSS) / 65536.0f;
        break;
    case BMI088_ACC_RANGE_12G:
        s_acc_scale = (12.0f * 2.0f * GRAVITY_MSS) / 65536.0f;
        break;
    case BMI088_ACC_RANGE_24G:
        s_acc_scale = (24.0f * 2.0f * GRAVITY_MSS) / 65536.0f;
        break;
    }

    // Gyroscope scale: LSB to rad/s
    // Sensitivity from datasheet
    switch (s_config.gyro_range) {
    case BMI088_GYRO_RANGE_125DPS:
        s_gyro_scale = (125.0f * 2.0f * DEG_TO_RAD) / 65536.0f;
        break;
    case BMI088_GYRO_RANGE_250DPS:
        s_gyro_scale = (250.0f * 2.0f * DEG_TO_RAD) / 65536.0f;
        break;
    case BMI088_GYRO_RANGE_500DPS:
        s_gyro_scale = (500.0f * 2.0f * DEG_TO_RAD) / 65536.0f;
        break;
    case BMI088_GYRO_RANGE_1000DPS:
        s_gyro_scale = (1000.0f * 2.0f * DEG_TO_RAD) / 65536.0f;
        break;
    case BMI088_GYRO_RANGE_2000DPS:
        s_gyro_scale = (2000.0f * 2.0f * DEG_TO_RAD) / 65536.0f;
        break;
    }
}

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

bool bmi088_init(const bmi088_config_t *config) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (bmi088_config_t)BMI088_CONFIG_DEFAULT;
    }

    calculate_scale_factors();

    // -------------------------------------------------------------------------
    // Accelerometer Initialization
    // -------------------------------------------------------------------------

    // Perform dummy read to switch accelerometer to SPI mode
    acc_read_reg(BMI088_ACC_CHIP_ID);
    bmi088_delay_ms(1);

    // Soft reset accelerometer
    acc_write_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_CMD);
    bmi088_delay_ms(50);

    // Dummy read again after reset
    acc_read_reg(BMI088_ACC_CHIP_ID);
    bmi088_delay_ms(1);

    // Verify accelerometer chip ID
    uint8_t acc_id = acc_read_reg(BMI088_ACC_CHIP_ID);
    if (acc_id != BMI088_ACC_CHIP_ID_VALUE) {
        return false;
    }

    // Configure accelerometer
    // PWR_CONF: disable suspend mode (active mode)
    acc_write_reg(BMI088_ACC_PWR_CONF, 0x00);
    bmi088_delay_ms(1);

    // PWR_CTRL: enable accelerometer
    acc_write_reg(BMI088_ACC_PWR_CTRL, 0x04);
    bmi088_delay_ms(50);

    // ACC_CONF: set ODR and bandwidth
    // BWP = normal (bits 7:4), ODR (bits 3:0)
    acc_write_reg(BMI088_ACC_CONF, (0x0A << 4) | s_config.acc_odr);
    bmi088_delay_us(2);

    // ACC_RANGE: set measurement range
    acc_write_reg(BMI088_ACC_RANGE, s_config.acc_range);
    bmi088_delay_us(2);

    // -------------------------------------------------------------------------
    // Gyroscope Initialization
    // -------------------------------------------------------------------------

    // Soft reset gyroscope
    gyro_write_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_CMD);
    bmi088_delay_ms(50);

    // Verify gyroscope chip ID
    uint8_t gyro_id = gyro_read_reg(BMI088_GYRO_CHIP_ID);
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        return false;
    }

    // Configure gyroscope
    // GYRO_RANGE: set measurement range
    gyro_write_reg(BMI088_GYRO_RANGE, s_config.gyro_range);
    bmi088_delay_us(2);

    // GYRO_BANDWIDTH: set ODR and bandwidth
    gyro_write_reg(BMI088_GYRO_BANDWIDTH, s_config.gyro_odr);
    bmi088_delay_us(2);

    // GYRO_LPM1: normal power mode
    gyro_write_reg(BMI088_GYRO_LPM1, 0x00);
    bmi088_delay_ms(1);

    s_initialized = true;
    return true;
}

bool bmi088_is_ready(void) {
    if (!s_initialized) {
        return false;
    }

    // Check both chip IDs
    uint8_t acc_id = acc_read_reg(BMI088_ACC_CHIP_ID);
    uint8_t gyro_id = gyro_read_reg(BMI088_GYRO_CHIP_ID);

    return (acc_id == BMI088_ACC_CHIP_ID_VALUE) &&
           (gyro_id == BMI088_GYRO_CHIP_ID_VALUE);
}

bool bmi088_read_accel_raw(bmi088_raw_t *data) {
    if (!s_initialized) {
        return false;
    }

    uint8_t buf[6];
    acc_read_burst(BMI088_ACC_X_LSB, buf, 6);

    data->x = (int16_t)((buf[1] << 8) | buf[0]);
    data->y = (int16_t)((buf[3] << 8) | buf[2]);
    data->z = (int16_t)((buf[5] << 8) | buf[4]);

    return true;
}

bool bmi088_read_gyro_raw(bmi088_raw_t *data) {
    if (!s_initialized) {
        return false;
    }

    uint8_t buf[6];
    gyro_read_burst(BMI088_GYRO_X_LSB, buf, 6);

    data->x = (int16_t)((buf[1] << 8) | buf[0]);
    data->y = (int16_t)((buf[3] << 8) | buf[2]);
    data->z = (int16_t)((buf[5] << 8) | buf[4]);

    return true;
}

bool bmi088_read_accel(bmi088_data_t *data) {
    bmi088_raw_t raw;
    if (!bmi088_read_accel_raw(&raw)) {
        return false;
    }

    data->x = raw.x * s_acc_scale;
    data->y = raw.y * s_acc_scale;
    data->z = raw.z * s_acc_scale;

    return true;
}

bool bmi088_read_gyro(bmi088_data_t *data) {
    bmi088_raw_t raw;
    if (!bmi088_read_gyro_raw(&raw)) {
        return false;
    }

    data->x = raw.x * s_gyro_scale;
    data->y = raw.y * s_gyro_scale;
    data->z = raw.z * s_gyro_scale;

    return true;
}

bool bmi088_read_all(bmi088_data_t *accel, bmi088_data_t *gyro) {
    return bmi088_read_accel(accel) && bmi088_read_gyro(gyro);
}

bool bmi088_read_temp(float *temp_c) {
    if (!s_initialized) {
        return false;
    }

    uint8_t buf[2];
    acc_read_burst(BMI088_ACC_TEMP_MSB, buf, 2);

    // Temperature is 11-bit signed, MSB first
    int16_t raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    // Sign extend from 11 bits
    if (raw > 1023) {
        raw -= 2048;
    }

    // Convert to Celsius: 0 LSB = 23 degC, 1 LSB = 0.125 degC
    *temp_c = (raw * 0.125f) + 23.0f;

    return true;
}

bool bmi088_self_test(void) {
    // TODO: Implement self-test routine
    // For now, just verify chip IDs
    return bmi088_is_ready();
}

void bmi088_reset(void) {
    acc_write_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_CMD);
    gyro_write_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_CMD);
    bmi088_delay_ms(50);
    s_initialized = false;
}

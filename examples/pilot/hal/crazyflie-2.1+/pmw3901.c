// PMW3901 Optical Flow Sensor Driver Implementation
//
// Reference: PMW3901MB-TXQT Datasheet
// Reference: Bitcraze PMW3901 driver (pmw3901.c)

#include "pmw3901.h"

// ----------------------------------------------------------------------------
// Register Definitions
// ----------------------------------------------------------------------------

#define REG_PRODUCT_ID      0x00
#define REG_REVISION_ID     0x01
#define REG_MOTION          0x02
#define REG_DELTA_X_L       0x03
#define REG_DELTA_X_H       0x04
#define REG_DELTA_Y_L       0x05
#define REG_DELTA_Y_H       0x06
#define REG_SQUAL           0x07
#define REG_RAW_DATA_SUM    0x08
#define REG_MAXIMUM_RAW     0x09
#define REG_MINIMUM_RAW     0x0A
#define REG_SHUTTER_LOWER   0x0B
#define REG_SHUTTER_UPPER   0x0C
#define REG_OBSERVATION     0x15
#define REG_MOTION_BURST    0x16
#define REG_POWER_UP_RESET  0x3A
#define REG_SHUTDOWN        0x3B
#define REG_RAW_DATA_GRAB   0x58
#define REG_RAW_DATA_GRAB_STATUS 0x59
#define REG_RAWDATA_OUT     0x5A
#define REG_INVERSE_PRODUCT_ID 0x5F

// Expected IDs
#define PMW3901_PRODUCT_ID  0x49
#define PMW3901_INVERSE_ID  0xB6

// SPI read/write flags
#define SPI_READ   0x00
#define SPI_WRITE  0x80

// Motion register bits
#define MOTION_MOT          0x80
#define MOTION_OVF          0x10

// ----------------------------------------------------------------------------
// Static State
// ----------------------------------------------------------------------------

static bool s_initialized = false;

// ----------------------------------------------------------------------------
// Low-Level Register Access
// ----------------------------------------------------------------------------

static uint8_t read_reg(uint8_t reg) {
    pmw3901_cs_low();
    pmw3901_spi_transfer(reg & 0x7F);  // MSB = 0 for read
    pmw3901_delay_us(50);              // Wait time between address and data
    uint8_t value = pmw3901_spi_transfer(0x00);
    pmw3901_cs_high();
    pmw3901_delay_us(200);             // Minimum time between transactions
    return value;
}

static void write_reg(uint8_t reg, uint8_t value) {
    pmw3901_cs_low();
    pmw3901_spi_transfer(reg | 0x80);  // MSB = 1 for write
    pmw3901_spi_transfer(value);
    pmw3901_cs_high();
    pmw3901_delay_us(200);             // Minimum time between transactions
}

// Burst read for motion data
static void read_motion_burst(uint8_t *buf, uint8_t len) {
    pmw3901_cs_low();
    pmw3901_spi_transfer(REG_MOTION_BURST);
    pmw3901_delay_us(50);

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = pmw3901_spi_transfer(0x00);
    }

    pmw3901_cs_high();
    pmw3901_delay_us(500);  // Longer delay after burst read
}

// ----------------------------------------------------------------------------
// Initialization Sequences (from Bitcraze driver)
// ----------------------------------------------------------------------------

static void init_registers(void) {
    // Performance optimization registers (from Bitcraze)
    // These are undocumented magic values from PixArt

    write_reg(0x7F, 0x00);
    write_reg(0x61, 0xAD);
    write_reg(0x7F, 0x03);
    write_reg(0x40, 0x00);
    write_reg(0x7F, 0x05);
    write_reg(0x41, 0xB3);
    write_reg(0x43, 0xF1);
    write_reg(0x45, 0x14);
    write_reg(0x5B, 0x32);
    write_reg(0x5F, 0x34);
    write_reg(0x7B, 0x08);
    write_reg(0x7F, 0x06);
    write_reg(0x44, 0x1B);
    write_reg(0x40, 0xBF);
    write_reg(0x4E, 0x3F);
    write_reg(0x7F, 0x08);
    write_reg(0x65, 0x20);
    write_reg(0x6A, 0x18);
    write_reg(0x7F, 0x09);
    write_reg(0x4F, 0xAF);
    write_reg(0x5F, 0x40);
    write_reg(0x48, 0x80);
    write_reg(0x49, 0x80);
    write_reg(0x57, 0x77);
    write_reg(0x60, 0x78);
    write_reg(0x61, 0x78);
    write_reg(0x62, 0x08);
    write_reg(0x63, 0x50);
    write_reg(0x7F, 0x0A);
    write_reg(0x45, 0x60);
    write_reg(0x7F, 0x00);
    write_reg(0x4D, 0x11);
    write_reg(0x55, 0x80);
    write_reg(0x74, 0x1F);
    write_reg(0x75, 0x1F);
    write_reg(0x4A, 0x78);
    write_reg(0x4B, 0x78);
    write_reg(0x44, 0x08);
    write_reg(0x45, 0x50);
    write_reg(0x64, 0xFF);
    write_reg(0x65, 0x1F);
    write_reg(0x7F, 0x14);
    write_reg(0x65, 0x60);
    write_reg(0x66, 0x08);
    write_reg(0x63, 0x78);
    write_reg(0x7F, 0x15);
    write_reg(0x48, 0x58);
    write_reg(0x7F, 0x07);
    write_reg(0x41, 0x0D);
    write_reg(0x43, 0x14);
    write_reg(0x4B, 0x0E);
    write_reg(0x45, 0x0F);
    write_reg(0x44, 0x42);
    write_reg(0x4C, 0x80);
    write_reg(0x7F, 0x10);
    write_reg(0x5B, 0x02);
    write_reg(0x7F, 0x07);
    write_reg(0x40, 0x41);
    write_reg(0x70, 0x00);

    pmw3901_delay_ms(10);

    write_reg(0x32, 0x44);
    write_reg(0x7F, 0x07);
    write_reg(0x40, 0x40);
    write_reg(0x7F, 0x06);
    write_reg(0x62, 0xF0);
    write_reg(0x63, 0x00);
    write_reg(0x7F, 0x0D);
    write_reg(0x48, 0xC0);
    write_reg(0x6F, 0xD5);
    write_reg(0x7F, 0x00);
    write_reg(0x5B, 0xA0);
    write_reg(0x4E, 0xA8);
    write_reg(0x5A, 0x50);
    write_reg(0x40, 0x80);
}

// ----------------------------------------------------------------------------
// Public API Implementation
// ----------------------------------------------------------------------------

bool pmw3901_init(void) {
    // Power-up reset
    write_reg(REG_POWER_UP_RESET, 0x5A);
    pmw3901_delay_ms(50);

    // Read and discard motion registers (clears delta counters)
    read_reg(REG_MOTION);
    read_reg(REG_DELTA_X_L);
    read_reg(REG_DELTA_X_H);
    read_reg(REG_DELTA_Y_L);
    read_reg(REG_DELTA_Y_H);

    // Verify product ID
    uint8_t product_id = read_reg(REG_PRODUCT_ID);
    uint8_t inverse_id = read_reg(REG_INVERSE_PRODUCT_ID);

    if (product_id != PMW3901_PRODUCT_ID || inverse_id != PMW3901_INVERSE_ID) {
        return false;
    }

    // Initialize performance registers
    init_registers();

    s_initialized = true;
    return true;
}

bool pmw3901_is_ready(void) {
    if (!s_initialized) {
        return false;
    }

    uint8_t product_id = read_reg(REG_PRODUCT_ID);
    return product_id == PMW3901_PRODUCT_ID;
}

bool pmw3901_read_motion(pmw3901_motion_t *data) {
    if (!s_initialized) {
        return false;
    }

    // Motion burst read: Motion, Obs, Delta_X_L, Delta_X_H, Delta_Y_L, Delta_Y_H,
    //                    SQUAL, RawData_Sum, Maximum_RawData, Minimum_RawData,
    //                    Shutter_Upper, Shutter_Lower
    uint8_t buf[12];
    read_motion_burst(buf, 12);

    // Parse motion data
    data->motion = (buf[0] & MOTION_MOT) != 0;
    data->delta_x = (int16_t)((buf[3] << 8) | buf[2]);
    data->delta_y = (int16_t)((buf[5] << 8) | buf[4]);
    data->squal = buf[6];
    data->shutter = (uint16_t)((buf[10] << 8) | buf[11]);

    return true;
}

bool pmw3901_read_delta(int16_t *delta_x, int16_t *delta_y) {
    pmw3901_motion_t motion;
    if (!pmw3901_read_motion(&motion)) {
        return false;
    }

    if (delta_x) *delta_x = motion.delta_x;
    if (delta_y) *delta_y = motion.delta_y;

    return true;
}

void pmw3901_set_frame_capture(bool enable) {
    if (enable) {
        write_reg(REG_RAW_DATA_GRAB, 0xFF);
    } else {
        write_reg(REG_RAW_DATA_GRAB, 0x00);
    }
}

bool pmw3901_read_frame(uint8_t *buf) {
    if (!s_initialized) {
        return false;
    }

    // Wait for frame to be ready
    uint8_t status;
    int timeout = 100;
    do {
        status = read_reg(REG_RAW_DATA_GRAB_STATUS);
        pmw3901_delay_ms(1);
    } while ((status & 0xC0) != 0xC0 && --timeout > 0);

    if (timeout == 0) {
        return false;
    }

    // Read 35x35 = 1225 bytes
    pmw3901_cs_low();
    pmw3901_spi_transfer(REG_RAWDATA_OUT);
    pmw3901_delay_us(50);

    for (int i = 0; i < 1225; i++) {
        buf[i] = pmw3901_spi_transfer(0x00);
    }

    pmw3901_cs_high();

    return true;
}

bool pmw3901_get_accumulated(int16_t *delta_x, int16_t *delta_y) {
    if (!s_initialized) {
        return false;
    }

    // Read without clearing (use standard register reads)
    uint8_t xl = read_reg(REG_DELTA_X_L);
    uint8_t xh = read_reg(REG_DELTA_X_H);
    uint8_t yl = read_reg(REG_DELTA_Y_L);
    uint8_t yh = read_reg(REG_DELTA_Y_H);

    if (delta_x) *delta_x = (int16_t)((xh << 8) | xl);
    if (delta_y) *delta_y = (int16_t)((yh << 8) | yl);

    return true;
}

void pmw3901_reset(void) {
    write_reg(REG_POWER_UP_RESET, 0x5A);
    pmw3901_delay_ms(50);
    s_initialized = false;
}

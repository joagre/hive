/*
 * PMW3901 Optical Flow Sensor Driver (C port)
 *
 * Ported from Bitcraze Arduino driver to plain C.
 * Original: https://github.com/bitcraze/Bitcraze_PMW3901
 *
 * Copyright (c) 2017 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "pmw3901.h"
#include <stddef.h>

// ----------------------------------------------------------------------------
// Low-level register access
// ----------------------------------------------------------------------------

void pmw3901_register_write(pmw3901_dev_t *dev, uint8_t reg, uint8_t value) {
    // Set MSB for write operation
    reg |= 0x80;

    dev->platform->cs_set(0);  // CS low
    dev->platform->delay_us(50);

    dev->platform->spi_transfer(reg, NULL);
    dev->platform->spi_transfer(value, NULL);

    dev->platform->delay_us(50);
    dev->platform->cs_set(1);  // CS high

    dev->platform->delay_us(200);
}

uint8_t pmw3901_register_read(pmw3901_dev_t *dev, uint8_t reg) {
    uint8_t value;

    // Clear MSB for read operation
    reg &= ~0x80;

    dev->platform->cs_set(0);  // CS low
    dev->platform->delay_us(50);

    dev->platform->spi_transfer(reg, NULL);
    dev->platform->delay_us(50);
    dev->platform->spi_transfer(0x00, &value);

    dev->platform->delay_us(100);
    dev->platform->cs_set(1);  // CS high

    return value;
}

// ----------------------------------------------------------------------------
// Performance optimization registers (magic initialization sequence)
// ----------------------------------------------------------------------------

static void init_registers(pmw3901_dev_t *dev) {
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x61, 0xAD);
    pmw3901_register_write(dev, 0x7F, 0x03);
    pmw3901_register_write(dev, 0x40, 0x00);
    pmw3901_register_write(dev, 0x7F, 0x05);
    pmw3901_register_write(dev, 0x41, 0xB3);
    pmw3901_register_write(dev, 0x43, 0xF1);
    pmw3901_register_write(dev, 0x45, 0x14);
    pmw3901_register_write(dev, 0x5B, 0x32);
    pmw3901_register_write(dev, 0x5F, 0x34);
    pmw3901_register_write(dev, 0x7B, 0x08);
    pmw3901_register_write(dev, 0x7F, 0x06);
    pmw3901_register_write(dev, 0x44, 0x1B);
    pmw3901_register_write(dev, 0x40, 0xBF);
    pmw3901_register_write(dev, 0x4E, 0x3F);
    pmw3901_register_write(dev, 0x7F, 0x08);
    pmw3901_register_write(dev, 0x65, 0x20);
    pmw3901_register_write(dev, 0x6A, 0x18);
    pmw3901_register_write(dev, 0x7F, 0x09);
    pmw3901_register_write(dev, 0x4F, 0xAF);
    pmw3901_register_write(dev, 0x5F, 0x40);
    pmw3901_register_write(dev, 0x48, 0x80);
    pmw3901_register_write(dev, 0x49, 0x80);
    pmw3901_register_write(dev, 0x57, 0x77);
    pmw3901_register_write(dev, 0x60, 0x78);
    pmw3901_register_write(dev, 0x61, 0x78);
    pmw3901_register_write(dev, 0x62, 0x08);
    pmw3901_register_write(dev, 0x63, 0x50);
    pmw3901_register_write(dev, 0x7F, 0x0A);
    pmw3901_register_write(dev, 0x45, 0x60);
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x4D, 0x11);
    pmw3901_register_write(dev, 0x55, 0x80);
    pmw3901_register_write(dev, 0x74, 0x1F);
    pmw3901_register_write(dev, 0x75, 0x1F);
    pmw3901_register_write(dev, 0x4A, 0x78);
    pmw3901_register_write(dev, 0x4B, 0x78);
    pmw3901_register_write(dev, 0x44, 0x08);
    pmw3901_register_write(dev, 0x45, 0x50);
    pmw3901_register_write(dev, 0x64, 0xFF);
    pmw3901_register_write(dev, 0x65, 0x1F);
    pmw3901_register_write(dev, 0x7F, 0x14);
    pmw3901_register_write(dev, 0x65, 0x60);
    pmw3901_register_write(dev, 0x66, 0x08);
    pmw3901_register_write(dev, 0x63, 0x78);
    pmw3901_register_write(dev, 0x7F, 0x15);
    pmw3901_register_write(dev, 0x48, 0x58);
    pmw3901_register_write(dev, 0x7F, 0x07);
    pmw3901_register_write(dev, 0x41, 0x0D);
    pmw3901_register_write(dev, 0x43, 0x14);
    pmw3901_register_write(dev, 0x4B, 0x0E);
    pmw3901_register_write(dev, 0x45, 0x0F);
    pmw3901_register_write(dev, 0x44, 0x42);
    pmw3901_register_write(dev, 0x4C, 0x80);
    pmw3901_register_write(dev, 0x7F, 0x10);
    pmw3901_register_write(dev, 0x5B, 0x02);
    pmw3901_register_write(dev, 0x7F, 0x07);
    pmw3901_register_write(dev, 0x40, 0x41);
    pmw3901_register_write(dev, 0x70, 0x00);

    dev->platform->delay_ms(100);

    pmw3901_register_write(dev, 0x32, 0x44);
    pmw3901_register_write(dev, 0x7F, 0x07);
    pmw3901_register_write(dev, 0x40, 0x40);
    pmw3901_register_write(dev, 0x7F, 0x06);
    pmw3901_register_write(dev, 0x62, 0xf0);
    pmw3901_register_write(dev, 0x63, 0x00);
    pmw3901_register_write(dev, 0x7F, 0x0D);
    pmw3901_register_write(dev, 0x48, 0xC0);
    pmw3901_register_write(dev, 0x6F, 0xd5);
    pmw3901_register_write(dev, 0x7F, 0x00);
    pmw3901_register_write(dev, 0x5B, 0xa0);
    pmw3901_register_write(dev, 0x4E, 0xA8);
    pmw3901_register_write(dev, 0x5A, 0x50);
    pmw3901_register_write(dev, 0x40, 0x80);
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

bool pmw3901_init(pmw3901_dev_t *dev, const pmw3901_platform_t *platform) {
    dev->platform = platform;

    // Reset SPI bus with CS toggling
    dev->platform->cs_set(1);  // CS high
    dev->platform->delay_ms(1);
    dev->platform->cs_set(0);  // CS low
    dev->platform->delay_ms(1);
    dev->platform->cs_set(1);  // CS high
    dev->platform->delay_ms(1);

    // Power on reset
    pmw3901_register_write(dev, 0x3A, 0x5A);
    dev->platform->delay_ms(5);

    // Verify chip ID
    uint8_t chip_id = pmw3901_register_read(dev, 0x00);
    uint8_t chip_id_inv = pmw3901_register_read(dev, 0x5F);

    if (chip_id != PMW3901_CHIP_ID || chip_id_inv != PMW3901_CHIP_ID_INVERSE) {
        return false;
    }

    // Clear motion registers
    pmw3901_register_read(dev, 0x02);
    pmw3901_register_read(dev, 0x03);
    pmw3901_register_read(dev, 0x04);
    pmw3901_register_read(dev, 0x05);
    pmw3901_register_read(dev, 0x06);
    dev->platform->delay_ms(1);

    // Apply performance optimization settings
    init_registers(dev);

    return true;
}

void pmw3901_read_motion(pmw3901_dev_t *dev, int16_t *deltaX, int16_t *deltaY) {
    // Read motion register to latch values
    pmw3901_register_read(dev, 0x02);

    // Read delta X (little-endian: low byte first)
    uint8_t x_lo = pmw3901_register_read(dev, 0x03);
    uint8_t x_hi = pmw3901_register_read(dev, 0x04);
    *deltaX = (int16_t)((x_hi << 8) | x_lo);

    // Read delta Y (little-endian: low byte first)
    uint8_t y_lo = pmw3901_register_read(dev, 0x05);
    uint8_t y_hi = pmw3901_register_read(dev, 0x06);
    *deltaY = (int16_t)((y_hi << 8) | y_lo);
}

void pmw3901_set_led(pmw3901_dev_t *dev, bool led_on) {
    dev->platform->delay_ms(200);
    pmw3901_register_write(dev, 0x7F, 0x14);
    pmw3901_register_write(dev, 0x6F, led_on ? 0x1C : 0x00);
    pmw3901_register_write(dev, 0x7F, 0x00);
}

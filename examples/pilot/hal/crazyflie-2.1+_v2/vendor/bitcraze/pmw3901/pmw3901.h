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

#ifndef PMW3901_H
#define PMW3901_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Expected chip IDs
#define PMW3901_CHIP_ID         0x49
#define PMW3901_CHIP_ID_INVERSE 0xB6

// Platform abstraction callbacks - must be provided by user
typedef struct {
    // SPI transfer: send tx_data, receive into rx_data (can be NULL if not needed)
    // Returns 0 on success, non-zero on error
    int (*spi_transfer)(uint8_t tx_data, uint8_t *rx_data);

    // Chip select control: level 0 = low (active), 1 = high (inactive)
    void (*cs_set)(int level);

    // Delay functions
    void (*delay_ms)(uint32_t ms);
    void (*delay_us)(uint32_t us);
} pmw3901_platform_t;

// Device handle
typedef struct {
    const pmw3901_platform_t *platform;
} pmw3901_dev_t;

// Initialize device with platform callbacks
// Returns true on success (chip ID verified), false on failure
bool pmw3901_init(pmw3901_dev_t *dev, const pmw3901_platform_t *platform);

// Read accumulated motion since last read
// deltaX, deltaY: motion in sensor pixels (can be negative)
void pmw3901_read_motion(pmw3901_dev_t *dev, int16_t *deltaX, int16_t *deltaY);

// Control the sensor LED
void pmw3901_set_led(pmw3901_dev_t *dev, bool led_on);

// Low-level register access (for debugging)
void pmw3901_register_write(pmw3901_dev_t *dev, uint8_t reg, uint8_t value);
uint8_t pmw3901_register_read(pmw3901_dev_t *dev, uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif // PMW3901_H

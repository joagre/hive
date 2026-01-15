// PMW3901 Optical Flow Sensor Driver for Crazyflie Flow Deck v2
//
// The PMW3901 is an optical motion sensor that tracks surface motion.
// Uses SPI interface at 2 MHz.
//
// Resolution: 35x35 pixel array
// Frame rate: up to 121 FPS
// Field of view: 42 degrees
//
// Reference: PMW3901MB-TXQT Datasheet

#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Data Structures
// ----------------------------------------------------------------------------

// Motion data from sensor
typedef struct {
    int16_t delta_x;     // Motion in X direction (pixels)
    int16_t delta_y;     // Motion in Y direction (pixels)
    uint8_t squal;       // Surface quality (0-255, higher = better)
    uint16_t shutter;    // Shutter time (exposure indicator)
    bool motion;         // True if motion detected
} pmw3901_motion_t;

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

// Initialize PMW3901 sensor
// Returns: true on success, false on error
bool pmw3901_init(void);

// Check if sensor is initialized and responding
bool pmw3901_is_ready(void);

// Read motion data (clears delta counters)
// data: Output motion data structure
// Returns: true on success
bool pmw3901_read_motion(pmw3901_motion_t *data);

// Read motion delta only (simpler, faster)
// delta_x, delta_y: Output motion deltas (can be NULL)
// Returns: true on success
bool pmw3901_read_delta(int16_t *delta_x, int16_t *delta_y);

// Enable/disable frame capture mode (for debugging)
void pmw3901_set_frame_capture(bool enable);

// Read frame buffer (35x35 = 1225 bytes)
// Must call pmw3901_set_frame_capture(true) first
// buf: Output buffer (at least 1225 bytes)
// Returns: true on success
bool pmw3901_read_frame(uint8_t *buf);

// Get accumulated motion since last read (doesn't reset)
bool pmw3901_get_accumulated(int16_t *delta_x, int16_t *delta_y);

// Software reset
void pmw3901_reset(void);

// ----------------------------------------------------------------------------
// Low-Level SPI Interface (to be implemented by platform)
// ----------------------------------------------------------------------------

// These functions must be implemented by the platform layer
extern void pmw3901_cs_low(void);
extern void pmw3901_cs_high(void);
extern uint8_t pmw3901_spi_transfer(uint8_t data);
extern void pmw3901_delay_us(uint32_t us);
extern void pmw3901_delay_ms(uint32_t ms);

#endif // PMW3901_H

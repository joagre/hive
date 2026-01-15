// VL53L1x Time-of-Flight Distance Sensor Driver for Crazyflie Flow Deck v2
//
// The VL53L1x is a laser-ranging ToF sensor.
// Uses I2C interface.
//
// Range: up to 4 meters
// Resolution: 1mm
// Frame rate: up to 50 Hz
//
// Reference: ST VL53L1X Datasheet (DocID031436)

#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Distance mode
typedef enum {
    VL53L1X_DIST_SHORT  = 1,   // Up to 1.3m, better ambient immunity
    VL53L1X_DIST_LONG   = 2    // Up to 4m, more sensitive to ambient
} vl53l1x_dist_mode_t;

// Timing budget (measurement time in ms)
// Longer = more accurate but slower
typedef enum {
    VL53L1X_TIMING_15MS  = 15,
    VL53L1X_TIMING_20MS  = 20,
    VL53L1X_TIMING_33MS  = 33,
    VL53L1X_TIMING_50MS  = 50,
    VL53L1X_TIMING_100MS = 100,
    VL53L1X_TIMING_200MS = 200,
    VL53L1X_TIMING_500MS = 500
} vl53l1x_timing_t;

// Configuration structure
typedef struct {
    vl53l1x_dist_mode_t distance_mode;
    vl53l1x_timing_t timing_budget_ms;
    uint16_t inter_measurement_ms;   // For continuous mode
} vl53l1x_config_t;

// Default configuration
#define VL53L1X_CONFIG_DEFAULT { \
    .distance_mode = VL53L1X_DIST_SHORT, \
    .timing_budget_ms = VL53L1X_TIMING_33MS, \
    .inter_measurement_ms = 50 \
}

// ----------------------------------------------------------------------------
// Data Structures
// ----------------------------------------------------------------------------

// Range status codes
typedef enum {
    VL53L1X_RANGE_VALID              = 0,
    VL53L1X_RANGE_SIGMA_FAIL         = 1,
    VL53L1X_RANGE_SIGNAL_FAIL        = 2,
    VL53L1X_RANGE_OUT_OF_BOUNDS_FAIL = 4,
    VL53L1X_RANGE_HARDWARE_FAIL      = 5,
    VL53L1X_RANGE_WRAP_TARGET_FAIL   = 7,
    VL53L1X_RANGE_NO_TARGET          = 255
} vl53l1x_range_status_t;

// Full ranging data
typedef struct {
    uint16_t distance_mm;            // Distance in millimeters
    uint16_t signal_rate;            // Signal strength (MCPS)
    uint16_t ambient_rate;           // Ambient light level (MCPS)
    uint16_t sigma_mm;               // Estimated standard deviation (mm)
    vl53l1x_range_status_t status;   // Range validity status
} vl53l1x_result_t;

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

// Initialize VL53L1x sensor
// config: Configuration (or NULL for defaults)
// Returns: true on success
bool vl53l1x_init(const vl53l1x_config_t *config);

// Check if sensor is initialized and responding
bool vl53l1x_is_ready(void);

// Start continuous ranging mode
bool vl53l1x_start_ranging(void);

// Stop continuous ranging mode
bool vl53l1x_stop_ranging(void);

// Check if new data is available
bool vl53l1x_data_ready(void);

// Read distance (mm) - waits for data if not ready
// Returns 0 on error or if no target detected
uint16_t vl53l1x_read_distance(void);

// Read full ranging result with status
bool vl53l1x_read_result(vl53l1x_result_t *result);

// Clear interrupt (call after reading data in interrupt mode)
void vl53l1x_clear_interrupt(void);

// Perform single measurement (blocking)
uint16_t vl53l1x_measure_single(void);

// Set distance mode
bool vl53l1x_set_distance_mode(vl53l1x_dist_mode_t mode);

// Set timing budget
bool vl53l1x_set_timing_budget(vl53l1x_timing_t timing_ms);

// Get sensor model ID
uint16_t vl53l1x_get_model_id(void);

// Software reset
void vl53l1x_reset(void);

// ----------------------------------------------------------------------------
// Low-Level I2C Interface (to be implemented by platform)
// ----------------------------------------------------------------------------

// Default I2C address
#define VL53L1X_I2C_ADDR_DEFAULT 0x29

// These functions must be implemented by the platform layer
extern bool vl53l1x_i2c_read(uint8_t addr, uint16_t reg, uint8_t *data, uint16_t len);
extern bool vl53l1x_i2c_write(uint8_t addr, uint16_t reg, uint8_t *data, uint16_t len);
extern void vl53l1x_delay_ms(uint32_t ms);

#endif // VL53L1X_H

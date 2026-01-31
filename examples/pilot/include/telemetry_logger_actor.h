// Telemetry logger actor header
//
// Logs flight telemetry data to CSV file for analysis.
// Works on both Webots simulation and Crazyflie hardware.
// Useful for PID tuning and flight analysis.
//
// Storage selection:
// - Prefers /sd (SD card on Crazyflie with SD deck)
// - Falls back to /tmp (Webots simulation)
// - Gracefully disables if no storage available

#ifndef TELEMETRY_LOGGER_ACTOR_H
#define TELEMETRY_LOGGER_ACTOR_H

#include "hive_types.h"
#include "pilot_buses.h"

// Logging rate in Hz (25Hz = 40ms interval)
#define TELEMETRY_LOG_RATE_HZ 25

// Configuration for telemetry logger actor
typedef struct {
    const pilot_buses_t *buses;
} telemetry_logger_config_t;

// Initialize telemetry logger state from config
void *telemetry_logger_init(void *init_args);

// Telemetry logger actor entry point
void telemetry_logger_actor(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count);

#endif // TELEMETRY_LOGGER_ACTOR_H

// Telemetry logger actor header
//
// Logs flight telemetry data to CSV file for analysis.
// Works on both Webots simulation and Crazyflie hardware.
// Useful for PID tuning and flight analysis.
//
// Enable with -DENABLE_TELEMETRY_LOG=1 (default: on for Webots, off for Crazyflie)

#ifndef TELEMETRY_LOGGER_ACTOR_H
#define TELEMETRY_LOGGER_ACTOR_H

#include "hive_types.h"

// Auto-enable for Webots, disable for Crazyflie (no SD card by default)
#ifndef ENABLE_TELEMETRY_LOG
#ifdef SIMULATED_TIME
#define ENABLE_TELEMETRY_LOG 1
#else
#define ENABLE_TELEMETRY_LOG 0
#endif
#endif

#if ENABLE_TELEMETRY_LOG

#include "pilot_buses.h"

// Logging rate in Hz (25Hz = 40ms interval)
#define TELEMETRY_LOG_RATE_HZ 25

// Configuration for telemetry logger actor
typedef struct {
    const pilot_buses_t *buses;
    const char *log_path;
} telemetry_logger_config_t;

// Initialize telemetry logger state from config
void *telemetry_logger_init(void *init_args);

// Telemetry logger actor entry point
void telemetry_logger_actor(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count);

#endif // ENABLE_TELEMETRY_LOG

#endif // TELEMETRY_LOGGER_ACTOR_H

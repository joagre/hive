// Logger actor header
//
// Manages both hive runtime log and telemetry CSV log.
// - Hive log: system messages, errors, debug output
// - Telemetry CSV: flight data at 25Hz for PID tuning
//
// Storage selection:
// - Prefers /sd (SD card on Crazyflie with SD deck)
// - Falls back to /tmp (Webots simulation)
// - Gracefully disables if no storage available

#ifndef LOGGER_ACTOR_H
#define LOGGER_ACTOR_H

#include "hive_types.h"
#include "pilot_buses.h"

// TELEMETRY_LOG_RATE_HZ defined in config.h

// Configuration for logger actor
typedef struct {
    const pilot_buses_t *buses;
} logger_config_t;

// Initialize logger state from config
void *logger_actor_init(void *init_args);

// Logger actor entry point
void logger_actor(void *args, const hive_spawn_info_t *siblings,
                  size_t sibling_count);

#endif // LOGGER_ACTOR_H

// Battery monitoring actor header
//
// Monitors battery voltage at 2 Hz via hal_power_get_battery().
// Two-tier thresholds (Bitcraze values):
//   WARNING:  3.2V - log once
//   CRITICAL: 3.0V - debounced (10 consecutive = 5s), emergency landing
//
// On simulation platforms, hal_power_get_battery() returns 4.2V,
// so this actor runs harmlessly (never triggers).

#ifndef BATTERY_ACTOR_H
#define BATTERY_ACTOR_H

#include "hive_types.h"

// Initialize battery actor state from pilot_buses (extracts params)
void *battery_actor_init(void *init_args);

// Battery actor entry point
void battery_actor(void *args, const hive_spawn_info_t *siblings,
                   size_t sibling_count);

#endif // BATTERY_ACTOR_H

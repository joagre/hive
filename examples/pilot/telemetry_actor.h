// Telemetry actor header
//
// Radio telemetry for ground station logging/analysis.
// Only available on platforms with HAL_HAS_RADIO defined.

#ifndef TELEMETRY_ACTOR_H
#define TELEMETRY_ACTOR_H

#include "hive_types.h"

#ifdef HAL_HAS_RADIO

// Initialize telemetry actor state from pilot_buses
void *telemetry_actor_init(void *init_args);

// Telemetry actor entry point
void telemetry_actor(void *args, const hive_spawn_info *siblings,
                     size_t sibling_count);

#endif // HAL_HAS_RADIO

#endif // TELEMETRY_ACTOR_H

// Comms actor header
//
// Ground station communication: telemetry, log download, command upload.
// Only available on platforms with HAL_HAS_RADIO defined.

#ifndef COMMS_ACTOR_H
#define COMMS_ACTOR_H

#include "hive_types.h"

#ifdef HAL_HAS_RADIO

// Initialize comms actor state from pilot_buses
void *comms_actor_init(void *init_args);

// Comms actor entry point
void comms_actor(void *args, const hive_spawn_info_t *siblings,
                 size_t sibling_count);

#endif // HAL_HAS_RADIO

#endif // COMMS_ACTOR_H

// Motor actor - Output layer
//
// Subscribes to torque bus, writes to hardware via HAL.
// Uses hive_select() to wait on torque bus OR STOP notification simultaneously.

#ifndef MOTOR_ACTOR_H
#define MOTOR_ACTOR_H

#include "hive_runtime.h"

// Init function - extracts bus IDs from pilot_buses
void *motor_actor_init(void *init_args);

// Actor entry point
void motor_actor(void *args, const hive_spawn_info *siblings,
                 size_t sibling_count);

#endif // MOTOR_ACTOR_H

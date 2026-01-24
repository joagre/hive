// Hardware Abstraction Layer - Context Switching
//
// Each platform provides:
// - hive_context_t struct (in hive_hal_context_defs.h)
// - hive_context_init() implementation
// - hive_context_switch_asm() assembly

#ifndef HIVE_HAL_CONTEXT_H
#define HIVE_HAL_CONTEXT_H

#include <stddef.h>

// Platform-specific context struct
// Included from src/hal/<platform>/hive_hal_context_defs.h via build system
#include "hive_hal_context_defs.h"

// Initialize a new context for an actor.
// stack: pointer to base of stack allocation (stack grows down)
// stack_size: size of stack in bytes
// fn: actor function to call when context is first entered
//
// Platform implementations handle:
// - Stack alignment (16 bytes on x86-64, 8 bytes on ARM)
// - Return address setup
// - Callee-saved register initialization
void hive_context_init(hive_context_t *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *, const void *, size_t));

// Switch from current context to target context.
// Saves CPU state in 'from', restores from 'to'.
// Implemented in platform-specific assembly.
void hive_context_switch(hive_context_t *from, hive_context_t *to);

#endif // HIVE_HAL_CONTEXT_H

// HAL Context Implementation - STM32 (ARM Cortex-M)
//
// Platform-specific context initialization and entry for ARM Cortex-M.
// Assembly context switch is in hive_context_arm_cm.S

#include "hal/hive_hal_context.h"
#include "hive_internal.h"
#include "hive_actor.h"
#include <stdint.h>
#include <string.h>

// Forward declaration of assembly function
extern void hive_context_switch_asm(hive_context_t *from, hive_context_t *to);

// Forward declaration of the actual actor_t runner (not static - needed for asm
// branch)
void hive_context_entry_run(void (*fn)(void *, const hive_spawn_info_t *,
                                       size_t));

// Naked wrapper - no prologue/epilogue, so r4 is preserved from context switch
__attribute__((naked)) static void context_entry(void) {
    // r4 = fn (set by hive_context_init, preserved by context switch)
    __asm__ volatile("mov r0, r4\n"               // r0 = fn
                     "b hive_context_entry_run\n" // tail call (no return)
    );
}

// Actual actor_t runner - called with fn as parameter
// Retrieves startup info from current actor_t and calls actor_t function
void hive_context_entry_run(void (*fn)(void *, const hive_spawn_info_t *,
                                       size_t)) {
    // Get startup info from current actor_t
    actor_t *current = hive_actor_current();
    void *args = current->startup_args;
    const hive_spawn_info_t *siblings = current->startup_siblings;
    size_t sibling_count = current->startup_sibling_count;

    // Call the actor_t function with all three arguments
    fn(args, siblings, sibling_count);

    // Actor returned without calling hive_exit() - this is a crash
    hive_exit_crash();
}

void hive_context_init(hive_context_t *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *, const void *, size_t)) {
    // Zero out context
    memset(ctx, 0, sizeof(hive_context_t));

    // Stack grows down on ARM
    // Calculate stack top (align to 8 bytes as required by ARM AAPCS)
    uintptr_t stack_top = (uintptr_t)stack + stack_size;
    stack_top &= ~((uintptr_t)7); // Align to 8 bytes

    // Store function pointer in callee-saved register r4
    // Other startup info (args, siblings, count) is stored in actor_t struct
    // Use memcpy to avoid function pointer to void* conversion warning
    memcpy(&ctx->r4, &fn, sizeof(fn));

    // Push return address (context_entry) onto stack
    // The context switch will pop this into PC via pop {pc}
    // IMPORTANT: On Cortex-M, addresses loaded into PC must have LSB=1 for
    // Thumb mode
    stack_top -= sizeof(void *);
    uintptr_t entry_addr = (uintptr_t)context_entry;
    entry_addr |= 1; // Ensure Thumb bit is set
    *(uintptr_t *)stack_top = entry_addr;

    ctx->sp = (void *)stack_top;
}

void hive_context_switch(hive_context_t *from, hive_context_t *to) {
    hive_context_switch_asm(from, to);
}

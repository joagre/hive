// HAL Context Implementation - Linux (x86-64)
//
// Platform-specific context initialization and entry for x86-64.
// Assembly context switch is in hive_context_x86_64.S

#include "hal/hive_hal_context.h"
#include "hive_internal.h"
#include "hive_runtime.h"
#include "hive_actor.h"
#include <stdint.h>
#include <string.h>

// Forward declaration of assembly function
extern void hive_context_switch_asm(hive_context_t *from, hive_context_t *to);

// Wrapper function that calls the actor_t function and handles return
static void context_entry(void) {
    // When we first enter, r12 contains our function pointer
    // We need to extract it via inline assembly
    void (*fn)(void *, const hive_spawn_info_t *, size_t);

    __asm__ volatile("movq %%r12, %0\n" : "=r"(fn) : : "r12");

    // Get startup info from current actor_t
    actor_t *current = hive_actor_current();
    void *args = current->startup_args;
    const hive_spawn_info_t *siblings = current->startup_siblings;
    size_t sibling_count = current->startup_sibling_count;

    // Call the actor_t function with all three arguments
    fn(args, siblings, sibling_count);

    // Actor returned without calling hive_exit() - treat as normal exit
    // (Erlang semantics: process with no more code to execute terminates normally)
    hive_exit(HIVE_EXIT_REASON_NORMAL);
}

void hive_context_init(hive_context_t *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *, const void *, size_t)) {
    // Zero out context
    memset(ctx, 0, sizeof(hive_context_t));

    // Stack grows down on x86-64
    // Calculate stack top (align to 16 bytes as required by x86-64 ABI)
    uintptr_t stack_top = (uintptr_t)stack + stack_size;
    stack_top &= ~((uintptr_t)15); // Align to 16 bytes

    // x86-64 ABI requires RSP % 16 == 8 when entering a function (before
    // pushing frame pointer) Our context switch uses RET to jump to
    // context_entry, which pops the return address So we need: (RSP after RET)
    // % 16 == 8 Which means: (RSP before RET) % 16 == 0 (since RET adds 8) We
    // already have 16-byte alignment, so just push the return address

    // Store function pointer in callee-saved register r12
    // Other startup info (args, siblings, count) is stored in actor_t struct
    // Use a union to safely convert function pointer to void pointer
    union {
        void (*fn_ptr)(void *, const void *, size_t);
        void *obj_ptr;
    } fn_conv;
    fn_conv.fn_ptr = fn;

    union {
        void (*fn_ptr)(void);
        void *obj_ptr;
    } entry_conv;
    entry_conv.fn_ptr = context_entry;

    ctx->r12 = fn_conv.obj_ptr;

    // Set instruction pointer to context_entry
    // We do this by pushing the return address onto the stack
    // When the context switch does 'ret', it will pop this address and jump to
    // it After this push, RSP will be at (16-byte aligned - 8) After RET pops
    // it, RSP will be 16-byte aligned - but we need it to be (16-aligned - 8)!
    // So we need to push an extra dummy value first
    stack_top -= sizeof(void *);
    *(void **)stack_top = (void *)0; // Dummy padding for alignment
    stack_top -= sizeof(void *);
    *(void **)stack_top = entry_conv.obj_ptr;

    ctx->rsp = (void *)stack_top;
}

void hive_context_switch(hive_context_t *from, hive_context_t *to) {
    hive_context_switch_asm(from, to);
}

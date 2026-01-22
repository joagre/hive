// Hardware Abstraction Layer - Context Initialization Template
//
// This file provides the context initialization function for your platform.
// Copy to src/hal/<platform>/hive_hal_context_<platform>.c and implement.
//
// Context initialization sets up a new actor's stack so that when
// hive_context_switch() switches to it, execution begins at the actor's
// entry point function.

#include "hal/hive_hal_context.h"
#include <stdint.h>
#include <string.h>

// The actor entry point wrapper (defined in hive_actor.c)
extern void hive_actor_entry(void *args, const void *siblings, size_t count);

// Initialize a context for a new actor.
//
// Parameters:
//   ctx        - Context structure to initialize
//   stack      - Base of allocated stack (low address)
//   stack_size - Size of stack in bytes
//   fn         - Actor entry point (actually hive_actor_entry)
//
// The stack grows DOWN on most architectures. This function must:
//   1. Calculate the stack top (stack + stack_size)
//   2. Align the stack pointer (16 bytes on x86-64, 8 bytes on ARM)
//   3. Set up the initial stack frame so context_switch will:
//      - Start executing at fn
//      - Have valid callee-saved registers
//
// Stack layout after initialization (example, grows down):
//
//   High addresses
//   +------------------+
//   | return address   |  <- Points to fn or trampoline
//   +------------------+
//   | saved registers  |  <- Initial values (typically 0)
//   +------------------+
//   | ...              |
//   +------------------+  <- ctx->sp points here
//   | (free stack)     |
//   +------------------+
//   Low addresses (stack base)
//
void hive_context_init(hive_context *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *, const void *, size_t)) {
    // Calculate stack top (stacks grow down)
    uintptr_t stack_top = (uintptr_t)stack + stack_size;

    // Align stack pointer (architecture-specific)
    // x86-64: 16-byte alignment
    // ARM: 8-byte alignment (AAPCS)
    stack_top &= ~(uintptr_t)0xF; // 16-byte align (conservative)

    // TODO: Set up initial stack frame
    //
    // The context_switch assembly will pop registers and "return" to
    // the address at the top of the stack. You need to arrange the
    // stack so that:
    //
    // 1. The "return address" points to fn (or a trampoline)
    // 2. Callee-saved registers are initialized (typically to 0)
    // 3. ctx->sp points to where context_switch expects
    //
    // Example for x86-64:
    //   Push return address (fn)
    //   Push rbp, rbx, r12-r15 (all zeros)
    //   Set ctx->rsp to current stack position
    //
    // Example for ARM Cortex-M:
    //   Store r4-r11 at known offsets
    //   Set LR (stored in r4) to fn address
    //   Set ctx->sp to stack position

    // Initialize context to zeros
    memset(ctx, 0, sizeof(*ctx));

    // TODO: Set stack pointer in context
    ctx->sp = (void *)stack_top;

    // TODO: Set up return address or entry point
    // This is architecture-specific

    (void)fn; // TODO: Use fn as entry point
}

#include "hive_context.h"
#include "hive_internal.h"
#include <stdint.h>
#include <string.h>

// Forward declaration of assembly function
extern void hive_context_switch_asm(hive_context *from, hive_context *to);

#if defined(HIVE_PLATFORM_LINUX) || !defined(HIVE_PLATFORM_STM32)
// =============================================================================
// x86-64 Implementation
// =============================================================================

// Wrapper function that calls the actor function and handles return
static void context_entry(void) {
    // When we first enter, r12 and r13 contain our function and argument
    // We need to extract them via inline assembly
    void (*fn)(void *);
    void *arg;

    __asm__ volatile("movq %%r12, %0\n"
                     "movq %%r13, %1\n"
                     : "=r"(fn), "=r"(arg)
                     :
                     : "r12", "r13");

    fn(arg);

    // Actor returned without calling hive_exit() - this is a crash
    hive_exit_crash();
}

void hive_context_init(hive_context *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *), void *arg) {
    // Zero out context
    memset(ctx, 0, sizeof(hive_context));

    // Stack grows down on x86-64
    // Calculate stack top (align to 16 bytes as required by x86-64 ABI)
    uintptr_t stack_top = (uintptr_t)stack + stack_size;
    stack_top &= ~((uintptr_t)15); // Align to 16 bytes

    // x86-64 ABI requires RSP % 16 == 8 when entering a function (before
    // pushing frame pointer) Our context switch uses RET to jump to
    // context_entry, which pops the return address So we need: (RSP after RET)
    // % 16 == 8 Which means: (RSP before RET) % 16 == 0 (since RET adds 8) We
    // already have 16-byte alignment, so just push the return address

    // Store function and argument in callee-saved registers
    // These will be preserved across the context switch
    // Use a union to safely convert function pointer to void pointer
    union {
        void (*fn_ptr)(void *);
        void *obj_ptr;
    } fn_conv;
    fn_conv.fn_ptr = fn;

    union {
        void (*fn_ptr)(void);
        void *obj_ptr;
    } entry_conv;
    entry_conv.fn_ptr = context_entry;

    ctx->r12 = fn_conv.obj_ptr;
    ctx->r13 = arg;

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

#elif defined(HIVE_PLATFORM_STM32)
// =============================================================================
// ARM Cortex-M Implementation
// =============================================================================

// Forward declaration of the actual actor runner (not static - needed for asm
// branch)
void hive_context_entry_run(void (*fn)(void *), void *arg);

// Naked wrapper - no prologue/epilogue, so r4/r5 are preserved from context
// switch
__attribute__((naked)) static void context_entry(void) {
    // r4 = fn, r5 = arg (set by hive_context_init, preserved by context switch)
    __asm__ volatile("mov r0, r4\n"               // r0 = fn
                     "mov r1, r5\n"               // r1 = arg
                     "b hive_context_entry_run\n" // tail call (no return)
    );
}

// Actual actor runner - called with fn and arg as parameters
void hive_context_entry_run(void (*fn)(void *), void *arg) {
    fn(arg);

    // Actor returned without calling hive_exit() - this is a crash
    hive_exit_crash();
}

void hive_context_init(hive_context *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *), void *arg) {
    // Zero out context
    memset(ctx, 0, sizeof(hive_context));

    // Stack grows down on ARM
    // Calculate stack top (align to 8 bytes as required by ARM AAPCS)
    uintptr_t stack_top = (uintptr_t)stack + stack_size;
    stack_top &= ~((uintptr_t)7); // Align to 8 bytes

    // Store function and argument in callee-saved registers
    // These will be preserved across the context switch
    // Use memcpy to avoid function pointer to void* conversion warning
    memcpy(&ctx->r4, &fn, sizeof(fn));
    ctx->r5 = arg;

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

#endif

void hive_context_switch(hive_context *from, hive_context *to) {
    hive_context_switch_asm(from, to);
}

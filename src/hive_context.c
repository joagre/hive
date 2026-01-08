#include "hive_context.h"
#include "hive_internal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// Forward declaration of assembly function
extern void hive_context_switch_asm(hive_context *from, hive_context *to);

// Wrapper function that calls the actor function and handles return
static void context_entry(void) {
    // When we first enter, r12 and r13 contain our function and argument
    // We need to extract them via inline assembly
    void (*fn)(void *);
    void *arg;

    __asm__ volatile (
        "movq %%r12, %0\n"
        "movq %%r13, %1\n"
        : "=r"(fn), "=r"(arg)
        :
        : "r12", "r13"
    );

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
    stack_top &= ~((uintptr_t)15);  // Align to 16 bytes

    // x86-64 ABI requires RSP % 16 == 8 when entering a function (before pushing frame pointer)
    // Our context switch uses RET to jump to context_entry, which pops the return address
    // So we need: (RSP after RET) % 16 == 8
    // Which means: (RSP before RET) % 16 == 0 (since RET adds 8)
    // We already have 16-byte alignment, so just push the return address

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
    // When the context switch does 'ret', it will pop this address and jump to it
    // After this push, RSP will be at (16-byte aligned - 8)
    // After RET pops it, RSP will be 16-byte aligned - but we need it to be (16-aligned - 8)!
    // So we need to push an extra dummy value first
    stack_top -= sizeof(void *);
    *(void **)stack_top = (void *)0;  // Dummy padding for alignment
    stack_top -= sizeof(void *);
    *(void **)stack_top = entry_conv.obj_ptr;

    ctx->rsp = (void *)stack_top;
}

void hive_context_switch(hive_context *from, hive_context *to) {
    hive_context_switch_asm(from, to);
}

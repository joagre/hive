#ifndef HIVE_CONTEXT_H
#define HIVE_CONTEXT_H

#include <stddef.h>
#include "hive_static_config.h"

#if defined(HIVE_PLATFORM_LINUX) || !defined(HIVE_PLATFORM_STM32)
// Context for x86-64
// Stores callee-saved registers: rbx, rbp, r12-r15, and rsp
typedef struct {
    void *rsp; // Stack pointer
    void *rbx;
    void *rbp;
    void *r12;
    void *r13;
    void *r14;
    void *r15;
} hive_context;

#elif defined(HIVE_PLATFORM_STM32)
// Context for ARM Cortex-M4F (with FPU)
// Stores callee-saved registers: r4-r11 and sp (r13)
// Also stores FPU callee-saved registers: s16-s31
// Note: lr (r14) is saved on stack by context switch
typedef struct {
    void *sp; // Stack pointer (r13)
    void *r4;
    void *r5;
    void *r6;
    void *r7;
    void *r8;
    void *r9;
    void *r10;
    void *r11;
    // FPU callee-saved registers (Cortex-M4F)
    float s16, s17, s18, s19;
    float s20, s21, s22, s23;
    float s24, s25, s26, s27;
    float s28, s29, s30, s31;
} hive_context;

#endif

// Switch from 'from' context to 'to' context
// Saves current context in 'from' and restores 'to'
void hive_context_switch(hive_context *from, hive_context *to);

// Initialize a new context
// stack: pointer to top of stack (stack grows down)
// stack_size: size of stack in bytes
// fn: function to execute
// arg: argument to pass to function
void hive_context_init(hive_context *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *), void *arg);

#endif // HIVE_CONTEXT_H

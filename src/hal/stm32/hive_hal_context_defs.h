// ARM Cortex-M4F Context Structure
//
// Stores callee-saved registers per ARM AAPCS:
// r4-r11, sp (r13), and FPU registers s16-s31

#ifndef HIVE_HAL_CONTEXT_DEFS_H
#define HIVE_HAL_CONTEXT_DEFS_H

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

#endif // HIVE_HAL_CONTEXT_DEFS_H

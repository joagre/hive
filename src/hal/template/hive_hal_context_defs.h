// Hardware Abstraction Layer - Context Definitions Template
//
// This file defines the platform-specific context structure used for
// cooperative context switching between actors.
//
// Copy this file to src/hal/<platform>/hive_hal_context_defs.h and modify
// the hive_context struct for your architecture.
//
// Requirements:
//   1. The struct must store all callee-saved registers
//   2. The struct must store the stack pointer
//   3. Size should be minimal (stored per-actor)
//
// You also need to implement:
//   - hive_hal_context_<platform>.c - Context initialization
//   - hive_context_<arch>.S - Context switch assembly

#ifndef HIVE_HAL_CONTEXT_DEFS_H
#define HIVE_HAL_CONTEXT_DEFS_H

#include <stdint.h>

// =============================================================================
// Example: x86-64 Context (Linux)
// =============================================================================
//
// Callee-saved registers per System V AMD64 ABI:
//   rbx, rbp, r12, r13, r14, r15, rsp
//
// typedef struct {
//     void *rsp;  // Stack pointer
//     void *rbx;  // Callee-saved
//     void *rbp;  // Frame pointer
//     void *r12;  // Callee-saved
//     void *r13;  // Callee-saved
//     void *r14;  // Callee-saved
//     void *r15;  // Callee-saved
// } hive_context;

// =============================================================================
// Example: ARM Cortex-M Context (STM32)
// =============================================================================
//
// Callee-saved registers per ARM AAPCS:
//   r4-r11, sp
// With FPU (Cortex-M4F):
//   s16-s31 are also callee-saved
//
// typedef struct {
//     void *sp;     // Stack pointer (r13)
//     uint32_t r4;  // Callee-saved registers
//     uint32_t r5;
//     uint32_t r6;
//     uint32_t r7;
//     uint32_t r8;
//     uint32_t r9;
//     uint32_t r10;
//     uint32_t r11;
// #ifdef __ARM_FP
//     uint32_t s16_31[16];  // FPU callee-saved (64 bytes)
// #endif
// } hive_context;

// =============================================================================
// Your Platform Context
// =============================================================================

typedef struct {
    void *sp; // Stack pointer (required)
    // TODO: Add callee-saved registers for your architecture
    // Use your platform's ABI documentation to determine which registers
    // must be preserved across function calls.
} hive_context;

#endif // HIVE_HAL_CONTEXT_DEFS_H

// Hardware Abstraction Layer - Context Switch Assembly Template
//
// This file provides the context switch function for your platform.
// Copy to src/hal/<platform>/hive_context_<arch>.S and implement.
//
// Context switching saves the current actor's registers and restores
// the next actor's registers. This is the core of cooperative multitasking.
//
// Function signature (C):
//   void hive_context_switch(hive_context *from, hive_context *to);
//
// Parameters:
//   from - Context structure to save current state into
//   to   - Context structure to restore state from
//
// Requirements:
//   1. Save ALL callee-saved registers to 'from' context
//   2. Save the stack pointer to 'from' context
//   3. Restore the stack pointer from 'to' context
//   4. Restore ALL callee-saved registers from 'to' context
//   5. Return (which jumps to the saved return address)
//
// Notes:
//   - Caller-saved registers are already saved by the C calling convention
//   - The return address is typically on the stack or in a link register
//   - This function "returns" to a different location than it was called from

// =============================================================================
// Example: x86-64 (System V AMD64 ABI)
// =============================================================================
//
// Callee-saved: rbx, rbp, r12, r13, r14, r15
// Stack pointer: rsp
// Return address: on stack
//
// .global hive_context_switch
// .type hive_context_switch, @function
// hive_context_switch:
//     // Save callee-saved registers to 'from' (rdi)
//     mov %rbx, 8(%rdi)
//     mov %rbp, 16(%rdi)
//     mov %r12, 24(%rdi)
//     mov %r13, 32(%rdi)
//     mov %r14, 40(%rdi)
//     mov %r15, 48(%rdi)
//     mov %rsp, (%rdi)     // Save stack pointer
//
//     // Restore from 'to' (rsi)
//     mov (%rsi), %rsp     // Restore stack pointer
//     mov 8(%rsi), %rbx
//     mov 16(%rsi), %rbp
//     mov 24(%rsi), %r12
//     mov 32(%rsi), %r13
//     mov 40(%rsi), %r14
//     mov 48(%rsi), %r15
//     ret                  // Return to new context

// =============================================================================
// Example: ARM Cortex-M (AAPCS)
// =============================================================================
//
// Callee-saved: r4-r11
// Stack pointer: sp (r13)
// Link register: lr (r14) - return address
//
// .global hive_context_switch
// .type hive_context_switch, %function
// .thumb_func
// hive_context_switch:
//     // Save callee-saved registers
//     push {r4-r11, lr}
//     str sp, [r0]         // Save stack pointer to from->sp
//
//     // Restore from 'to' (r1)
//     ldr sp, [r1]         // Restore stack pointer
//     pop {r4-r11, pc}     // Restore registers and return

// =============================================================================
// Your Platform Implementation
// =============================================================================

    .global hive_context_switch
    .type hive_context_switch, @function  // Use %function for ARM

hive_context_switch:
    // TODO: Implement for your architecture
    //
    // 1. Save callee-saved registers to memory pointed to by first argument
    // 2. Save stack pointer
    // 3. Restore stack pointer from second argument
    // 4. Restore callee-saved registers
    // 5. Return (this will jump to the new context's saved return point)
    //
    // Refer to your platform's ABI documentation for:
    //   - Which registers are callee-saved
    //   - How arguments are passed (registers vs stack)
    //   - Stack alignment requirements
    //   - Return address handling

    ret  // Placeholder - replace with actual implementation

    .size hive_context_switch, .-hive_context_switch

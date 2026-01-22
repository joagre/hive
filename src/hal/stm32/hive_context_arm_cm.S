@ ARM Cortex-M context switch assembly
@ Thumb-2 instruction set
@
@ Supports both Cortex-M3 (no FPU) and Cortex-M4F (with FPU).
@ FPU register save/restore is conditional on __ARM_FP being defined.

    .syntax unified
    .thumb
    .text

    .global hive_context_switch_asm
    .type hive_context_switch_asm, %function
    .thumb_func

@ void hive_context_switch_asm(hive_context *from, hive_context *to)
@ r0 = from context pointer
@ r1 = to context pointer
@
@ hive_context layout:
@   offset 0:  sp  (stack pointer)
@   offset 4:  r4
@   offset 8:  r5
@   offset 12: r6
@   offset 16: r7
@   offset 20: r8
@   offset 24: r9
@   offset 28: r10
@   offset 32: r11
@   offset 36: s16-s31 (FPU callee-saved, 16 x 4 bytes = 64 bytes) - only if FPU present
@
@ The return address is stored on the stack (pushed by init or previous switch).
@ We use 'pop {pc}' to return, which loads the address from stack into PC.

hive_context_switch_asm:
    @ Push LR onto stack (save return address for when we switch back)
    push {lr}

    @ Save current context to 'from' (r0)
    @ Save callee-saved registers r4-r11 and sp (after push)
    str sp,  [r0, #0]
    str r4,  [r0, #4]
    str r5,  [r0, #8]
    str r6,  [r0, #12]
    str r7,  [r0, #16]
    str r8,  [r0, #20]
    str r9,  [r0, #24]
    str r10, [r0, #28]
    str r11, [r0, #32]

#ifdef __ARM_FP
    @ Save FPU callee-saved registers s16-s31 to context (offset 36)
    @ Only on Cortex-M4F and similar with hardware FPU
    add r2, r0, #36
    vstm r2, {s16-s31}
#endif

    @ Restore new context from 'to' (r1)
    ldr sp,  [r1, #0]
    ldr r4,  [r1, #4]
    ldr r5,  [r1, #8]
    ldr r6,  [r1, #12]
    ldr r7,  [r1, #16]
    ldr r8,  [r1, #20]
    ldr r9,  [r1, #24]
    ldr r10, [r1, #28]
    ldr r11, [r1, #32]

#ifdef __ARM_FP
    @ Restore FPU callee-saved registers s16-s31 from context (offset 36)
    @ Only on Cortex-M4F and similar with hardware FPU
    add r2, r1, #36
    vldm r2, {s16-s31}
#endif

    @ Return to entry point (for new context) or saved return address
    @ Use explicit load and bx instead of pop {pc} for clearer Thumb handling
    ldr r0, [sp]        @ Load return address from stack
    add sp, sp, #4      @ Adjust sp (like pop would)
    bx r0               @ Branch with mode exchange

    .size hive_context_switch_asm, .-hive_context_switch_asm

    .end

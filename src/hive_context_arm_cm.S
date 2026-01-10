@ ARM Cortex-M context switch assembly
@ Thumb-2 instruction set

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
@ hive_context layout (9 words = 36 bytes):
@   offset 0:  sp  (stack pointer)
@   offset 4:  r4
@   offset 8:  r5
@   offset 12: r6
@   offset 16: r7
@   offset 20: r8
@   offset 24: r9
@   offset 28: r10
@   offset 32: r11

hive_context_switch_asm:
    @ Save current context to 'from' (r0)
    @ Save callee-saved registers r4-r11 and sp
    str sp,  [r0, #0]
    str r4,  [r0, #4]
    str r5,  [r0, #8]
    str r6,  [r0, #12]
    str r7,  [r0, #16]
    str r8,  [r0, #20]
    str r9,  [r0, #24]
    str r10, [r0, #28]
    str r11, [r0, #32]

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

    @ Return to caller (or to new context entry point)
    @ The return address was pushed on the stack during context_init
    @ or is in LR from a previous call
    bx lr

    .size hive_context_switch_asm, .-hive_context_switch_asm

    .end

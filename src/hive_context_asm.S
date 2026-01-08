.text
.globl hive_context_switch_asm
.type hive_context_switch_asm, @function

hive_context_switch_asm:
    # Save current context (first argument: rdi = from)
    movq %rsp, 0(%rdi)
    movq %rbx, 8(%rdi)
    movq %rbp, 16(%rdi)
    movq %r12, 24(%rdi)
    movq %r13, 32(%rdi)
    movq %r14, 40(%rdi)
    movq %r15, 48(%rdi)

    # Restore new context (second argument: rsi = to)
    movq 0(%rsi), %rsp
    movq 8(%rsi), %rbx
    movq 16(%rsi), %rbp
    movq 24(%rsi), %r12
    movq 32(%rsi), %r13
    movq 40(%rsi), %r14
    movq 48(%rsi), %r15

    ret

.size hive_context_switch_asm, .-hive_context_switch_asm

.section .note.GNU-stack,"",@progbits

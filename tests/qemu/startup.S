/**
 * Startup code for QEMU LM3S6965EVB (Cortex-M3)
 *
 * Provides:
 *   - Interrupt vector table
 *   - Reset handler (copies .data, zeros .bss, calls main)
 *   - SysTick handler for timer ticks
 *   - Default interrupt handlers
 */

    .syntax unified
    .cpu cortex-m3
    .thumb

/* ----------------------------------------------------------------------------
 * Vector table
 * -------------------------------------------------------------------------- */

    .section .isr_vector, "a", %progbits
    .type g_pfnVectors, %object

g_pfnVectors:
    .word _estack                   /* Top of stack */
    .word Reset_Handler             /* Reset handler */
    .word NMI_Handler               /* NMI handler */
    .word HardFault_Handler         /* Hard fault handler */
    .word MemManage_Handler         /* MPU fault handler */
    .word BusFault_Handler          /* Bus fault handler */
    .word UsageFault_Handler        /* Usage fault handler */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word SVC_Handler               /* SVCall handler */
    .word DebugMon_Handler          /* Debug monitor handler */
    .word 0                         /* Reserved */
    .word PendSV_Handler            /* PendSV handler */
    .word SysTick_Handler           /* SysTick handler */

    .size g_pfnVectors, .-g_pfnVectors

/* ----------------------------------------------------------------------------
 * Reset handler
 * -------------------------------------------------------------------------- */

    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
    .thumb_func

Reset_Handler:
    /* Set stack pointer */
    ldr sp, =_estack

    /* Copy .data section from Flash to RAM */
    ldr r0, =_sdata         /* Destination (RAM) */
    ldr r1, =_edata         /* End of destination */
    ldr r2, =_sidata        /* Source (Flash) */
    movs r3, #0
    b LoopCopyDataInit

CopyDataInit:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4

LoopCopyDataInit:
    adds r4, r0, r3
    cmp r4, r1
    bcc CopyDataInit

    /* Zero .bss section */
    ldr r0, =_sbss          /* Start of BSS */
    ldr r1, =_ebss          /* End of BSS */
    movs r2, #0
    b LoopFillZerobss

FillZerobss:
    str r2, [r0]
    adds r0, r0, #4

LoopFillZerobss:
    cmp r0, r1
    bcc FillZerobss

    /* Call main() */
    bl main

    /* If main returns, exit via semihosting */
    mov r0, #0              /* Exit code */
    bl semihosting_exit

    /* Fallback: infinite loop */
    b .

    .size Reset_Handler, .-Reset_Handler

/* ----------------------------------------------------------------------------
 * SysTick handler - calls hive_timer_tick_isr
 * -------------------------------------------------------------------------- */

    .section .text.SysTick_Handler
    .weak SysTick_Handler
    .type SysTick_Handler, %function
    .thumb_func

SysTick_Handler:
    push {lr}
    bl hive_timer_tick_isr
    pop {pc}

    .size SysTick_Handler, .-SysTick_Handler

/* ----------------------------------------------------------------------------
 * Default interrupt handlers
 * -------------------------------------------------------------------------- */

    .section .text.Default_Handler, "ax", %progbits
    .thumb_func

Default_Handler:
Infinite_Loop:
    b Infinite_Loop

    .size Default_Handler, .-Default_Handler

/* Weak aliases for handlers */
    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler
    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler
    .weak MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler
    .weak BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler
    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler
    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler
    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler
    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .end

/**
 * Startup code for STM32F401CEU6 (STEVAL-DRONE01)
 *
 * Based on CMSIS startup file for STM32F401xE.
 * Provides:
 *   - Interrupt vector table
 *   - Reset handler (copies .data, zeros .bss, calls main)
 *   - Default interrupt handlers
 */

    .syntax unified
    .cpu cortex-m4
    .fpu fpv4-sp-d16
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

    /* External interrupts (STM32F401 specific) */
    .word WWDG_IRQHandler           /* Window watchdog */
    .word PVD_IRQHandler            /* PVD through EXTI */
    .word TAMP_STAMP_IRQHandler     /* Tamper and timestamp */
    .word RTC_WKUP_IRQHandler       /* RTC wakeup */
    .word FLASH_IRQHandler          /* Flash */
    .word RCC_IRQHandler            /* RCC */
    .word EXTI0_IRQHandler          /* EXTI line 0 */
    .word EXTI1_IRQHandler          /* EXTI line 1 */
    .word EXTI2_IRQHandler          /* EXTI line 2 */
    .word EXTI3_IRQHandler          /* EXTI line 3 */
    .word EXTI4_IRQHandler          /* EXTI line 4 */
    .word DMA1_Stream0_IRQHandler   /* DMA1 stream 0 */
    .word DMA1_Stream1_IRQHandler   /* DMA1 stream 1 */
    .word DMA1_Stream2_IRQHandler   /* DMA1 stream 2 */
    .word DMA1_Stream3_IRQHandler   /* DMA1 stream 3 */
    .word DMA1_Stream4_IRQHandler   /* DMA1 stream 4 */
    .word DMA1_Stream5_IRQHandler   /* DMA1 stream 5 */
    .word DMA1_Stream6_IRQHandler   /* DMA1 stream 6 */
    .word ADC_IRQHandler            /* ADC1 */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word EXTI9_5_IRQHandler        /* EXTI lines 5-9 */
    .word TIM1_BRK_TIM9_IRQHandler  /* TIM1 break, TIM9 */
    .word TIM1_UP_TIM10_IRQHandler  /* TIM1 update, TIM10 */
    .word TIM1_TRG_COM_TIM11_IRQHandler /* TIM1 trigger/commutation, TIM11 */
    .word TIM1_CC_IRQHandler        /* TIM1 capture compare */
    .word TIM2_IRQHandler           /* TIM2 */
    .word TIM3_IRQHandler           /* TIM3 */
    .word TIM4_IRQHandler           /* TIM4 */
    .word I2C1_EV_IRQHandler        /* I2C1 event */
    .word I2C1_ER_IRQHandler        /* I2C1 error */
    .word I2C2_EV_IRQHandler        /* I2C2 event */
    .word I2C2_ER_IRQHandler        /* I2C2 error */
    .word SPI1_IRQHandler           /* SPI1 */
    .word SPI2_IRQHandler           /* SPI2 */
    .word USART1_IRQHandler         /* USART1 */
    .word USART2_IRQHandler         /* USART2 */
    .word 0                         /* Reserved */
    .word EXTI15_10_IRQHandler      /* EXTI lines 10-15 */
    .word RTC_Alarm_IRQHandler      /* RTC alarm through EXTI */
    .word OTG_FS_WKUP_IRQHandler    /* USB OTG FS wakeup */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word DMA1_Stream7_IRQHandler   /* DMA1 stream 7 */
    .word 0                         /* Reserved */
    .word SDIO_IRQHandler           /* SDIO */
    .word TIM5_IRQHandler           /* TIM5 */
    .word SPI3_IRQHandler           /* SPI3 */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word DMA2_Stream0_IRQHandler   /* DMA2 stream 0 */
    .word DMA2_Stream1_IRQHandler   /* DMA2 stream 1 */
    .word DMA2_Stream2_IRQHandler   /* DMA2 stream 2 */
    .word DMA2_Stream3_IRQHandler   /* DMA2 stream 3 */
    .word DMA2_Stream4_IRQHandler   /* DMA2 stream 4 */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word OTG_FS_IRQHandler         /* USB OTG FS */
    .word DMA2_Stream5_IRQHandler   /* DMA2 stream 5 */
    .word DMA2_Stream6_IRQHandler   /* DMA2 stream 6 */
    .word DMA2_Stream7_IRQHandler   /* DMA2 stream 7 */
    .word USART6_IRQHandler         /* USART6 */
    .word I2C3_EV_IRQHandler        /* I2C3 event */
    .word I2C3_ER_IRQHandler        /* I2C3 error */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word FPU_IRQHandler            /* FPU */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word SPI4_IRQHandler           /* SPI4 */

    .size g_pfnVectors, .-g_pfnVectors

/* ----------------------------------------------------------------------------
 * Reset handler
 * -------------------------------------------------------------------------- */

    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function

Reset_Handler:
    /* Set stack pointer */
    ldr sp, =_estack

    /* Enable FPU (Cortex-M4 has FPU) */
    /* Set CP10 and CP11 to full access in CPACR */
    ldr r0, =0xE000ED88     /* CPACR address */
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20) /* Enable CP10 and CP11 */
    str r1, [r0]
    dsb                      /* Data sync barrier */
    isb                      /* Instruction sync barrier */

    /* Copy .data section from Flash to RAM */
    ldr r0, =_sidata        /* Source: .data in flash */
    ldr r1, =_sdata         /* Dest start: .data in RAM */
    ldr r2, =_edata         /* Dest end */
copy_data:
    cmp r1, r2
    bhs copy_data_done      /* Unsigned comparison */
    ldr r3, [r0]
    str r3, [r1]
    adds r0, r0, #4
    adds r1, r1, #4
    b copy_data
copy_data_done:

    /* Zero .bss section */
    ldr r0, =_sbss          /* Start of BSS */
    ldr r1, =_ebss          /* End of BSS */
    movs r2, #0
zero_bss:
    cmp r0, r1
    bhs zero_bss_done       /* Unsigned comparison */
    str r2, [r0]
    adds r0, r0, #4
    b zero_bss
zero_bss_done:

    /* Re-set stack pointer to ensure it's valid and aligned */
    ldr sp, =_estack

    /* Call main */
    bl main

    /* If main returns, loop forever */
    b .

    .size Reset_Handler, .-Reset_Handler

/* ----------------------------------------------------------------------------
 * Default interrupt handlers (weak aliases to Default_Handler)
 * -------------------------------------------------------------------------- */

    .section .text.Default_Handler, "ax", %progbits

Default_Handler:
Infinite_Loop:
    b Infinite_Loop

    .size Default_Handler, .-Default_Handler

/* Special WWDG handler - clears interrupt and returns instead of looping */
    .section .text.WWDG_IRQHandler, "ax", %progbits
    .global WWDG_IRQHandler
    .type WWDG_IRQHandler, %function

WWDG_IRQHandler:
    /* Clear EWIF flag in WWDG_SR (0x40002C08) by writing 0 */
    ldr r0, =0x40002C08
    movs r1, #0
    str r1, [r0]
    /* Refresh watchdog counter to maximum (0x7F) to prevent reset */
    /* WWDG_CR = 0x40002C00, write 0xFF (WDGA=1, T=0x7F) */
    ldr r0, =0x40002C00
    movs r1, #0xFF
    str r1, [r0]
    bx lr

    .size WWDG_IRQHandler, .-WWDG_IRQHandler

/* Macro for weak aliases */
    .macro def_irq_handler handler_name
    .weak \handler_name
    .thumb_set \handler_name, Default_Handler
    .endm

/* System exceptions */
    def_irq_handler NMI_Handler
    def_irq_handler HardFault_Handler
    def_irq_handler MemManage_Handler
    def_irq_handler BusFault_Handler
    def_irq_handler UsageFault_Handler
    def_irq_handler SVC_Handler
    def_irq_handler DebugMon_Handler
    def_irq_handler PendSV_Handler
    def_irq_handler SysTick_Handler

/* External interrupts */
    /* WWDG_IRQHandler defined above - not a weak alias */
    def_irq_handler PVD_IRQHandler
    def_irq_handler TAMP_STAMP_IRQHandler
    def_irq_handler RTC_WKUP_IRQHandler
    def_irq_handler FLASH_IRQHandler
    def_irq_handler RCC_IRQHandler
    def_irq_handler EXTI0_IRQHandler
    def_irq_handler EXTI1_IRQHandler
    def_irq_handler EXTI2_IRQHandler
    def_irq_handler EXTI3_IRQHandler
    def_irq_handler EXTI4_IRQHandler
    def_irq_handler DMA1_Stream0_IRQHandler
    def_irq_handler DMA1_Stream1_IRQHandler
    def_irq_handler DMA1_Stream2_IRQHandler
    def_irq_handler DMA1_Stream3_IRQHandler
    def_irq_handler DMA1_Stream4_IRQHandler
    def_irq_handler DMA1_Stream5_IRQHandler
    def_irq_handler DMA1_Stream6_IRQHandler
    def_irq_handler ADC_IRQHandler
    def_irq_handler EXTI9_5_IRQHandler
    def_irq_handler TIM1_BRK_TIM9_IRQHandler
    def_irq_handler TIM1_UP_TIM10_IRQHandler
    def_irq_handler TIM1_TRG_COM_TIM11_IRQHandler
    def_irq_handler TIM1_CC_IRQHandler
    def_irq_handler TIM2_IRQHandler
    def_irq_handler TIM3_IRQHandler
    def_irq_handler TIM4_IRQHandler
    def_irq_handler I2C1_EV_IRQHandler
    def_irq_handler I2C1_ER_IRQHandler
    def_irq_handler I2C2_EV_IRQHandler
    def_irq_handler I2C2_ER_IRQHandler
    def_irq_handler SPI1_IRQHandler
    def_irq_handler SPI2_IRQHandler
    def_irq_handler USART1_IRQHandler
    def_irq_handler USART2_IRQHandler
    def_irq_handler EXTI15_10_IRQHandler
    def_irq_handler RTC_Alarm_IRQHandler
    def_irq_handler OTG_FS_WKUP_IRQHandler
    def_irq_handler DMA1_Stream7_IRQHandler
    def_irq_handler SDIO_IRQHandler
    def_irq_handler TIM5_IRQHandler
    def_irq_handler SPI3_IRQHandler
    def_irq_handler DMA2_Stream0_IRQHandler
    def_irq_handler DMA2_Stream1_IRQHandler
    def_irq_handler DMA2_Stream2_IRQHandler
    def_irq_handler DMA2_Stream3_IRQHandler
    def_irq_handler DMA2_Stream4_IRQHandler
    def_irq_handler OTG_FS_IRQHandler
    def_irq_handler DMA2_Stream5_IRQHandler
    def_irq_handler DMA2_Stream6_IRQHandler
    def_irq_handler DMA2_Stream7_IRQHandler
    def_irq_handler USART6_IRQHandler
    def_irq_handler I2C3_EV_IRQHandler
    def_irq_handler I2C3_ER_IRQHandler
    def_irq_handler FPU_IRQHandler
    def_irq_handler SPI4_IRQHandler

    .end

/**
 * STM32F405RG Startup Code
 *
 * - Sets up stack pointer
 * - Copies .data section from Flash to RAM
 * - Zeros .bss section
 * - Copies .ramfunc section from Flash to RAM
 * - Calls SystemInit() (must be provided by application)
 * - Calls main()
 * - Defines vector table with all STM32F405 interrupts
 */

    .syntax unified
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .thumb

/* External symbols */
.global g_pfnVectors
.global Default_Handler

/* Start address for stack (from linker script) */
.word _estack

/* Start/end addresses for init sections (from linker script) */
.word _sidata
.word _sdata
.word _edata
.word _sbss
.word _ebss
.word _siramfunc
.word _sramfunc
.word _eramfunc

/**
 * Reset Handler - Entry point after reset
 */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* Set stack pointer */
    ldr r0, =_estack
    mov sp, r0

    /* Enable FPU (CP10, CP11 full access) */
    ldr r0, =0xE000ED88
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20)
    str r1, [r0]
    dsb
    isb

    /* Copy .data section from Flash to RAM */
    ldr r0, =_sdata      /* Destination start */
    ldr r1, =_edata      /* Destination end */
    ldr r2, =_sidata     /* Source start */
    b .L_copy_data_check

.L_copy_data_loop:
    ldr r3, [r2], #4
    str r3, [r0], #4

.L_copy_data_check:
    cmp r0, r1
    blo .L_copy_data_loop

    /* Zero .bss section */
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
    b .L_zero_bss_check

.L_zero_bss_loop:
    str r2, [r0], #4

.L_zero_bss_check:
    cmp r0, r1
    blo .L_zero_bss_loop

    /* Copy .ramfunc section from Flash to RAM */
    ldr r0, =_sramfunc
    ldr r1, =_eramfunc
    ldr r2, =_siramfunc
    b .L_copy_ramfunc_check

.L_copy_ramfunc_loop:
    ldr r3, [r2], #4
    str r3, [r0], #4

.L_copy_ramfunc_check:
    cmp r0, r1
    blo .L_copy_ramfunc_loop

    /* Call SystemInit() - must be provided by system_stm32f4xx.c */
    bl SystemInit

    /* Call static constructors */
    bl __libc_init_array

    /* Call main() */
    bl main

    /* If main returns, loop forever */
.L_loop_forever:
    b .L_loop_forever

.size Reset_Handler, .-Reset_Handler

/**
 * Default Handler - Infinite loop for unhandled interrupts
 */
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
.L_default_loop:
    b .L_default_loop
.size Default_Handler, .-Default_Handler

/**
 * Vector Table
 */
    .section .isr_vector,"a",%progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word _estack                   /* 0x0000: Initial stack pointer */
    .word Reset_Handler             /* 0x0004: Reset */
    .word NMI_Handler               /* 0x0008: NMI */
    .word HardFault_Handler         /* 0x000C: Hard fault */
    .word MemManage_Handler         /* 0x0010: Memory management fault */
    .word BusFault_Handler          /* 0x0014: Bus fault */
    .word UsageFault_Handler        /* 0x0018: Usage fault */
    .word 0                         /* 0x001C: Reserved */
    .word 0                         /* 0x0020: Reserved */
    .word 0                         /* 0x0024: Reserved */
    .word 0                         /* 0x0028: Reserved */
    .word SVC_Handler               /* 0x002C: SVCall */
    .word DebugMon_Handler          /* 0x0030: Debug monitor */
    .word 0                         /* 0x0034: Reserved */
    .word PendSV_Handler            /* 0x0038: PendSV */
    .word SysTick_Handler           /* 0x003C: SysTick */

    /* External interrupts (STM32F405 specific) */
    .word WWDG_IRQHandler           /* 0x0040: Window Watchdog */
    .word PVD_IRQHandler            /* 0x0044: PVD through EXTI */
    .word TAMP_STAMP_IRQHandler     /* 0x0048: Tamper and TimeStamp */
    .word RTC_WKUP_IRQHandler       /* 0x004C: RTC Wakeup */
    .word FLASH_IRQHandler          /* 0x0050: Flash */
    .word RCC_IRQHandler            /* 0x0054: RCC */
    .word EXTI0_IRQHandler          /* 0x0058: EXTI Line 0 */
    .word EXTI1_IRQHandler          /* 0x005C: EXTI Line 1 */
    .word EXTI2_IRQHandler          /* 0x0060: EXTI Line 2 */
    .word EXTI3_IRQHandler          /* 0x0064: EXTI Line 3 */
    .word EXTI4_IRQHandler          /* 0x0068: EXTI Line 4 */
    .word DMA1_Stream0_IRQHandler   /* 0x006C: DMA1 Stream 0 */
    .word DMA1_Stream1_IRQHandler   /* 0x0070: DMA1 Stream 1 */
    .word DMA1_Stream2_IRQHandler   /* 0x0074: DMA1 Stream 2 */
    .word DMA1_Stream3_IRQHandler   /* 0x0078: DMA1 Stream 3 */
    .word DMA1_Stream4_IRQHandler   /* 0x007C: DMA1 Stream 4 */
    .word DMA1_Stream5_IRQHandler   /* 0x0080: DMA1 Stream 5 */
    .word DMA1_Stream6_IRQHandler   /* 0x0084: DMA1 Stream 6 */
    .word ADC_IRQHandler            /* 0x0088: ADC1, ADC2, ADC3 */
    .word CAN1_TX_IRQHandler        /* 0x008C: CAN1 TX */
    .word CAN1_RX0_IRQHandler       /* 0x0090: CAN1 RX0 */
    .word CAN1_RX1_IRQHandler       /* 0x0094: CAN1 RX1 */
    .word CAN1_SCE_IRQHandler       /* 0x0098: CAN1 SCE */
    .word EXTI9_5_IRQHandler        /* 0x009C: EXTI Lines 5-9 */
    .word TIM1_BRK_TIM9_IRQHandler  /* 0x00A0: TIM1 Break and TIM9 */
    .word TIM1_UP_TIM10_IRQHandler  /* 0x00A4: TIM1 Update and TIM10 */
    .word TIM1_TRG_COM_TIM11_IRQHandler /* 0x00A8: TIM1 Trigger/Commutation and TIM11 */
    .word TIM1_CC_IRQHandler        /* 0x00AC: TIM1 Capture Compare */
    .word TIM2_IRQHandler           /* 0x00B0: TIM2 */
    .word TIM3_IRQHandler           /* 0x00B4: TIM3 */
    .word TIM4_IRQHandler           /* 0x00B8: TIM4 */
    .word I2C1_EV_IRQHandler        /* 0x00BC: I2C1 Event */
    .word I2C1_ER_IRQHandler        /* 0x00C0: I2C1 Error */
    .word I2C2_EV_IRQHandler        /* 0x00C4: I2C2 Event */
    .word I2C2_ER_IRQHandler        /* 0x00C8: I2C2 Error */
    .word SPI1_IRQHandler           /* 0x00CC: SPI1 */
    .word SPI2_IRQHandler           /* 0x00D0: SPI2 */
    .word USART1_IRQHandler         /* 0x00D4: USART1 */
    .word USART2_IRQHandler         /* 0x00D8: USART2 */
    .word USART3_IRQHandler         /* 0x00DC: USART3 */
    .word EXTI15_10_IRQHandler      /* 0x00E0: EXTI Lines 10-15 */
    .word RTC_Alarm_IRQHandler      /* 0x00E4: RTC Alarm through EXTI */
    .word OTG_FS_WKUP_IRQHandler    /* 0x00E8: USB OTG FS Wakeup */
    .word TIM8_BRK_TIM12_IRQHandler /* 0x00EC: TIM8 Break and TIM12 */
    .word TIM8_UP_TIM13_IRQHandler  /* 0x00F0: TIM8 Update and TIM13 */
    .word TIM8_TRG_COM_TIM14_IRQHandler /* 0x00F4: TIM8 Trigger/Commutation and TIM14 */
    .word TIM8_CC_IRQHandler        /* 0x00F8: TIM8 Capture Compare */
    .word DMA1_Stream7_IRQHandler   /* 0x00FC: DMA1 Stream 7 */
    .word FSMC_IRQHandler           /* 0x0100: FSMC */
    .word SDIO_IRQHandler           /* 0x0104: SDIO */
    .word TIM5_IRQHandler           /* 0x0108: TIM5 */
    .word SPI3_IRQHandler           /* 0x010C: SPI3 */
    .word UART4_IRQHandler          /* 0x0110: UART4 */
    .word UART5_IRQHandler          /* 0x0114: UART5 */
    .word TIM6_DAC_IRQHandler       /* 0x0118: TIM6 and DAC underrun */
    .word TIM7_IRQHandler           /* 0x011C: TIM7 */
    .word DMA2_Stream0_IRQHandler   /* 0x0120: DMA2 Stream 0 */
    .word DMA2_Stream1_IRQHandler   /* 0x0124: DMA2 Stream 1 */
    .word DMA2_Stream2_IRQHandler   /* 0x0128: DMA2 Stream 2 */
    .word DMA2_Stream3_IRQHandler   /* 0x012C: DMA2 Stream 3 */
    .word DMA2_Stream4_IRQHandler   /* 0x0130: DMA2 Stream 4 */
    .word 0                         /* 0x0134: Reserved */
    .word 0                         /* 0x0138: Reserved */
    .word CAN2_TX_IRQHandler        /* 0x013C: CAN2 TX */
    .word CAN2_RX0_IRQHandler       /* 0x0140: CAN2 RX0 */
    .word CAN2_RX1_IRQHandler       /* 0x0144: CAN2 RX1 */
    .word CAN2_SCE_IRQHandler       /* 0x0148: CAN2 SCE */
    .word OTG_FS_IRQHandler         /* 0x014C: USB OTG FS */
    .word DMA2_Stream5_IRQHandler   /* 0x0150: DMA2 Stream 5 */
    .word DMA2_Stream6_IRQHandler   /* 0x0154: DMA2 Stream 6 */
    .word DMA2_Stream7_IRQHandler   /* 0x0158: DMA2 Stream 7 */
    .word USART6_IRQHandler         /* 0x015C: USART6 */
    .word I2C3_EV_IRQHandler        /* 0x0160: I2C3 Event */
    .word I2C3_ER_IRQHandler        /* 0x0164: I2C3 Error */
    .word OTG_HS_EP1_OUT_IRQHandler /* 0x0168: USB OTG HS EP1 Out */
    .word OTG_HS_EP1_IN_IRQHandler  /* 0x016C: USB OTG HS EP1 In */
    .word OTG_HS_WKUP_IRQHandler    /* 0x0170: USB OTG HS Wakeup */
    .word OTG_HS_IRQHandler         /* 0x0174: USB OTG HS */
    .word 0                         /* 0x0178: Reserved */
    .word 0                         /* 0x017C: Reserved */
    .word HASH_RNG_IRQHandler       /* 0x0180: Hash and RNG */
    .word FPU_IRQHandler            /* 0x0184: FPU */

/**
 * Weak aliases for default handlers
 */
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

    .weak SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    .weak WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler, Default_Handler

    .weak PVD_IRQHandler
    .thumb_set PVD_IRQHandler, Default_Handler

    .weak TAMP_STAMP_IRQHandler
    .thumb_set TAMP_STAMP_IRQHandler, Default_Handler

    .weak RTC_WKUP_IRQHandler
    .thumb_set RTC_WKUP_IRQHandler, Default_Handler

    .weak FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler

    .weak RCC_IRQHandler
    .thumb_set RCC_IRQHandler, Default_Handler

    .weak EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler, Default_Handler

    .weak EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler, Default_Handler

    .weak EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler, Default_Handler

    .weak EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler, Default_Handler

    .weak EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler, Default_Handler

    .weak DMA1_Stream0_IRQHandler
    .thumb_set DMA1_Stream0_IRQHandler, Default_Handler

    .weak DMA1_Stream1_IRQHandler
    .thumb_set DMA1_Stream1_IRQHandler, Default_Handler

    .weak DMA1_Stream2_IRQHandler
    .thumb_set DMA1_Stream2_IRQHandler, Default_Handler

    .weak DMA1_Stream3_IRQHandler
    .thumb_set DMA1_Stream3_IRQHandler, Default_Handler

    .weak DMA1_Stream4_IRQHandler
    .thumb_set DMA1_Stream4_IRQHandler, Default_Handler

    .weak DMA1_Stream5_IRQHandler
    .thumb_set DMA1_Stream5_IRQHandler, Default_Handler

    .weak DMA1_Stream6_IRQHandler
    .thumb_set DMA1_Stream6_IRQHandler, Default_Handler

    .weak ADC_IRQHandler
    .thumb_set ADC_IRQHandler, Default_Handler

    .weak CAN1_TX_IRQHandler
    .thumb_set CAN1_TX_IRQHandler, Default_Handler

    .weak CAN1_RX0_IRQHandler
    .thumb_set CAN1_RX0_IRQHandler, Default_Handler

    .weak CAN1_RX1_IRQHandler
    .thumb_set CAN1_RX1_IRQHandler, Default_Handler

    .weak CAN1_SCE_IRQHandler
    .thumb_set CAN1_SCE_IRQHandler, Default_Handler

    .weak EXTI9_5_IRQHandler
    .thumb_set EXTI9_5_IRQHandler, Default_Handler

    .weak TIM1_BRK_TIM9_IRQHandler
    .thumb_set TIM1_BRK_TIM9_IRQHandler, Default_Handler

    .weak TIM1_UP_TIM10_IRQHandler
    .thumb_set TIM1_UP_TIM10_IRQHandler, Default_Handler

    .weak TIM1_TRG_COM_TIM11_IRQHandler
    .thumb_set TIM1_TRG_COM_TIM11_IRQHandler, Default_Handler

    .weak TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler

    .weak TIM2_IRQHandler
    .thumb_set TIM2_IRQHandler, Default_Handler

    .weak TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler, Default_Handler

    .weak TIM4_IRQHandler
    .thumb_set TIM4_IRQHandler, Default_Handler

    .weak I2C1_EV_IRQHandler
    .thumb_set I2C1_EV_IRQHandler, Default_Handler

    .weak I2C1_ER_IRQHandler
    .thumb_set I2C1_ER_IRQHandler, Default_Handler

    .weak I2C2_EV_IRQHandler
    .thumb_set I2C2_EV_IRQHandler, Default_Handler

    .weak I2C2_ER_IRQHandler
    .thumb_set I2C2_ER_IRQHandler, Default_Handler

    .weak SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler

    .weak SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler, Default_Handler

    .weak USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler

    .weak USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler

    .weak USART3_IRQHandler
    .thumb_set USART3_IRQHandler, Default_Handler

    .weak EXTI15_10_IRQHandler
    .thumb_set EXTI15_10_IRQHandler, Default_Handler

    .weak RTC_Alarm_IRQHandler
    .thumb_set RTC_Alarm_IRQHandler, Default_Handler

    .weak OTG_FS_WKUP_IRQHandler
    .thumb_set OTG_FS_WKUP_IRQHandler, Default_Handler

    .weak TIM8_BRK_TIM12_IRQHandler
    .thumb_set TIM8_BRK_TIM12_IRQHandler, Default_Handler

    .weak TIM8_UP_TIM13_IRQHandler
    .thumb_set TIM8_UP_TIM13_IRQHandler, Default_Handler

    .weak TIM8_TRG_COM_TIM14_IRQHandler
    .thumb_set TIM8_TRG_COM_TIM14_IRQHandler, Default_Handler

    .weak TIM8_CC_IRQHandler
    .thumb_set TIM8_CC_IRQHandler, Default_Handler

    .weak DMA1_Stream7_IRQHandler
    .thumb_set DMA1_Stream7_IRQHandler, Default_Handler

    .weak FSMC_IRQHandler
    .thumb_set FSMC_IRQHandler, Default_Handler

    .weak SDIO_IRQHandler
    .thumb_set SDIO_IRQHandler, Default_Handler

    .weak TIM5_IRQHandler
    .thumb_set TIM5_IRQHandler, Default_Handler

    .weak SPI3_IRQHandler
    .thumb_set SPI3_IRQHandler, Default_Handler

    .weak UART4_IRQHandler
    .thumb_set UART4_IRQHandler, Default_Handler

    .weak UART5_IRQHandler
    .thumb_set UART5_IRQHandler, Default_Handler

    .weak TIM6_DAC_IRQHandler
    .thumb_set TIM6_DAC_IRQHandler, Default_Handler

    .weak TIM7_IRQHandler
    .thumb_set TIM7_IRQHandler, Default_Handler

    .weak DMA2_Stream0_IRQHandler
    .thumb_set DMA2_Stream0_IRQHandler, Default_Handler

    .weak DMA2_Stream1_IRQHandler
    .thumb_set DMA2_Stream1_IRQHandler, Default_Handler

    .weak DMA2_Stream2_IRQHandler
    .thumb_set DMA2_Stream2_IRQHandler, Default_Handler

    .weak DMA2_Stream3_IRQHandler
    .thumb_set DMA2_Stream3_IRQHandler, Default_Handler

    .weak DMA2_Stream4_IRQHandler
    .thumb_set DMA2_Stream4_IRQHandler, Default_Handler

    .weak CAN2_TX_IRQHandler
    .thumb_set CAN2_TX_IRQHandler, Default_Handler

    .weak CAN2_RX0_IRQHandler
    .thumb_set CAN2_RX0_IRQHandler, Default_Handler

    .weak CAN2_RX1_IRQHandler
    .thumb_set CAN2_RX1_IRQHandler, Default_Handler

    .weak CAN2_SCE_IRQHandler
    .thumb_set CAN2_SCE_IRQHandler, Default_Handler

    .weak OTG_FS_IRQHandler
    .thumb_set OTG_FS_IRQHandler, Default_Handler

    .weak DMA2_Stream5_IRQHandler
    .thumb_set DMA2_Stream5_IRQHandler, Default_Handler

    .weak DMA2_Stream6_IRQHandler
    .thumb_set DMA2_Stream6_IRQHandler, Default_Handler

    .weak DMA2_Stream7_IRQHandler
    .thumb_set DMA2_Stream7_IRQHandler, Default_Handler

    .weak USART6_IRQHandler
    .thumb_set USART6_IRQHandler, Default_Handler

    .weak I2C3_EV_IRQHandler
    .thumb_set I2C3_EV_IRQHandler, Default_Handler

    .weak I2C3_ER_IRQHandler
    .thumb_set I2C3_ER_IRQHandler, Default_Handler

    .weak OTG_HS_EP1_OUT_IRQHandler
    .thumb_set OTG_HS_EP1_OUT_IRQHandler, Default_Handler

    .weak OTG_HS_EP1_IN_IRQHandler
    .thumb_set OTG_HS_EP1_IN_IRQHandler, Default_Handler

    .weak OTG_HS_WKUP_IRQHandler
    .thumb_set OTG_HS_WKUP_IRQHandler, Default_Handler

    .weak OTG_HS_IRQHandler
    .thumb_set OTG_HS_IRQHandler, Default_Handler

    .weak HASH_RNG_IRQHandler
    .thumb_set HASH_RNG_IRQHandler, Default_Handler

    .weak FPU_IRQHandler
    .thumb_set FPU_IRQHandler, Default_Handler

.end

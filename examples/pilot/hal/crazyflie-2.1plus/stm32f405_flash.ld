/* STM32F405RG Linker Script
 *
 * Memory layout for Crazyflie 2.1+:
 *   Flash: 1 MB (0x08000000 - 0x080FFFFF)
 *   SRAM:  128 KB (0x20000000 - 0x2001FFFF) - Main SRAM
 *   CCM:   64 KB (0x10000000 - 0x1000FFFF) - Core Coupled Memory
 *
 * Flash sector layout (STM32F405RG):
 *   Sector 0:    16 KB        (0x08000000 - 0x08003FFF) = Bootloader
 *   Sectors 1-3: 16 KB each   (0x08004000 - 0x0800FFFF) =  48 KB
 *   Sector 4:    64 KB        (0x08010000 - 0x0801FFFF) =  64 KB
 *   Sectors 5-7: 128 KB each  (0x08020000 - 0x0807FFFF) = 384 KB
 *   Sectors 8-11: 128 KB each (0x08080000 - 0x080FFFFF) = 512 KB (DATA)
 *
 * Bootloader: Sector 0 (16 KB) at 0x08000000
 * Firmware:   Sectors 1-7 (496 KB) at 0x08004000
 * Data:       Sector 8 = /log (128 KB flight log)
 *             Sectors 9-11 reserved for future use
 *
 * Note: CCM is only accessible by the CPU (no DMA access).
 *       Use CCM for stack and performance-critical data.
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K   /* Sectors 0-7 (no bootloader) */
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
    CCM (rwx)   : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Entry point */
ENTRY(Reset_Handler)

/* Stack and heap sizes */
_Min_Heap_Size = 0x2000;   /* 8 KB heap */
_Min_Stack_Size = 0x4000;  /* 16 KB stack (in CCM) */

SECTIONS
{
    /* Interrupt vector table */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* Program code */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Read-only data */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* ARM exception handling */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    /* Preinit and init arrays */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /* Used by startup code to initialize data */
    _sidata = LOADADDR(.data);

    /* Initialized data (copied from Flash to RAM) */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    /* Uninitialized data (zeroed at startup) */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    /* Heap (in main RAM) */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        PROVIDE(_heap_start = .);
        . = . + _Min_Heap_Size;
        . = ALIGN(8);
        PROVIDE(_heap_end = .);
    } > RAM

    /* Stack (in CCM for faster access) */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = .;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
        _estack = .;
    } > CCM

    /* RAM functions (copied to RAM for execution) */
    .ramfunc :
    {
        . = ALIGN(4);
        _sramfunc = .;
        *(.ramfunc)
        *(.ramfunc.*)
        . = ALIGN(4);
        _eramfunc = .;
    } > RAM AT > FLASH
    _siramfunc = LOADADDR(.ramfunc);

    /* Discard */
    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }

    /* ARM build attributes */
    .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* Provide stack pointer for startup */
PROVIDE(_stack = _estack);

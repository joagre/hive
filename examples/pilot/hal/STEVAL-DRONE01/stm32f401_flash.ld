/*
 * Linker script for STM32F401CEU6 (STEVAL-DRONE01)
 *
 * Memory:
 *   Flash: 512KB @ 0x08000000
 *   RAM:   96KB  @ 0x20000000
 *
 * Based on STM32CubeMX generated linker script.
 */

/* Entry point */
ENTRY(Reset_Handler)

/* Highest address of the stack (end of RAM) */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Generate error if heap + stack don't fit in RAM */
_Min_Heap_Size = 0x400;   /* 1KB heap */
_Min_Stack_Size = 0x800;  /* 2KB stack */

/* Memory regions */
/* STM32F401xB/xC has 64KB SRAM (not 96KB like STM32F401xD/xE) */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Section definitions */
SECTIONS
{
    /* ---------------------------------------------------------------------------
     * Interrupt vector table - must be at start of Flash
     * ------------------------------------------------------------------------- */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* ---------------------------------------------------------------------------
     * Code and read-only data
     * ------------------------------------------------------------------------- */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)          /* ARM/Thumb interworking */
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Constant data */
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

    /* Pre-initialization array (constructors called before main) */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    /* Initialization array (C++ constructors) */
    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    /* Finalization array (C++ destructors) */
    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /* Used by startup code to initialize data section */
    _sidata = LOADADDR(.data);

    /* ---------------------------------------------------------------------------
     * Initialized data - copied from Flash to RAM at startup
     * ------------------------------------------------------------------------- */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;         /* Start of data in RAM */
        *(.data)
        *(.data*)
        *(.RamFunc)         /* Functions executed from RAM */
        *(.RamFunc*)
        . = ALIGN(4);
        _edata = .;         /* End of data in RAM */
    } > RAM AT > FLASH

    /* ---------------------------------------------------------------------------
     * Uninitialized data (BSS) - zeroed at startup
     * ------------------------------------------------------------------------- */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;          /* Start of BSS */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;          /* End of BSS */
        __bss_end__ = _ebss;
    } > RAM

    /* ---------------------------------------------------------------------------
     * User heap - grows upward from end of BSS
     * ------------------------------------------------------------------------- */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM

    /* ---------------------------------------------------------------------------
     * Debug sections (not loaded to target)
     * ------------------------------------------------------------------------- */

    /* Remove information from standard libraries */
    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }

    /* DWARF debug sections */
    .debug_info     0 : { *(.debug_info) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    .debug_ranges   0 : { *(.debug_ranges) }

    /* ARM attributes */
    .ARM.attributes 0 : { *(.ARM.attributes) }
}

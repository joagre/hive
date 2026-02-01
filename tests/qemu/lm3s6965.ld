/*
 * Linker script for LM3S6965 (QEMU lm3s6965evb target)
 *
 * Memory:
 *   Flash: 256KB @ 0x00000000
 *   RAM:   64KB  @ 0x20000000
 *
 * Used for QEMU-based testing of the hive runtime.
 */

ENTRY(Reset_Handler)

/* End of stack (top of RAM) */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Memory regions */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS
{
    /* Vector table - must be at start of Flash */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* Code and read-only data */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* Location of .data in Flash (for startup copy) */
    _sidata = LOADADDR(.data);

    /* Initialized data - copied from Flash to RAM at startup */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    /* Uninitialized data (BSS) - zeroed at startup */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Provide end symbol for newlib's sbrk */
    PROVIDE(end = _ebss);
    PROVIDE(_end = _ebss);

    /* Heap and stack space check */
    ._check_stack :
    {
        . = ALIGN(8);
        . = . + 0x1000;  /* 4KB minimum stack */
        . = ALIGN(8);
    } > RAM
}

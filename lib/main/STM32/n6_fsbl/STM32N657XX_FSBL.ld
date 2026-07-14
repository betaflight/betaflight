/*
 * Linker script for the Betaflight N6570-DK FSBL stub.
 *
 * Boot ROM loads the signed FSBL into AXISRAM2 secure starting at 0x34180400
 * (the 0x400 offset matches `STM32_SigningTool_CLI -align`). All sections
 * live in this single region — the boot ROM populates .data alongside
 * .text during load, so .data LMA == VMA and the startup .data copy is a
 * no-op. 256 KiB sits within the system DFU FSBL slot's payload cap and
 * well below AXISRAM2's 1 MiB limit.
 */

ENTRY(Reset_Handler)

_Min_Stack_Size = 0x1000;

MEMORY
{
    RAM (rwx) : ORIGIN = 0x34180400, LENGTH = 256K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_sstack = _estack - _Min_Stack_Size;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > RAM

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text.*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        . = ALIGN(4);
    } > RAM

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
    } > RAM

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > RAM

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > RAM

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = .;
    } > RAM

    /* Reserve stack at the top of RAM. Linker fails the build if we
     * overflow into .bss. */
    ._stack_reservation :
    {
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM

    /DISCARD/ :
    {
        *(.ARM.attributes)
        *(.comment)
    }
}

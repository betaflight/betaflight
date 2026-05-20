/*
 * Linker script for the Betaflight N6 OpenBootloader (OBL).
 *
 * Boot ROM behaviour (BOOT0=USER, BOOT1=2):
 *   1. Reads signed FSBL header from XSPI 0x70000000
 *   2. Verifies + copies payload into AXISRAM2 secure starting at
 *      0x34180400 (the 0x400 offset matches `STM32_SigningTool_CLI -align`)
 *   3. Branches to Reset_Handler at the loaded entry point
 *
 * The boot ROM caps @FSBL DFU staging at ~485 KiB empirically — the
 * destination region from 0x34180400 to end of AXISRAM2 secure is 521 KiB
 * but the boot ROM reserves ~36 KiB of scratch. The 480 KiB region below
 * is the safe budget for OBL + flash driver + DFU stack + self-installer.
 */

ENTRY(Reset_Handler)

_Min_Stack_Size = 0x2000;   /* 8 KiB — DFU upload buffer + USB ISR + libc */

MEMORY
{
    /* Loaded region: signed FSBL slot in AXISRAM2 secure. Boot ROM lands
     * the .stm32 payload here. */
    RAM (rwx) : ORIGIN = 0x34180400, LENGTH = 480K
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

    /* Embedded "self" payload: the OBL writes the bytes of its own loaded
     * image to nor0 0x0 during recovery self-install. The payload starts
     * at the FSBL header (0x34180000 in AXISRAM, which is 0x34180400 minus
     * the 0x400 alignment header — but boot ROM only stages from 0x34180400
     * onwards, so the "self" we replicate is the post-header content). The
     * signed header has to be re-applied externally — at install time we
     * read it from nor0 too via memory-mapped XSPI before erase. */
    _self_load_start = ORIGIN(RAM);
    _self_load_end   = .;     /* updated by linker after .data */

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
        /* End-of-image marker for newlib's _sbrk. We don't use malloc,
         * but linking pulls _sbrk in regardless; satisfy it with a
         * symbol that points past .bss so any incidental allocation
         * lands in unused heap space below the stack reservation. */
        PROVIDE(end = .);
        PROVIDE(_end = .);
    } > RAM

    /* Stack reservation at the top of RAM. Linker fails the build if BSS
     * overflows into the stack region. */
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

/*
 * Minimal linker script for ESP32-C5 (RISC-V)
 *
 * Skeleton for build-target bring-up. No peripheral mappings; per-driver
 * peripheral section attachment comes with the C5 driver port.
 *
 * ESP32-C5 memory map (from soc.h):
 *   HP SRAM:  0x40800000 - 0x40860000 (384 KB, unified IRAM/DRAM)
 *   Flash:    0x42000000 - mapped via MMU
 */

MEMORY
{
    iram_seg (RWX) :   org = 0x40800000, len = 384K
    dram_seg (RW)  :   org = 0x40800000, len = 384K
    flash_seg (RX) :   org = 0x42000000, len = 4M
}

ENTRY(main)

_estack = ORIGIN(dram_seg) + LENGTH(dram_seg);
_Min_Stack_Size = 0x800;

SECTIONS
{
    .iram.text :
    {
        _iram_text_start = ABSOLUTE(.);
        *(.iram1.literal .iram1.*)
        *(.iram.text.literal .iram.text .iram.text.*)
        _iram_text_end = ABSOLUTE(.);
    } > iram_seg

    .flash.text :
    {
        _stext = .;
        *(.literal .literal.* .text .text.*)
        _etext = .;
    } > flash_seg

    .pg_registry :
    {
        PROVIDE_HIDDEN (__pg_registry_start = .);
        KEEP (*(.pg_registry))
        KEEP (*(SORT(.pg_registry.*)))
        PROVIDE_HIDDEN (__pg_registry_end = .);
    } > flash_seg

    .pg_resetdata :
    {
        PROVIDE_HIDDEN (__pg_resetdata_start = .);
        KEEP (*(.pg_resetdata))
        PROVIDE_HIDDEN (__pg_resetdata_end = .);
    } > flash_seg

    .flash.rodata :
    {
        _srodata = .;
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)
        _erodata = .;
    } > flash_seg

    .data :
    {
        _sdata = .;
        *(.data .data.*)
        *(.sdata .sdata.*)
        _edata = .;
    } > dram_seg AT > flash_seg

    .bss (NOLOAD) :
    {
        _sbss = .;
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        _ebss = .;
    } > dram_seg

    .noinit (NOLOAD) :
    {
        *(.noinit .noinit.*)
        *(.uninitialized_data.persistent)
    } > dram_seg

    _end = .;

    _heap_start = .;
    _heap_end = ORIGIN(dram_seg) + LENGTH(dram_seg);

    __config_start = ORIGIN(flash_seg) + LENGTH(flash_seg) - 64K;
    __config_end = ORIGIN(flash_seg) + LENGTH(flash_seg);

    /DISCARD/ :
    {
        *(.comment)
        *(.note.*)
        *(.eh_frame*)
    }
}

/*
 * Minimal linker script for ESP32-S3
 *
 * This is a placeholder for initial compilation.
 * The real linker script will be derived from ESP-IDF once
 * the SDK submodule is integrated.
 *
 * ESP32-S3 memory map:
 *   IRAM:  0x40370000 - 0x403BFFFF (320 KB instruction RAM)
 *   DRAM:  0x3FC88000 - 0x3FCEFFFF (416 KB data RAM)
 *   Flash: 0x42000000 - mapped via MMU (up to 16 MB)
 */

MEMORY
{
    iram_seg (RWX) :   org = 0x40370000, len = 320K
    dram_seg (RW)  :   org = 0x3FC88000, len = 416K
    flash_seg (RX) :   org = 0x42000000, len = 8M
}

/* Default entry point */
ENTRY(main)

/* Stack at end of DRAM */
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
        *(.literal .literal.*)
        *(.text .text.*)
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

    /* Provide heap and stack regions */
    _heap_start = .;
    _heap_end = ORIGIN(dram_seg) + LENGTH(dram_seg);

    /* Config flash storage - last 64KB of flash */
    __config_start = ORIGIN(flash_seg) + LENGTH(flash_seg) - 64K;
    __config_end = ORIGIN(flash_seg) + LENGTH(flash_seg);

    /DISCARD/ :
    {
        *(.comment)
        *(.note.*)
        *(.eh_frame*)
    }
}

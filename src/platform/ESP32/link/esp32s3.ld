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
ENTRY(app_main)

SECTIONS
{
    .iram.text :
    {
        _iram_text_start = ABSOLUTE(.);
        *(.iram1 .iram1.*)
        *(.iram.text .iram.text.*)
        _iram_text_end = ABSOLUTE(.);
    } > iram_seg

    .flash.text :
    {
        _stext = .;
        *(.text .text.*)
        *(.literal .literal.*)
        _etext = .;
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

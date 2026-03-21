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

    /* Peripheral register regions (memory-mapped I/O) */
    periph_uart0_seg (RW) :   org = 0x60000000, len = 0x1000
    periph_gpio_seg (RW) :    org = 0x60004000, len = 0x1000
    periph_uart1_seg (RW) :   org = 0x60010000, len = 0x1000
    periph_i2c0_seg (RW) :    org = 0x60013000, len = 0x1000
    periph_rmt_seg (RW) :     org = 0x60016000, len = 0x800
    periph_rmtmem_seg (RW) :  org = 0x60016800, len = 0x800
    periph_ledc_seg (RW) :    org = 0x60019000, len = 0x1000
    periph_systimer_seg (RW): org = 0x60023000, len = 0x1000
    periph_spi2_seg (RW) :    org = 0x60024000, len = 0x1000
    periph_spi3_seg (RW) :    org = 0x60025000, len = 0x1000
    periph_i2c1_seg (RW) :    org = 0x60027000, len = 0x1000
    periph_uart2_seg (RW) :   org = 0x6002E000, len = 0x1000
    periph_usb_serial_seg (RW): org = 0x60038000, len = 0x1000
    periph_system_seg (RW) :  org = 0x600C0000, len = 0x1000
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

    /* Provide heap and stack regions */
    _heap_start = .;
    _heap_end = ORIGIN(dram_seg) + LENGTH(dram_seg);

    /* Config flash storage - last 64KB of flash */
    __config_start = ORIGIN(flash_seg) + LENGTH(flash_seg) - 64K;
    __config_end = ORIGIN(flash_seg) + LENGTH(flash_seg);

    /* Peripheral register mappings at fixed hardware addresses */
    .peripheral_uart0 (NOLOAD) :    { *(.peripheral_uart0) }   > periph_uart0_seg
    .peripheral_gpio (NOLOAD) :     { *(.peripheral_gpio) }    > periph_gpio_seg
    .peripheral_uart1 (NOLOAD) :    { *(.peripheral_uart1) }   > periph_uart1_seg
    .peripheral_i2c0 (NOLOAD) :     { *(.peripheral_i2c0) }    > periph_i2c0_seg
    .peripheral_rmt (NOLOAD) :      { *(.peripheral_rmt) }     > periph_rmt_seg
    .peripheral_rmtmem (NOLOAD) :   { *(.peripheral_rmtmem) }  > periph_rmtmem_seg
    .peripheral_ledc (NOLOAD) :     { *(.peripheral_ledc) }    > periph_ledc_seg
    .peripheral_systimer (NOLOAD) : { *(.peripheral_systimer) } > periph_systimer_seg
    .peripheral_spi2 (NOLOAD) :     { *(.peripheral_spi2) }    > periph_spi2_seg
    .peripheral_spi3 (NOLOAD) :     { *(.peripheral_spi3) }    > periph_spi3_seg
    .peripheral_i2c1 (NOLOAD) :     { *(.peripheral_i2c1) }    > periph_i2c1_seg
    .peripheral_uart2 (NOLOAD) :    { *(.peripheral_uart2) }   > periph_uart2_seg
    .peripheral_usb_serial (NOLOAD): { *(.peripheral_usb_serial) } > periph_usb_serial_seg
    .peripheral_system (NOLOAD) :   { *(.peripheral_system) }  > periph_system_seg

    /DISCARD/ :
    {
        *(.comment)
        *(.note.*)
        *(.eh_frame*)
    }
}

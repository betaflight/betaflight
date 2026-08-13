/*
 * Linker script for ESP32-S3, booted by the ESP-IDF second-stage bootloader.
 *
 * The image is produced with `esptool elf2image` and flashed at the factory
 * app offset. The bootloader maps the flash-resident segments through the MMU
 * and copies the RAM segments before jumping to the entry point:
 *
 *   .flash.text   -> IROM window 0x42000000  (execute-in-place code/literals)
 *   .flash.rodata -> DROM window 0x3C000000  (read-only data, byte accessible)
 *   .data/.bss    -> DRAM 0x3FC90000          (.data loaded by the bootloader,
 *                                              .bss zeroed by the startup stub)
 *
 * Read-only data MUST live in the DROM window: the IROM/IBUS window only allows
 * 32-bit aligned word accesses, so byte/halfword reads of rodata (strings, etc.)
 * from there fault. The 0x20 offset on the flash windows leaves room for the
 * image + first segment headers so the MMU page mapping lines up (matches IDF).
 */

MEMORY
{
    /* 0x40374000..0x40378000 is the instruction-only startup bank.  Reserve
     * another measured 32 KB from the following dual-port SRAM for the PID,
     * filtering and mixer hot paths.  It aliases DRAM
     * 0x3FC88000..0x3FC90000, so .data/.bss must start after that range. */
    iram_seg (RWX) :   org = 0x40374000, len = 0xC000
    /* End at 0x3FCEF000, NOT 0x3FCF0000 (SOC_DIRAM_DRAM_HIGH): the ROM keeps live
     * data structures in the top of DRAM - the SPI-flash chip descriptor
     * (g_rom_flashchip, via rom_spiflash_legacy_data @0x3FCEFFE4) and friends sit at
     * 0x3FCEF81C..0x3FCEFFFC. _estack is the top of this region and the main stack
     * grows down from it, so reaching 0x3FCF0000 would let the stack overwrite those
     * ROM globals - after which esp_rom_spiflash_* read a garbage chip descriptor and
     * hang in wait_idle (and esp_rom_spiflash_attach faults). Reserve the top 4 KB. */
    dram_seg (RW)  :   org = 0x3FC90000, len = 0x5F000
    irom_seg (RX)  :   org = 0x42000020, len = 0x800000 - 0x20
    drom_seg (R)   :   org = 0x3C000020, len = 0x800000 - 0x20

    /* Peripheral register regions (memory-mapped I/O) */
    periph_uart0_seg (RW) :   org = 0x60000000, len = 0x1000
    periph_gpio_seg (RW) :    org = 0x60004000, len = 0x1000
    periph_sens_seg (RW) :    org = 0x60008800, len = 0x800
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
    periph_gdma_seg (RW) :    org = 0x6003F000, len = 0x1000
    periph_system_seg (RW) :  org = 0x600C0000, len = 0x1000
}

/* Entry stub (see start_esp32s3.S); the bootloader jumps here. */
ENTRY(_start)

/* Stack at end of DRAM */
_estack = ORIGIN(dram_seg) + LENGTH(dram_seg);
_Min_Stack_Size = 0x800;

SECTIONS
{
    /* Startup must fit below 0x40378000.  ISR entry code is placed before the
     * wider hot path so future FAST_CODE growth can never displace it. */
    .iram.text :
    {
        _iram_text_start = ABSOLUTE(.);
        /* Entry stub first: the image entry point must be in IRAM. */
        KEEP (*(.iram1.start))
        KEEP (*(.iram1.core1))
        *(.iram1.literal .iram1.*)

        . = ALIGN(4);
        _iram_runtime_start = ABSOLUTE(.);
        _iram_isr_start = ABSOLUTE(.);
        *(.iram.isr.literal .iram.isr .iram.isr.*)
        _iram_isr_end = ABSOLUTE(.);

        . = ALIGN(4);
        _iram_hot_start = ABSOLUTE(.);
        *(.iram.hot.literal .iram.hot .iram.hot.*)
        /* Preserve compatibility with explicit ESP-IDF-style IRAM sections. */
        *(.iram.text.literal .iram.text .iram.text.*)
        _iram_hot_end = ABSOLUTE(.);
        _iram_text_end = ABSOLUTE(.);
    } > iram_seg

    ASSERT(_iram_runtime_start <= 0x40378000,
        "ESP32-S3 startup IRAM exceeds the dedicated 16 KB bank")
    ASSERT(_iram_text_end <= ORIGIN(iram_seg) + LENGTH(iram_seg) - 0x1000,
        "ESP32-S3 IRAM code exceeds the 44 KB budget (4 KB reserve)")

    /* Execute-in-place code in the IROM window. Defined before the DROM dummy
     * below because that dummy uses SIZEOF(.flash.text). */
    .flash.text :
    {
        _stext = .;
        *(.literal .literal.* .text .text.*)
        _etext = .;
    } > irom_seg

    /* The ESP32-S3 flash MMU is shared between the IROM (0x42000000) and DROM
     * (0x3C000000) windows: the MMU entry index is (vaddr & 0x1FFFFFF) >> 16,
     * so both windows alias the same 64KB entries. IROM occupies entries
     * starting at 0; this NOLOAD dummy advances the DROM location counter past
     * the IROM image size so .flash.rodata lands on later, non-colliding
     * entries (mirrors ESP-IDF's .flash_rodata_dummy). */
    .flash_rodata_dummy (NOLOAD) :
    {
        . = ALIGN(4) + SIZEOF(.flash.text);
        . = ALIGN(0x10000) + 0x20;
    } > drom_seg

    /* Read-only data in the byte-accessible DROM window. */
    .flash.rodata :
    {
        _srodata = .;
        /* App descriptor must be the first bytes of the DROM segment so the
         * bootloader finds it at the expected offset. */
        KEEP (*(.rodata_desc))
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)

        . = ALIGN(4);
        PROVIDE_HIDDEN (__pg_registry_start = .);
        KEEP (*(.pg_registry))
        KEEP (*(SORT(.pg_registry.*)))
        PROVIDE_HIDDEN (__pg_registry_end = .);

        . = ALIGN(4);
        PROVIDE_HIDDEN (__pg_resetdata_start = .);
        KEEP (*(.pg_resetdata))
        PROVIDE_HIDDEN (__pg_resetdata_end = .);

        . = ALIGN(4);
        _erodata = .;
    } > drom_seg

    /* Initialised data: loaded into DRAM by the bootloader. */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        *(.sdata .sdata.*)
        . = ALIGN(4);
        _edata = .;
    } > dram_seg

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
        . = ALIGN(4);
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
    _heap_end = _estack - _Min_Stack_Size;
    ASSERT(_heap_start <= _heap_end,
        "ESP32-S3 static DRAM leaves no room for the minimum stack")

    /* Persistent config storage: 64 KB at flash offset 0x210000, immediately after
     * the 2 MB factory app partition (same offset on the 4/8/16 MB boards). At
     * runtime configFlashInit() (ESP32/config_flash.c) maps it into the byte-
     * accessible DROM window through the flash MMU, so the memory-mapped reads in
     * config_eeprom.c resolve here. It MUST be the DROM window (0x3C000000): byte
     * reads from the IROM window (0x42000000) fault. */
    __config_start = 0x3C000000 + 0x210000;
    __config_end   = __config_start + 64K;

    /* Peripheral register mappings at fixed hardware addresses */
    .peripheral_uart0 (NOLOAD) :    { *(.peripheral_uart0) }   > periph_uart0_seg
    .peripheral_gpio (NOLOAD) :     { *(.peripheral_gpio) }    > periph_gpio_seg
    .peripheral_sens (NOLOAD) :     { *(.peripheral_sens) }    > periph_sens_seg
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
    .peripheral_gdma (NOLOAD) :     { *(.peripheral_gdma) }    > periph_gdma_seg
    .peripheral_system (NOLOAD) :   { *(.peripheral_system) }  > periph_system_seg

    /DISCARD/ :
    {
        *(.comment)
        *(.note.*)
        *(.eh_frame*)
    }
}

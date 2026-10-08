/*
 * navq95_flexspi.ld — XIP-from-FlexSPI1 linker script for the NavQ95
 *                     (NXP i.MX 95, Cortex-M7).
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * BOOT / MEMORY MODEL
 * ─────────────────────────────────────────────────────────────────────────────
 * The M7 image is MCUboot-wrapped and XIPs from FlexSPI1 external NOR.
 *   - FlexSPI1 primary window : 0x28000000   (flash target; e.g. pyocd --base-address 0x28020000)
 *   - FlexSPI1 low alias       : 0x02000000  (execution alias)
 *   - MCUboot header           : 0x800 bytes at the image base; the app vector
 *                                table therefore sits at image_base + 0x800.
 *
 * We link for XIP at the FlexSPI1 LOW ALIAS (0x02000000), with the image placed
 * at offset 0x20000 (matching the 0x28020000 flash slot), i.e. app base alias
 * 0x02020000, MCUboot header 0x02020000..0x020207FF, app vectors at 0x02020800.
 *
 * Writable sections (.data/.bss/stack/heap) live in on-chip SRAM; code XIPs from
 * flash. Startup copies .data (LMA in flash -> VMA in SRAM) and zeroes .bss.
 *
 * On-chip RAM (i.MX95 M7):
 *   ITCM  0x00000000 (256K)   DTCM/SRAM 0x20000000  SRAM 0x20008000 (224K)
 *   OCRAM 0x20480000 (256K)   DDR 0x80000000
 */

ENTRY(Reset_Handler)

/* MCUboot header size (confirmed from the flashed image: hdr_size = 0x800). */
MCUBOOT_HEADER_SIZE = 0x800;

/* Stack / heap (in SRAM) */
_Min_Heap_Size  = 0x4000;   /* 16 KB */
_Min_Stack_Size = 0x4000;   /* 16 KB */

MEMORY
{
    /* FlexSPI1 NOR, LOW ALIAS, at the 0x20000 flash slot.
     * m_header  = the 0x800 MCUboot header (reserved; filled by imgtool later)
     * m_vectors = app vector table, immediately after the header
     * m_flash   = XIP code/rodata */
    m_header     (r)   : ORIGIN = 0x02020000,              LENGTH = MCUBOOT_HEADER_SIZE   /* 0x800 */
    m_vectors    (rx)  : ORIGIN = 0x02020000 + 0x800,      LENGTH = 0x00000800            /* 2 KB vectors (i.MX95 M7 has ~278 IRQs -> table >1KB) */
    m_flash      (rx)  : ORIGIN = 0x02020000 + 0x800 + 0x800, LENGTH = 0x007FF000         /* ~8 MB XIP code */

    /* On-chip RAM */
    m_itcm       (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00040000   /* 256 KB ITCM  */
    m_sram       (rwx) : ORIGIN = 0x20008000, LENGTH = 0x00038000   /* 224 KB SRAM (data/bss/stack/heap) */
    m_ocram      (rwx) : ORIGIN = 0x20480000, LENGTH = 0x00040000   /* 256 KB OCRAM (DMA/non-cached later) */
}

_stack_top = ORIGIN(m_sram) + LENGTH(m_sram);   /* top of SRAM */
_estack    = _stack_top;

SECTIONS
{
    /* ── MCUboot header placeholder (imgtool overwrites this 0x800 region) ── */
    .mcuboot_header (NOLOAD) :
    {
        . = . + MCUBOOT_HEADER_SIZE;
    } > m_header

    /* ── Vector table, at app_base + 0x800 ─────────────────────────────── */
    .isr_vector :
    {
        . = ALIGN(4);
        __VECTOR_TABLE = .;
        __Vectors = .;
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > m_vectors

    /* ── Code + read-only data (XIP from flash) ────────────────────────── */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > m_flash

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > m_flash
    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > m_flash

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > m_flash
    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > m_flash
    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > m_flash

    /* ── Betaflight parameter-group (PG) registry — read-only, in flash ───
     * config.c / pg.c walk __pg_registry_start.._end and pgResetInstance reads
     * __pg_resetdata_start.._end. These output sections must be defined so the
     * symbols resolve and the data lands in flash. */
    .pg_registry :
    {
        PROVIDE_HIDDEN(__pg_registry_start = .);
        KEEP(*(.pg_registry))
        KEEP(*(SORT(.pg_registry.*)))
        PROVIDE_HIDDEN(__pg_registry_end = .);
    } > m_flash

    .pg_resetdata :
    {
        PROVIDE_HIDDEN(__pg_resetdata_start = .);
        KEEP(*(.pg_resetdata))
        PROVIDE_HIDDEN(__pg_resetdata_end = .);
    } > m_flash

    .build_info :
    {
        KEEP(*(.build_info))
    } > m_flash

    /* ── Initialised data: LMA in flash, VMA in SRAM (copied at startup) ── */
    _data_load = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _data_start = .;
        __data_start__ = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _data_end = .;
        __data_end__ = .;
    } > m_sram AT > m_flash

    /* ── BSS (zeroed at startup) in SRAM ───────────────────────────────── */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _bss_start = .;
        __bss_start__ = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
        __bss_end__ = .;
    } > m_sram

    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _heap_start = .;
        __end__ = .;
        PROVIDE(end = .);
        __HeapBase = .;
        . = . + _Min_Heap_Size;
        __HeapLimit = .;
        _heap_end = .;
    } > m_sram

    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > m_sram

    __StackTop   = ORIGIN(m_sram) + LENGTH(m_sram);
    __StackLimit = __StackTop - _Min_Stack_Size;
    PROVIDE(__stack = __StackTop);

    /DISCARD/ :
    {
        libc.a(*)
        libm.a(*)
        libgcc.a(*)
    }
    .ARM.attributes 0 : { *(.ARM.attributes) }

    PROVIDE(_end = _bss_end);
    PROVIDE(end  = _bss_end);

    ASSERT(__StackLimit >= __HeapLimit, "SRAM overflow: data+bss+heap+stack too large")
}

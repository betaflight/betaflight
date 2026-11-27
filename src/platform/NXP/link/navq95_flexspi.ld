/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * NavQ95 (i.MX 95 Cortex-M7) linker script.
 *
 * The image is MCUboot-wrapped and runs in place from the FlexSPI1 NOR. It is
 * flashed at 0x28020000 and linked at the 0x02020000 alias; the 0x800-byte
 * MCUboot header comes first, then the vector table. Data, bss, heap and stack
 * are in SRAM; the NOR write code is copied to OCRAM.
 */

ENTRY(Reset_Handler)

MCUBOOT_HEADER_SIZE = 0x800;

_Min_Heap_Size  = 0x4000;
_Min_Stack_Size = 0x4000;

MEMORY
{
    m_header     (r)   : ORIGIN = 0x02020000,                 LENGTH = MCUBOOT_HEADER_SIZE
    m_vectors    (rx)  : ORIGIN = 0x02020000 + 0x800,         LENGTH = 0x00000800
    m_flash      (rx)  : ORIGIN = 0x02020000 + 0x800 + 0x800, LENGTH = 0x007FF000

    m_itcm       (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00040000
    m_sram       (rwx) : ORIGIN = 0x20008000, LENGTH = 0x00038000
    m_ocram      (rwx) : ORIGIN = 0x20480000, LENGTH = 0x00040000
}

_stack_top = ORIGIN(m_sram) + LENGTH(m_sram);
_estack    = _stack_top;

/*
 * Config region in the NOR, in primary-window addresses, followed by a backup
 * of the same size. Its size must match EEPROM_SIZE in target.h.
 */
__config_start = 0x28E20000;
__config_end   = 0x28E22000;

SECTIONS
{
    /* Filled in by imgtool when the image is signed. */
    .mcuboot_header (NOLOAD) :
    {
        . = . + MCUBOOT_HEADER_SIZE;
    } > m_header

    .isr_vector :
    {
        . = ALIGN(4);
        __VECTOR_TABLE = .;
        __Vectors = .;
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > m_vectors

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

    /* Code that writes the NOR cannot run from it. */
    _ocram_text_load = LOADADDR(.ocram_text);
    .ocram_text :
    {
        . = ALIGN(4);
        _ocram_text_start = .;
        *(.ocram_text)
        *(.ocram_text*)
        . = ALIGN(4);
        _ocram_text_end = .;
    } > m_ocram AT > m_flash

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

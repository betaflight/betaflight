/*
*****************************************************************************
**
**  File        : STM32N657XX_RAM.ld
**
**  Abstract    : Linker script for STM32N657xx Device, RAM-only build for
**                debug iteration via OpenOCD `gdb load` over the on-board
**                ST-LINK. Mirror of STM32N657XX_LRUN.ld but with every
**                section linked at and loaded to AXISRAM directly — no
**                XSPI flash LMA bouncing, so the entire image (vectors +
**                code + rodata + data) is writable by gdb.
**
**                Layout:
**                AXISRAM1   1024K  @ 0x24000000  vectors + text + .data + bss + stack
**                AXISRAM2   1024K  @ 0x24100000  D2_RAM region (DMA_RW_D2)
**
**                The companion LRUN script remains the production target
**                (boot from XSPI flash at 0x70000000, copy to AXISRAM at
**                Reset_Handler). This script is dev-only.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

MEMORY
{
    RAM (rwx)         : ORIGIN = 0x24000000, LENGTH = 1024K
    D2_RAM (rwx)      : ORIGIN = 0x24100000, LENGTH = 1024K /* AXISRAM2 */
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", RAM)
REGION_ALIAS("FASTRAM", RAM)

/* Highest address of the user mode stack */
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - 8;

/* CONFIG_IN_RAM builds keep the EEPROM image in eepromData[] in AXISRAM,
 * but isEEPROMVersionValid() / isEEPROMStructureValid() in config_eeprom.c
 * read straight from &__config_start regardless of build flavour. Alias
 * the symbol onto eepromData[] (laid out in .bss) so those reads land on
 * real memory. EEPROM_SIZE is 4096 (default) which matches the array. */
__config_start = ABSOLUTE(eepromData);
__config_end   = ABSOLUTE(eepromData) + 4096;

_Min_Heap_Size = 0;
_Min_Stack_Size = 0x800;
_sstack = _estack - _Min_Stack_Size;

SECTIONS
{
  /*
   * ISR vector table at the start of AXISRAM1. Reset_Handler runs from
   * here. systemInit() should set VTOR = isr_vector_table_base so that
   * exception entry points to the right place.
   */
  .isr_vector :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector))
    *(.text.Reset_Handler)
    *(.text.Default_Handler)
    . = ALIGN(4);
  } >RAM

  /*
   * Firmware code and read-only data. VMA == LMA (both in RAM), so the
   * Reset_Handler's _stext.._etext copy loop reduces to a no-op in-place
   * memmove. Same for initialiseMemorySections() at the .fastram /
   * .dmaram boundaries.
   */
  _sitext = LOADADDR(.text);

  .text :
  {
    . = ALIGN(4);
    _stext = .;
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
  } >RAM

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >RAM

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*) __exidx_end = .;
  } >RAM

  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >RAM

  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >RAM

  .data :
  {
    . = ALIGN(4);
    *(.data)
    *(.data*)
    . = ALIGN(4);
  } >RAM

  ram_code = LOADADDR(.ram_code);
  .ram_code :
  {
    . = ALIGN(4);
    ram_code_start = .;
    *(.ram_code)
    *(.ram_code*)
    . = ALIGN(4);
    ram_code_end = .;
    _etext = .;
  } >RAM

  . = ALIGN(4);
  .bss (NOLOAD) :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(SORT_BY_ALIGNMENT(.bss*))
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  . = ALIGN(4);
  .sram2 (NOLOAD) :
  {
    _ssram2 = .;
    __sram2_start__ = _ssram2;
    *(.sram2)
    *(SORT_BY_ALIGNMENT(.sram2*))
    . = ALIGN(4);
    _esram2 = .;
    __sram2_end__ = _esram2;
  } >RAM

  _sfastram_idata = LOADADDR(.fastram_data);

  .fastram_data :
  {
    . = ALIGN(4);
    _sfastram_data = .;
    *(.fastram_data)
    *(.fastram_data*)
    . = ALIGN(4);
    _efastram_data = .;
  } >FASTRAM

  . = ALIGN(4);
  .fastram_bss (NOLOAD) :
  {
    _sfastram_bss = .;
    __fastram_bss_start__ = _sfastram_bss;
    *(.fastram_bss)
    *(SORT_BY_ALIGNMENT(.fastram_bss*))
    . = ALIGN(4);
    _efastram_bss = .;
    __fastram_bss_end__ = _efastram_bss;
  } >FASTRAM

  _sdmaram_idata = LOADADDR(.dmaram_data);

  . = ALIGN(32);
  .dmaram_data :
  {
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    _sdmaram_data = .;
    *(.dmaram_data)
    *(.dmaram_data*)
    . = ALIGN(32);
    _edmaram_data = .;
  } >RAM

  . = ALIGN(32);
  .dmaram_bss (NOLOAD) :
  {
    _sdmaram_bss = .;
    __dmaram_bss_start__ = _sdmaram_bss;
    *(.dmaram_bss)
    *(SORT_BY_ALIGNMENT(.dmaram_bss*))
    . = ALIGN(32);
    _edmaram_bss = .;
    __dmaram_bss_end__ = _edmaram_bss;
  } >RAM

  . = ALIGN(32);
  .DMA_RAM (NOLOAD) :
  {
    KEEP(*(.DMA_RAM))
    PROVIDE(dmaram_end = .);
    _edmaram = .;
    _dmaram_end__ = _edmaram;
  } >RAM

  .DMA_RW_D2 (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarw_start = .);
    _sdmarw = .;
    _dmarw_start__ = _sdmarw;
    KEEP(*(.DMA_RW))
    PROVIDE(dmarw_end = .);
    _edmarw = .;
    _dmarw_end__ = _edmarw;
  } >D2_RAM

  .DMA_RW_AXI (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarwaxi_start = .);
    _sdmarwaxi = .;
    _dmarwaxi_start__ = _sdmarwaxi;
    KEEP(*(.DMA_RW_AXI))
    PROVIDE(dmarwaxi_end = .);
    _edmarwaxi = .;
    _dmarwaxi_end__ = _edmarwaxi;
  } >RAM

  .persistent_data (NOLOAD) :
  {
    __persistent_data_start__ = .;
    *(.persistent_data)
    . = ALIGN(4);
    __persistent_data_end__ = .;
  } >RAM

  _heap_stack_end = ORIGIN(STACKRAM)+LENGTH(STACKRAM) - 8;
  _heap_stack_begin = _heap_stack_end - _Min_Stack_Size  - _Min_Heap_Size;
  . = _heap_stack_begin;
  ._user_heap_stack :
  {
    . = ALIGN(4);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(4);
  } >STACKRAM = 0xa5

  .memory_b1_text :
  {
    *(.mb1text)
    *(.mb1text*)
    *(.mb1rodata)
    *(.mb1rodata*)
  } >MEMORY_B1

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

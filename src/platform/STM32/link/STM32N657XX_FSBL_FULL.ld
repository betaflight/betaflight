/*
*****************************************************************************
**
**  File        : STM32N657XX_FSBL_FULL.ld
**
**  Abstract    : Linker script for "BF-as-FSBL" — link the entire
**                Betaflight image as a signed N6 FSBL so the boot ROM
**                loads it directly into AXISRAM and runs it. Replaces the
**                two-stage FSBL stub + XSPI-memory-mapped + memcpy + jump
**                handoff while debugging the bring-up reliability.
**
**                Layout:
**                AXISRAM2.S 509K @ 0x34180400  loaded sections
**                                              (.isr_vector, .text, .data)
**                AXISRAM1.S 1024K @ 0x34000000 NOLOAD: .bss, heap, stack,
**                                              fastram, dmaram, DMA_RAM
**                AXISRAM3   448K @ 0x24200000  D2_RAM (DMA_RW_D2 framebuffer)
**
**                The boot ROM copies the signed payload starting at
**                0x34180400 (0x400 header offset matches `-align`). All
**                other RAM regions are populated by the startup code.
**
*****************************************************************************
*/

ENTRY(Reset_Handler)

MEMORY
{
    /* Loaded image — boot ROM lands the .stm32 payload here. AXISRAM2
     * runs from 0x34100000-0x341FFFFF; the FSBL slot starts at the 0x80400
     * boot-ROM offset, leaving 521 728 bytes (509 KiB) of usable space. */
    LOAD_RAM (rwx)    : ORIGIN = 0x34180400, LENGTH = 521728

    /* AXISRAM1 secure — used for BSS, heap, stack, and DMA-capable RAM
     * regions. Boot ROM does not clock AXISRAM1; SystemInit re-enables it
     * before any of these sections are touched. */
    RAM (rwx)         : ORIGIN = 0x34000000, LENGTH = 1024K

    /* AXISRAM3..6 (contiguous: 0x24200000-0x243BFFFF) used as a single
     * region for the LCD framebuffer (DMA_RW_D2 NOLOAD). 4 × 448 KiB =
     * 1792 KiB, comfortably accommodates the 800x480 ARGB8888 buffer. */
    D2_RAM (rwx)      : ORIGIN = 0x24200000, LENGTH = 1792K

    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", RAM)
REGION_ALIAS("FASTRAM", RAM)

_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - 8;

__config_start = ABSOLUTE(eepromData);
__config_end   = ABSOLUTE(eepromData) + 4096;

_Min_Heap_Size = 0;
_Min_Stack_Size = 0x800;
_sstack = _estack - _Min_Stack_Size;

SECTIONS
{
  /*
   * ISR vector at the start of the loaded image. SP and Reset_Handler are
   * read by the boot ROM as it hands off; the rest of the table is used
   * after VTOR is set in SystemInit.
   */
  .isr_vector :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector))
    *(.text.Reset_Handler)
    *(.text.Default_Handler)
    . = ALIGN(4);
  } >LOAD_RAM

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
  } >LOAD_RAM

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >LOAD_RAM

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*) __exidx_end = .;
  } >LOAD_RAM

  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >LOAD_RAM

  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >LOAD_RAM

  .data :
  {
    . = ALIGN(4);
    *(.data)
    *(.data*)
    . = ALIGN(4);
  } >LOAD_RAM

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
  } >LOAD_RAM

  /* Everything below is NOLOAD — the boot ROM does not populate it; the
   * startup code zeroes .bss and reserves the stack/heap area at runtime. */

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

  .fastram_data (NOLOAD) :
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
  .dmaram_data (NOLOAD) :
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

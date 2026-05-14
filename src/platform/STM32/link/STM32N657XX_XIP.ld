/*
*****************************************************************************
**
**  File        : STM32N657XX_XIP.ld
**
**  Abstract    : Execute-in-place linker for the N6570-DK. Mirrors the
**                STM32H7 EXST split used by the SPRACINGH7 variants:
**                .text / .rodata / .pg_* / .ARM live in XSPI flash at
**                0x70100000 and execute in-place; only initialised data
**                and explicit RAM-resident sections are copied to
**                AXISRAM at Reset_Handler. Replaces the LRUN copy-
**                everything-to-RAM dance, eliminating the unreliable
**                memcpy + jump phase for boot.
**
**                Layout:
**                  AXISRAM1   1024K  @ 0x24000000  data/bss/heap/stack/DMA/ram_code
**                  AXISRAM3..6 1792K @ 0x24200000  contiguous (LCD framebuffer)
**                  XSPI flash 0x70100000+         vectors + text + rodata
**                                                  + LMA for copied data
**
**                The signed FSBL stub at 0x70000000 is responsible for
**                bringing XSPI into memory-mapped mode and jumping to
**                0x70100000; it must NOT deinit XSPI before the jump.
**
**                The clock-switch code path (system_stm32n6xx,
**                stm32n6xx_hal_rcc, stm32n6xx_hal_rcc_ex) is pulled
**                whole-file into the RAM-VMA-copy block so it runs from
**                AXISRAM. BF's SystemClock_Config parks CPU/SYSCLK on
**                HSI mid-init before retuning PLL1 — that yanks the AXI
**                clock that XSPI memory-mapped reads ride on. If the
**                next instruction fetch is from 0x70100000, it returns
**                garbage and the CPU stalls. LTO is disabled in N6.mk
**                for the XIP build so individual .o file names survive
**                for the EXCLUDE_FILE / file-prefix matching here.
**
*****************************************************************************
*/

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)        : ORIGIN = 0x70100000, LENGTH = 1024K
    FLASH_CONFIG (r)  : ORIGIN = 0x70200000, LENGTH = 128K
    FLASH1 (rx)       : ORIGIN = 0x70220000, LENGTH = 1408K

    RAM (rwx)         : ORIGIN = 0x24000000, LENGTH = 1024K   /* AXISRAM1 */
    /* AXISRAM2 NS — first 512 bytes carved out as a debug exchange
     * buffer for the BF↔OBL bring-up contract. OBL's @DBGRAM DFU alt
     * exposes this address so the host can pull markers BF wrote on
     * its way through init / fault. NOLOAD so startup doesn't zero it. */
    DBG_RAM (rw)      : ORIGIN = 0x24100000, LENGTH = 512
    /* AXISRAM3..6 contiguous (0x24200000 - 0x243BFFFF), used as a single
     * region for the LCD framebuffer (DMA_RW_D2 NOLOAD) when LCD enabled. */
    D2_RAM (rwx)      : ORIGIN = 0x24200000, LENGTH = 1792K

    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", RAM)
REGION_ALIAS("FASTRAM", RAM)

_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - 8;

__config_start = ORIGIN(FLASH_CONFIG);
__config_end   = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);

_Min_Heap_Size = 0;
_Min_Stack_Size = 0x800;
_sstack = _estack - _Min_Stack_Size;

SECTIONS
{
  /* ==========================================================
   * XIP-resident sections: live in XSPI and execute in-place.
   * ==========================================================
   *
   * .text excludes the three source files whose .text needs to live
   * in AXISRAM (clock-switch path). EXCLUDE_FILE works on the input
   * file basename — LTO must be off for these names to survive (it is,
   * see OPTIMISATION_BASE override in N6.mk for XIP=1).
   * ========================================================== */

  .isr_vector :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector))
    *(.text.Reset_Handler)
    *(.text.Default_Handler)
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(EXCLUDE_FILE(*system_stm32n6xx.o *stm32n6xx_hal_rcc.o *stm32n6xx_hal_rcc_ex.o) .text*)
    *(EXCLUDE_FILE(*system_stm32n6xx.o *stm32n6xx_hal_rcc.o *stm32n6xx_hal_rcc_ex.o) .rodata)
    *(EXCLUDE_FILE(*system_stm32n6xx.o *stm32n6xx_hal_rcc.o *stm32n6xx_hal_rcc_ex.o) .rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >FLASH

  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >FLASH

  /* ==========================================================
   * Copied sections: LMA in XSPI (immediately after .pg_resetdata in
   * the FLASH cursor), VMA in AXISRAM. Reset_Handler blits the whole
   * range _sitext..(_sitext + _etext - _stext) at boot. The four
   * sections below MUST stay contiguous in BOTH VMA and LMA — no
   * NOLOAD or XIP-only sections allowed in between.
   * ========================================================== */

  _sitext = LOADADDR(.ram_code);

  .ram_code :
  {
    . = ALIGN(4);
    _stext = .;
    ram_code_start = .;
    /* Clock-switch code path. Whole-file matched (file-prefixed
     * pattern claims every .text* input section from these .o
     * files) — paired with the EXCLUDE_FILE in .text above. */
    *system_stm32n6xx.o(.text* .rodata*)
    *stm32n6xx_hal_rcc.o(.text* .rodata*)
    *stm32n6xx_hal_rcc_ex.o(.text* .rodata*)
    *(.ram_code)
    *(.ram_code*)
    . = ALIGN(32);
    ram_code_end = .;
  } >RAM AT >FLASH

  ram_code = LOADADDR(.ram_code);

  .data :
  {
    . = ALIGN(4);
    *(.data)
    *(.data*)
    . = ALIGN(32);
  } >RAM AT >FLASH

  _sfastram_idata = LOADADDR(.fastram_data);
  .fastram_data :
  {
    . = ALIGN(4);
    _sfastram_data = .;
    *(.fastram_data)
    *(.fastram_data*)
    . = ALIGN(32);
    _efastram_data = .;
  } >FASTRAM AT >FLASH

  _sdmaram_idata = LOADADDR(.dmaram_data);
  .dmaram_data :
  {
    . = ALIGN(32);
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    _sdmaram_data = .;
    *(.dmaram_data)
    *(.dmaram_data*)
    . = ALIGN(32);
    _edmaram_data = .;
    _etext = .;
  } >RAM AT >FLASH

  /* ==========================================================
   * NOLOAD sections: live in AXISRAM, never copied from XSPI.
   * ========================================================== */

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

  /* BF↔OBL debug exchange buffer (AXISRAM2 NS, 512 B). NOLOAD so
   * startup doesn't zero it; the OBL maps the same address through
   * its @DBGRAM DFU alt. See platform/bf_obl_contract.h. */
  .bf_obl_dbg (NOLOAD) :
  {
    KEEP(*(.bf_obl_dbg))
  } >DBG_RAM

  .persistent_data (NOLOAD) :
  {
    __persistent_data_start__ = .;
    *(.persistent_data)
    . = ALIGN(4);
    __persistent_data_end__ = .;
  } >RAM

  _heap_stack_end = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - 8;
  _heap_stack_begin = _heap_stack_end - _Min_Stack_Size - _Min_Heap_Size;
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

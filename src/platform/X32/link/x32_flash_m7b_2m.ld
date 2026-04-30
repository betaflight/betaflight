/*
*****************************************************************************
**
**  File        : x32_flash_m7b.ld
**
**  Abstract    : Linker script for X32M7BXI Device with
**                1856K NOR-FLASH (XIP), 480KByte SRAM 1024KByte TCM (SRAM)
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(_start)

/*
0x00000000 to 0x0007FBFF  511K ITCM
0x20000000 to 0x2007FFFF  512K DTCM
0x24000000 to 0x2401FFFF  128K AXI SRAM
0x30000000 to 0x3001FFFF  128K SRAM1
0x30020000 to 0x3003FFFF  128K SRAM2
0x30040000 to 0x30047FFF   32K SRAM3
0x30048000 to 0x3004FFFF   32K SRAM4
0x30050000 to 0x30057FFF   32K SRAM5
0x38000000 to 0x38000FFF    4K BACKUP SRAM

0x15000000 to 0x151CFFFF 1856K full flash,
0x15000000 to 0x1501FFFF  128K isr vector, startup code,
0x15020000 to 0x1503FFFF  128K config,
0x15040000 to 0x151CFFFF 1600K firmware,
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x15000000, LENGTH = 128K
    FLASH_CONFIG (r)  : ORIGIN = 0x15020000, LENGTH = 128K
    FLASH1 (rx)       : ORIGIN = 0x15040000, LENGTH = 1600K

    ITCM_RAM (xrw)    : ORIGIN = 0x00000000, LENGTH = 511K
    DTCM_RAM (xrw)    : ORIGIN = 0x20000000, LENGTH = 512K
    AXI_SRAM (xrw)    : ORIGIN = 0x24000000, LENGTH = 128K

    AHB_SRAM (xrw)    : ORIGIN = 0x30000000, LENGTH = 352K
}

/* Entry Point */
ENTRY(_start)

/* Highest address of the user mode stack */
_estack = ORIGIN(AHB_SRAM) + LENGTH(AHB_SRAM);          /* boot-time stack (AHB_SRAM top) */
_app_estack = ORIGIN(AHB_SRAM) + LENGTH(AHB_SRAM);      /* app-time stack (DTCM_RAM top) */

/* Base address where the config is stored. */
__config_start = ORIGIN(FLASH_CONFIG);
__config_end = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size  = 0x1000;
_Min_Stack_Size = 0x1000;

/* Define output sections */
SECTIONS
{
  /* ===== Stage 1: Boot code in NOR-Flash (XIP) ===== */

  /* Boot vector table: SP + Reset_Handler, placed at FLASH base */
  .isr_vector_boot :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector_boot)) /* Startup code */
    . = ALIGN(4);
  } > FLASH

  /* Boot text: reset handler, lowlevel_init, _main, copy routines, Default_Handler
   *
   * SDK object files that are called during boot (before TCM copy) must also
   * be placed here.  
   *
   * We use a wildcard object-file pattern to match regardless of
   * build-dir prefix, for example ".../system_x32m7xx.o".
   */
  .text.boot :
  {
    . = ALIGN(4);
    KEEP(*(.text.reset))
    KEEP(*(.text.Default_Handler))
    *(.boot_text)
    *(.boot_text*)
    *(.rodata.boot)
    *(.rodata.boot*)

    /* SDK: system init -- called from lowlevel_init via SystemInit() */
    */system_x32m7xx.o(.text*)
    */system_x32m7xx.o(.rodata*)

    . = ALIGN(4);
  } > FLASH

  /* ===== Stage 2: Runtime image copied into ITCM_RAM as one block ===== */

  .runtime_image :
  {
    . = ALIGN(4);
    __image_copy_start__ = .;

    /* Runtime vector table sits at the start of the relocated image. */
    __tcm_vector_start__ = .;
    KEEP(*(.tcm_vector))
    __tcm_vector_end__ = .;

    . = ALIGN(4);
    /* Application code follows contiguously so _main() can copy one image. */
    __itcm_text_start__ = .;
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
    __itcm_text_end__ = .;

    __image_copy_end__ = .;
  } > ITCM_RAM AT > FLASH1

  __image_copy_load_start__ = LOADADDR(.runtime_image);
  __tcm_vector_load_start__ = __image_copy_load_start__;
  __itcm_text_load_start__ = __image_copy_load_start__ + (__itcm_text_start__ - __image_copy_start__);

  .ARM.extab   :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH1

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*) 
    __exidx_end = .;
  } >FLASH1

  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >FLASH1

  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >FLASH1

  /* ===== Stage 3: General data/bss in AHB_SRAM ===== */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)

    . = ALIGN(4);
    _edata = .;
  } > AHB_SRAM AT > FLASH1

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > AHB_SRAM

  /* Crash info section - preserved across resets for post-mortem debugging */
  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    __noinit_start__ = .;
    *(.noinit)
    *(.noinit*)
    . = ALIGN(4);
    __noinit_end__ = .;
  } > AHB_SRAM

  /* ===== Stage 4: TCM data/bss in DTCM_RAM ===== */

  .app_data :
  {
    . = ALIGN(4);
    __app_data_start__ = .;
    *(.tcm_data)
    *(.tcm_data*)
    . = ALIGN(4);
    __app_data_end__ = .;
  } > DTCM_RAM AT > FLASH
  __app_data_load_start__ = LOADADDR(.app_data);

  .app_bss (NOLOAD) :
  {
    . = ALIGN(4);
    __app_bss_start__ = .;
    *(.tcm_bss)
    *(.tcm_bss*)
    . = ALIGN(4);
    __app_bss_end__ = .;
  } > DTCM_RAM

  /* ===== Heap & Stack ===== */

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > AHB_SRAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

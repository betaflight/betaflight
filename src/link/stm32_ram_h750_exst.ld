/*
*****************************************************************************
**
**  File        : stm32_flash_h750_exst.ld
**
**  Abstract    : Linker script for STM32H750xB Device with
**                512K AXI-RAM mapped onto AXI bus on D1 domain
**                128K SRAM1 mapped on D2 domain
**                128K SRAM2 mapped on D2 domain
**                 32K SRAM3 mapped on D2 domain
**                 64K SRAM4 mapped on D3 domain
**                 64K ITCM
**                128K DTCM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x00000000 to 0x0000FFFF   64K ITCM
0x20000000 to 0x2001FFFF  128K DTCM, main RAM
0x24000000 to 0x2407FFFF  512K AXI SRAM, D1 domain
0x30000000 to 0x3001FFFF  128K SRAM1, D2 domain
0x30020000 to 0x3003FFFF  128K SRAM2, D2 domain
0x30040000 to 0x30047FFF   32K SRAM3, D2 domain, unused
0x38000000 to 0x3800FFFF   64K SRAM4, D3 domain, unused
0x38800000 to 0x38800FFF    4K BACKUP SRAM, Backup domain, unused

0x08000000 to 0x0801FFFF  128K isr vector, startup code, firmware, no config! // FLASH_Sector_0
*/

/* 

For H7 EXST (External Storage) targets a binary is built that is placed on an external device.
The bootloader will then copy this entire binary to RAM, at the CODE_RAM address.  The bootloader
then executes code at the CODE_RAM address. The address of CODE_RAM is fixed to 0x24010000
and must not be changed.

Currently, this is inefficient as there are two copies of some sections in RAM.  e.g. .tcm_code.

It would be technically possible to free more RAM by having a more intelligent build system
and bootloader which creates files for each of the sections that are usually copied from flash
to ram and one section for the main code.  e.g. one file for .tcm_code, one file for .data and
one for the main code/data, then load each to the appropriate address and adjust the usual startup
code which will no-longer need to duplicate code/data sections from RAM to ITCM/DTCM RAM.

The initial CODE_RAM is sized at 448K to enable all firmware features and to as much RAM free as 
possible.

*/

/* see .exst section below */
_exst_hash_size = 64;

/*

A section for custom defaults needs to exist for unified targets, however it is a hideous waste of precious RAM.
Using RAM will suffice until an alternative location for it can be made workable.

It would be much better to store the custom defaults on some spare flash pages on the external flash and have some
code to read them from external flash instead of a copy of them stored in precious RAM.
There are usually spare flash pages after the config page on the external flash, however current EXST bootloaders are
not 'custom defaults' aware. they only know about firmware partitions and config partitions.  Using the spare sectors
in the config partition for custom defaults would work, but if users use the bootloader menus to erase their config
then the custom defaults would also be erased...
Also, it would need a change the packaging a distribution method of BF as there would be 2 non-contiguous files to
flash if they were separated, i.e. load firmware at flash address 'x' and load custom defaults at flash address 'y'.

*/

_custom_defaults_size = 8K;

/* Specify the memory areas */
MEMORY
{
    ITCM_RAM (rwx)    : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM_RAM (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)         : ORIGIN = 0x24000000, LENGTH = 64K
    CODE_RAM (rx)     : ORIGIN = 0x24010000, LENGTH = 448K - _custom_defaults_size - _exst_hash_size /* hard coded start address, as required by SPRACINGH7 boot loader, don't change! */
    CUSTOM_DEFAULTS (r) : ORIGIN = ORIGIN(CODE_RAM) + LENGTH(CODE_RAM), LENGTH = _custom_defaults_size
    
    EXST_HASH (rx)    : ORIGIN = 0x24010000 + LENGTH(CODE_RAM) + LENGTH(CUSTOM_DEFAULTS), LENGTH = _exst_hash_size

    D2_RAM (rwx)      : ORIGIN = 0x30000000, LENGTH = 256K /* SRAM1 + SRAM2 */

    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
    QUADSPI (rx)      : ORIGIN = 0x90000000, LENGTH = 0K 
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)
REGION_ALIAS("MAIN", CODE_RAM)

INCLUDE "stm32_h750_common.ld"

SECTIONS
{
  /* used during startup to initialized dmaram_data */
  _sdmaram_idata = LOADADDR(.dmaram_data);

  . = ALIGN(32);
  .dmaram_data :
  {
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    _sdmaram_data = .;        /* create a global symbol at data start */
    *(.dmaram_data)           /* .data sections */
    *(.dmaram_data*)          /* .data* sections */
    . = ALIGN(32);
    _edmaram_data = .;        /* define a global symbol at data end */
  } >RAM AT >MAIN

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
}

INCLUDE "stm32_h750_common_post.ld"
INCLUDE "stm32_ram_h750_exst_post.ld"


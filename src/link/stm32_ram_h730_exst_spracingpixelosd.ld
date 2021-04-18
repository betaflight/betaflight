/*
*****************************************************************************
**
**  File        : stm32_flash_h750_exst_spracingpixelosd.ld
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
0x24000000 to 0x2404FFFF  320K AXI SRAM, D1 domain
0x30000000 to 0x30003FFF   16K SRAM1, D2 domain
0x30004000 to 0x30007FFF   16K SRAM2, D2 domain
0x38000000 to 0x38003FFF   16K SRAM4, D3 domain, unused
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

_spracing_pixel_osd_framebuffer_size = (360 * 288 * 2 / 8) * 2; /* (H * V * Bit-per-pixel / Bits-per-byte) * 2 framebuffers  = 51840 bytes */
/* Specify the memory areas */
MEMORY
{
    ITCM_RAM                          (rwx) : ORIGIN = 0x00000000, LENGTH = 64K - 32K
    SPRACINGPIXELOSD_CODE              (rx) : ORIGIN = ORIGIN(ITCM_RAM) + LENGTH(ITCM_RAM), LENGTH = 32K
    
    DTCM_RAM                          (rwx) : ORIGIN = 0x20000000, LENGTH = 128K - 64K - 4K
    SPRACINGPIXELOSD_DTCM_RAM         (rwx) : ORIGIN = ORIGIN(DTCM_RAM) + LENGTH(DTCM_RAM), LENGTH = 4K
    FRAMEBUFFER_RAM                    (rw) : ORIGIN = ORIGIN(DTCM_RAM) + 64K, LENGTH = _spracing_pixel_osd_framebuffer_size
    
    RAM                               (rwx) : ORIGIN = 0x24000000, LENGTH = 128K + 192K - 2K /* 128K AXI SRAM + 192K ITCM & AXI = 320K */
    SPRACINGPIXELOSD_D1_RAM            (rw) : ORIGIN = ORIGIN(RAM) + LENGTH(RAM) - 2K, LENGTH = 2K

    OCTOSPI2                           (rx) : ORIGIN = 0x70000000, LENGTH = 256M
    OCTOSPI1                           (rx) : ORIGIN = 0x90000000, LENGTH = 256M 
    OCTOSPI1_CODE                      (rx) : ORIGIN = ORIGIN(OCTOSPI1) + 1M, LENGTH = 1M - _exst_hash_size /* hard coded start address, as required by SPRACINGH7 boot loader, don't change! */
    EXST_HASH                          (rx) : ORIGIN = ORIGIN(OCTOSPI1_CODE) + LENGTH(OCTOSPI1_CODE), LENGTH = _exst_hash_size

    D2_RAM_REGION                     (rwx) : ORIGIN = 0x30000000, LENGTH = 32K /* SRAM1 16K + SRAM2 16K */
    
    SPRACINGPIXELOSD_D2_RAM            (rw) : ORIGIN = ORIGIN(D2_RAM_REGION), LENGTH = 1K
    D2_RAM                            (rwx) : ORIGIN = ORIGIN(SPRACINGPIXELOSD_D2_RAM) + LENGTH(SPRACINGPIXELOSD_D2_RAM), LENGTH = 32K - LENGTH(SPRACINGPIXELOSD_D2_RAM) /* SRAM1 16K + SRAM2 16K */
    D3_RAM                            (rwx) : ORIGIN = 0x38000000, LENGTH = 16K /* SRAM4 16K */

    MEMORY_B1                          (rx) : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)
REGION_ALIAS("MAIN", OCTOSPI1_CODE)

REGION_ALIAS("VECTAB", MAIN)

INCLUDE "stm32_h730_common.ld"

SECTIONS
{

  .FRAMEBUFFER_RAM (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(__frame_buffer_start = .);
    KEEP(*(.FRAMEBUFFER_RAM))
    
    /* fill to end of the area, so the usage is 100% */
    FILL(0xDEADBEEF);
    
    . = ORIGIN(FRAMEBUFFER_RAM) + LENGTH(FRAMEBUFFER_RAM);
    PROVIDE(__frame_buffer_end = .);
  } >FRAMEBUFFER_RAM

  .spracingpixelosdlibrary_d2_ram (NOLOAD):
  {
    . = ALIGN(32);
    PROVIDE(__spracingpixelosd_lib_d2_start = .);
    /* fill to end of the area, so the usage is 100% */
    FILL(0xDEADBEEF);
    . = ORIGIN(SPRACINGPIXELOSD_D2_RAM) + LENGTH(SPRACINGPIXELOSD_D2_RAM);
    PROVIDE(__spracingpixelosd_lib_d2_end = .);
  } >SPRACINGPIXELOSD_D2_RAM

  .spracingpixelosdlibrary_dtcm_ram (NOLOAD):
  {
    . = ALIGN(32);
    /* fill to end of the area, so the usage is 100% */
    FILL(0xDEADC0DE);
    . = ORIGIN(SPRACINGPIXELOSD_DTCM_RAM) + LENGTH(SPRACINGPIXELOSD_DTCM_RAM);
  } >SPRACINGPIXELOSD_DTCM_RAM

  /* used during startup to initialized dmaram_data */
  _sdmaram_idata = LOADADDR(.dmaram_data);

  .dmaram_data :
  {
    . = ALIGN(32);
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    _sdmaram_data = .;        /* create a global symbol at data start */
    *(.dmaram_data)           /* .data sections */
    *(.dmaram_data*)          /* .data* sections */
    . = ALIGN(32);
    _edmaram_data = .;        /* define a global symbol at data end */
  } >RAM AT >MAIN

  .dmaram_bss (NOLOAD) :
  {
    . = ALIGN(32);
    _sdmaram_bss = .;
    __dmaram_bss_start__ = _sdmaram_bss;
    *(.dmaram_bss)
    *(SORT_BY_ALIGNMENT(.dmaram_bss*))
    . = ALIGN(32);
    _edmaram_bss = .;
    __dmaram_bss_end__ = _edmaram_bss;
  } >RAM

  .DMA_RAM (NOLOAD) :
  {
    . = ALIGN(32);
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

  .spracingpixelosdlibrary_d1_ram (NOLOAD):
  {
    . = ALIGN(32);
    /* fill to end of the area, so the usage is 100% */
    FILL(0xCAFECAFE);
    . = ORIGIN(SPRACINGPIXELOSD_D1_RAM) + LENGTH(SPRACINGPIXELOSD_D1_RAM);
  } >SPRACINGPIXELOSD_D1_RAM
  
  .spracingpixelosdlibrary_api (NOLOAD):
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__library_descriptor_start = .);
    
    LONG(0x00000000); /* empty library descriptor */
    PROVIDE_HIDDEN (__library_vtable_start = .);
    
    /* fill to end of library, so the usage is 100% */
    FILL(0xDEADBEEF);
    . = ORIGIN(SPRACINGPIXELOSD_CODE) + LENGTH(SPRACINGPIXELOSD_CODE);
  } >SPRACINGPIXELOSD_CODE
  
}

INCLUDE "stm32_h730_common_post.ld"
INCLUDE "stm32_ram_h730_exst_post.ld"


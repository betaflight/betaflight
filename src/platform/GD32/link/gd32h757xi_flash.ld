/*
*****************************************************************************
**
**  File        : gd32_flash_h757xi.ld
**
**  Abstract    : Linker script for GD32H757xI Device with
**                2048KByte FLASH, 512KByte AXI SRAM, 512KByte Shared SRAM
**
**  Target      : GD32H757xI
**
**  Environment : Arm gcc toolchain
**
*****************************************************************************
*/

/*
0x08000000 to 0x080FFFFF   1024K full flash,
0x08000000 to 0x08003FFF   16K isr vector, startup code,
0x08004000 to 0x08007FFF   16K config,                  // FLASH_Sector_1
0x08008000 to 0x080FFFFF   992K firmware,
*/

/* memory map */
MEMORY
{
    FLASH (rx)            : ORIGIN = 0x08000000, LENGTH = 128K
    FLASH_CONFIG (r)      : ORIGIN = 0x08020000, LENGTH = 128K
    FLASH1 (rx)           : ORIGIN = 0x08040000, LENGTH = 1792K

    SYSTEM_MEMORY (rx)    : ORIGIN = 0x1FF00000, LENGTH = 64K

    ITCM_RAM (rwx)        : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM_RAM (rwx)        : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)             : ORIGIN = 0x24000000, LENGTH = 512K
    BACKUP_SRAM (rwx)     : ORIGIN = 0x38800000, LENGTH = 4K

    SRAM0_1 (rwx)         : ORIGIN = 0x30000000, LENGTH = 32K /* SRAM0 + SRAM1 */

    MEMORY_B1 (rx)        : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)

REGION_ALIAS("VECTAB", RAM)

REGION_ALIAS("MOVABLE_FLASH", FLASH1)

INCLUDE "gd32_flash_h7_split.ld"

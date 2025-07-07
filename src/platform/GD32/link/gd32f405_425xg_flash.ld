/*
*****************************************************************************
**
**  File        : gd32f405_425xg_flash.ld
**
**  Abstract    : Linker script for GD32F405_425xG Device with
**                1024KByte FLASH, 256KByte SRAM (that include 64K TCM SRAM)
**
**  Target      : GD32F405_425xG
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
    FLASH (rx)            : ORIGIN = 0x08000000, LENGTH = 16K
    FLASH_CONFIG (r)      : ORIGIN = 0x08004000, LENGTH = 16K
    FLASH1 (rx)           : ORIGIN = 0x08008000, LENGTH = 992K

    SYSTEM_MEMORY (rx)    : ORIGIN = 0x1FFF0000, LENGTH = 29K

    RAM (xrw)             : ORIGIN = 0x20000000, LENGTH = 192K
    TCMRAM (rw)           : ORIGIN = 0x10000000, LENGTH = 64K
    BACKUP_SRAM (rwx)     : ORIGIN = 0x40024000, LENGTH = 4K
    MEMORY_B1 (rx)        : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", TCMRAM)
REGION_ALIAS("FASTRAM", TCMRAM)
REGION_ALIAS("VECTAB", RAM)

REGION_ALIAS("MOVABLE_FLASH", FLASH1)

INCLUDE "gd32_flash_f4_split.ld"

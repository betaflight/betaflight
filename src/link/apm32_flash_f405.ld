/*
*****************************************************************************
**
**  File        : apm32_flash_f405.ld
**
**  Abstract    : Linker script for APM32F405RG Device with
**                1024KByte FLASH, 128KByte RAM 64KByte CCM (RAM)
**
*****************************************************************************
*/

/*
0x08000000 to 0x080FFFFF    1024K full flash,
0x08000000 to 0x08003FFF    16K isr vector, startup code,
0x08004000 to 0x08007FFF    16K config,                     // FLASH_Sector_1
0x08008000 to 0x080FFFFF    992K firmware,
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 16K
    FLASH_CONFIG (r)  : ORIGIN = 0x08004000, LENGTH = 16K
    FLASH1 (rx)       : ORIGIN = 0x08008000, LENGTH = 992K
    SYSTEM_MEMORY (rx): ORIGIN = 0x1FFF0000, LENGTH = 29K
    RAM (rwx)         : ORIGIN = 0x20000000, LENGTH = 128K
    CCM (rwx)         : ORIGIN = 0x10000000, LENGTH = 64K
    BACKUP_SRAM (rwx) : ORIGIN = 0x40024000, LENGTH = 4K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)
REGION_ALIAS("VECTAB", RAM)

/* Put various bits and bobs of data into the main chunk of flash as we have enough of it */

REGION_ALIAS("MOVABLE_FLASH", FLASH1)

INCLUDE "stm32_flash_f4_split.ld"

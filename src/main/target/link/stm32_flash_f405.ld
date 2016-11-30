/*
*****************************************************************************
**
**  File        : stm32_flash_f405.ld
**
**  Abstract    : Linker script for STM32F405RG Device with
**                1024KByte FLASH, 128KByte RAM 64KByte CCM (RAM)
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x08000000 to 0x08100000 1024K full flash,
0x08000000 to 0x080DFFFF 896K firmware,
0x080E0000 to 0x08100000 128K config,       // FLASH_Sector_11
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 896K
    FLASH_CONFIG (r)  : ORIGIN = 0x080E0000, LENGTH = 128K

    RAM (rwx)         : ORIGIN = 0x20000000, LENGTH = 128K
    CCM (rwx)         : ORIGIN = 0x10000000, LENGTH = 64K
    BACKUP_SRAM (rwx) : ORIGIN = 0x40024000, LENGTH = 4K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", CCM)

INCLUDE "stm32_flash.ld"
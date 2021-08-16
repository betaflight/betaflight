/*
*****************************************************************************
**
**  File        : stm32_flash_f411_opbl.ld
**
**  Abstract    : Linker script for STM32F411 Device with
**                512KByte FLASH, 128KByte RAM
**
*****************************************************************************
*/

/*
0x08000000 to 0x0807FFFF  512K full flash,
0x08000000 to 0x08003FFF   16K OPBL,
0x08004000 to 0x08007FFF   16K isr vector, startup code,
0x08008000 to 0x0800BFFF   16K config,                  // FLASH_Sector_2
0x0800C000 to 0x0807FFFF  464K firmware,
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08004000, LENGTH = 16K
    FLASH_CONFIG (r)  : ORIGIN = 0x08008000, LENGTH = 16K
    FLASH1 (rx)       : ORIGIN = 0x0800C000, LENGTH = 464K

    SYSTEM_MEMORY (rx): ORIGIN = 0x1FFF0000, LENGTH = 29K

    RAM (rwx)         : ORIGIN = 0x20000000, LENGTH = 128K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", RAM)
REGION_ALIAS("FASTRAM", RAM)

/* Put various bits and bobs of data into the free space after the vector table in sector 0 to save flash space. */

REGION_ALIAS("MOVABLE_FLASH", FLASH)

INCLUDE "stm32_flash_f4_split.ld"

/*
*****************************************************************************
**
**  File        : stm32_flash_f411.ld
**
**  Abstract    : Linker script for STM32F11 Device with
**                512KByte FLASH, 128KByte RAM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x08000000 to 0x08080000 512K full flash,
0x08000000 to 0x0807C000 469K firmware,
0x0807C000 to 0x08080000  16K config,
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 496K
    FLASH_CONFIG (r)  : ORIGIN = 0x0807C000, LENGTH = 16K

    RAM (rwx)         : ORIGIN = 0x20000000, LENGTH = 128K
    CCM (xrw)         : ORIGIN = 0x10000000, LENGTH = 64K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash.ld"

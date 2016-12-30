/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F103C8 Device with
**                64KByte FLASH, 20KByte RAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 62K
    FLASH_CONFIG (r)  : ORIGIN = 0x0800F800, LENGTH = 2K

    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 20K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", RAM)

INCLUDE "stm32_flash.ld"

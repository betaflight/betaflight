/*
*****************************************************************************
**
**  File        : stm32_flash_1m.ld
**
**  Abstract    : Linker script for STM32F103RG Device with
**                1MByte FLASH, 96KByte RAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 1020K
    FLASH_CONFIG (r)  : ORIGIN = 0x0803F000, LENGTH = 4K

    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 96K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", RAM)
REGION_ALIAS("FASTRAM", RAM)

INCLUDE "stm32_flash.ld"

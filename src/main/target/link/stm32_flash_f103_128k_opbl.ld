/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F103CB Device with
**                128KByte FLASH, 20KByte RAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    /* First 12K (0x3000 bytes) used for OP Bootloader, last 2K used for config storage */
    FLASH (rx)        : ORIGIN = 0x08003000, LENGTH = 114K
    FLASH_CONFIG (r)  : ORIGIN = 0x0801F800, LENGTH = 2K

    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 20K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", RAM)

INCLUDE "stm32_flash.ld"

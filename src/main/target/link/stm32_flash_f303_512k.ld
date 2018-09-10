/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F303xD/E Device
**                FLASH 512KByte
**                SRAM   64KByte
**                CCM    16KByte
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 508K
    FLASH_CONFIG (r)  : ORIGIN = 0x0807F000, LENGTH = 4K

    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 64K
    CCM (xrw)         : ORIGIN = 0x10000000, LENGTH = 16K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)

INCLUDE "stm32_flash.ld"

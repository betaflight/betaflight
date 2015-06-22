/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F103RC Device with
**                256KByte FLASH, 48KByte RAM
**
*****************************************************************************
*/

/* Specify the memory areas. Flash is limited for last 2K for configuration storage */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 254K /* last 2kb used for config storage */
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 48K
  MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash.ld"

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
  FLASH (rx)       : ORIGIN = 0x08003000, LENGTH = 124K - 0x03000 /* last 4kb used for config storage first 12k for OP Bootloader */
  FLASH_CONFIG (r) : ORIGIN = 0x0801F000, LENGTH = 4K

  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 20K
  MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash.ld"

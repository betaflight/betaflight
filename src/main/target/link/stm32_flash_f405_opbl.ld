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

/* Specify the memory areas */

/*
0x08000000 to 0x08100000 1024kb full flash,
0x08000000 to 0x08020000 128kb OPBL,
0x08020000 to 0x08080000 384kb firmware,
0x08080000 to 0x080A0000 128kb config,
*/

MEMORY
{
	FLASH (rx)      : ORIGIN = 0x08020000, LENGTH = 0x000A0000 
  CCM (rwx)       : ORIGIN = 0x10000000, LENGTH = 64K
	RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 0x00020000
	MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash.ld"
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
0x08000000 to 0x08080000 512kb full flash,
0x08000000 to 0x08060000 384kb firmware,
0x08060000 to 0x08080000 128kb config,
*/

/* Specify the memory areas */
MEMORY
{
	FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 0x00080000 
	RAM (rwx)       : ORIGIN = 0x20000000, LENGTH = 0x00020000
	MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash.ld"

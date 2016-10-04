/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F11 Device with
**                512KByte FLASH, 128KByte RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Environment : Atollic TrueSTUDIO(R)
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
*****************************************************************************
*/

/*
0x08000000 to 0x08080000 512kb full flash,
0x08000000 to 0x08010000 64kb OPBL,
0x08010000 to 0x08060000 320kb firmware,
0x08060000 to 0x08080000 128kb config,
*/

MEMORY
{
	FLASH (rx)      : ORIGIN = 0x08010000, LENGTH = 0x00070000 
	RAM (rwx)       : ORIGIN = 0x20000000, LENGTH = 0x00020000
	MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("CCSRAM", RAM);
REGION_ALIAS("SRAM", RAM);

INCLUDE "stm32_flash.ld"
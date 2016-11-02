/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F407VG Device with
**                1024KByte FLASH, 128KByte RAM
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

REGION_ALIAS("CCSRAM", CCM);
REGION_ALIAS("SRAM", RAM);

INCLUDE "stm32_flash.ld"
/*
*****************************************************************************
**
**  File        : stm32_flash_f745.ld
**
**  Abstract    : Linker script for STM32F745VGTx Device with
**                1024KByte FLASH, 320KByte RAM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 1M
    RAM (rwx)         : ORIGIN = 0x20000000, LENGTH = 320K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}
/* note CCM could be used for stack */

INCLUDE "stm32_flash.ld"

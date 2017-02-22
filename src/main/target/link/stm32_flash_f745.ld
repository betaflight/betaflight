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
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 32K
    FLASH_CONFIG (r)  : ORIGIN = 0x08008000, LENGTH = 32K
    FLASH1 (rx)       : ORIGIN = 0x08010000, LENGTH = 960K

    TCM (rwx)         : ORIGIN = 0x20000000, LENGTH = 64K
    RAM (rwx)         : ORIGIN = 0x20010000, LENGTH = 256K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}
/* note CCM could be used for stack */
REGION_ALIAS("STACKRAM", TCM)

INCLUDE "stm32_flash_f7.ld"

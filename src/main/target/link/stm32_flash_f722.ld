/*
*****************************************************************************
**
**  File        : stm32_flash_f722.ld
**
**  Abstract    : Linker script for STM32F722RETx Device with
**                512KByte FLASH, 256KByte RAM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 16K
    FLASH_CONFIG (r)  : ORIGIN = 0x08004000, LENGTH = 16K
    FLASH1 (rx)       : ORIGIN = 0x08008000, LENGTH = 480K

    TCM (rwx)         : ORIGIN = 0x20000000, LENGTH = 64K
    RAM (rwx)         : ORIGIN = 0x20010000, LENGTH = 192K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

/* note TCM could be used for stack */
REGION_ALIAS("STACKRAM", TCM)

INCLUDE "stm32_flash_f7.ld"

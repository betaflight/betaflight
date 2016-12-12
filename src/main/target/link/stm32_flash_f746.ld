/*
*****************************************************************************
**
**  File        : stm32_flash_f746.ld
**
**  Abstract    : Linker script for STM32F746VGTx Device with
**                1024KByte FLASH, 320KByte RAM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 768K
    FLASH_CONFIG (r)  : ORIGIN = 0x080C0000, LENGTH = 256K

    TCM (rwx)         : ORIGIN = 0x20000000, LENGTH = 64K
    RAM (rwx)         : ORIGIN = 0x20010000, LENGTH = 256K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}
/* note CCM could be used for stack */
REGION_ALIAS("STACKRAM", TCM)

INCLUDE "stm32_flash.ld"

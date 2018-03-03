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

/*
0x08000000 to 0x0807FFFF  512K full flash,
0x08000000 to 0x08003FFF   16K isr vector, startup code,
0x08004000 to 0x08007FFF   16K config,                  // FLASH_Sector_1
0x08008000 to 0x0807FFFF  480K firmware,
*/

/* Specify the memory areas */
MEMORY
{
    ITCM_ISR (rx)     : ORIGIN = 0x00000000, LENGTH = 1K
    ITCM_RAM (rx)     : ORIGIN = 0x00000400, LENGTH = 15K

    ITCMFL (rx)       : ORIGIN = 0x00200000, LENGTH = 16K 
    ITCMFL1 (rx)      : ORIGIN = 0x00208000, LENGTH = 480K 

    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 16K
    FLASH_CONFIG (r)  : ORIGIN = 0x08004000, LENGTH = 16K
    FLASH1 (rx)       : ORIGIN = 0x08008000, LENGTH = 480K

    TCM (rwx)         : ORIGIN = 0x20000000, LENGTH = 64K
    SRAM1 (rwx)       : ORIGIN = 0x20010000, LENGTH = 176K
    SRAM2 (rwx)       : ORIGIN = 0x2003C000, LENGTH = 16K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("RAM", SRAM1)

/* note TCM could be used for stack */
REGION_ALIAS("STACKRAM", TCM)
REGION_ALIAS("FASTRAM", TCM)

INCLUDE "stm32_flash_f7_split.ld"

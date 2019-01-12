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
0x08000000 to 0x0807FFFF  512K full flash
0x08000000 to 0x08003FFF   16K ISR vector
0x08004000 to 0x08007FFF   16K config
0x08008000 to 0x0807FFFF  480K firmware
*/

/* Specify the memory areas */
MEMORY
{
    ITCM_RAM (rx)           : ORIGIN = 0x00000000, LENGTH = 16K

    ITCM_FLASH (rx)         : ORIGIN = 0x00200000, LENGTH = 16K
    /* config occupies the entire flash sector 1 for the ease of erasure, 16K on F72x */
    ITCM_FLASH_CONFIG (r)   : ORIGIN = 0x00204000, LENGTH = 16K
    ITCM_FLASH1 (rx)        : ORIGIN = 0x00208000, LENGTH = 480K

    AXIM_FLASH (rx)         : ORIGIN = 0x08000000, LENGTH = 16K
    AXIM_FLASH_CONFIG (r)   : ORIGIN = 0x08004000, LENGTH = 16K
    AXIM_FLASH1 (rx)        : ORIGIN = 0x08008000, LENGTH = 480K

    DTCM_RAM (rwx)          : ORIGIN = 0x20000000, LENGTH = 64K
    SRAM1 (rwx)             : ORIGIN = 0x20010000, LENGTH = 176K
    SRAM2 (rwx)             : ORIGIN = 0x2003C000, LENGTH = 16K
    MEMORY_B1 (rx)          : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("FLASH", AXIM_FLASH)
REGION_ALIAS("FLASH_CONFIG", AXIM_FLASH_CONFIG)
REGION_ALIAS("FLASH1", AXIM_FLASH1)

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)
REGION_ALIAS("RAM", SRAM1)

INCLUDE "stm32_flash_f7_split.ld"

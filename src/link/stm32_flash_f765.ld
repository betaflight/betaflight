/*
*****************************************************************************
**
**  File        : stm32_flash_f765.ld
**
**  Abstract    : Linker script for STM32F765xITx Device with
**                2048KByte FLASH, 512KByte RAM
**
*****************************************************************************
*/

/*
0x00000000 to 0x00003FFF   16K TCM RAM,

0x08000000 to 0x081FFFFF 2048K full flash,
0x08000000 to 0x08007FFF   32K isr vector, startup code,
0x08008000 to 0x0800FFFF   32K config,
0x08010000 to 0x081FFFFF  1984K firmware,
*/

/* Specify the memory areas */
MEMORY
{
    ITCM_RAM (rx)           : ORIGIN = 0x00000000, LENGTH = 16K

    ITCM_FLASH (rx)         : ORIGIN = 0x00200000, LENGTH = 32K
    ITCM_FLASH_CONFIG (r)   : ORIGIN = 0x00208000, LENGTH = 32K
    ITCM_FLASH1 (rx)        : ORIGIN = 0x00210000, LENGTH = 1984K

    AXIM_FLASH (rx)         : ORIGIN = 0x08000000, LENGTH = 10K
    AXIM_FLASH_CUSTOM_DEFAULTS (r) : ORIGIN = 0x08002800, LENGTH = 10K
    AXIM_FLASH_OVERFLOW (r) : ORIGIN = 0x08005000, LENGTH = 12K
    AXIM_FLASH_CONFIG (r)   : ORIGIN = 0x08008000, LENGTH = 32K
    AXIM_FLASH1 (rx)        : ORIGIN = 0x08010000, LENGTH = 1984K

    DTCM_RAM (rwx)          : ORIGIN = 0x20000000, LENGTH = 128K
    SRAM1 (rwx)             : ORIGIN = 0x20020000, LENGTH = 368K
    SRAM2 (rwx)             : ORIGIN = 0x2007C000, LENGTH = 16K
    MEMORY_B1 (rx)          : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("FLASH", AXIM_FLASH)
REGION_ALIAS("FLASH_OVERFLOW", AXIM_FLASH_OVERFLOW)
REGION_ALIAS("FLASH_CUSTOM_DEFAULTS", AXIM_FLASH_CUSTOM_DEFAULTS)
REGION_ALIAS("FLASH_CONFIG", AXIM_FLASH_CONFIG)
REGION_ALIAS("FLASH1", AXIM_FLASH1)

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)
REGION_ALIAS("RAM", DTCM_RAM)

INCLUDE "stm32_flash_f7_split.ld"

/*
*****************************************************************************
**
**  File        : stm32_flash_f746.ld
**
**  Abstract    : Linker script for STM32F745VGTx Device with
**                2048KByte FLASH, 512KByte RAM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x08000000 to 0x081FFFFF 2048K full flash,
0x08000000 to 0x08007FFF   32K isr vector, startup code,
0x08008000 to 0x0800FFFF   32K config,                  // FLASH_Sector_1
0x08010000 to 0x081FFFFF  1984K firmware,
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 32K
    FLASH_CONFIG (r)  : ORIGIN = 0x08008000, LENGTH = 32K
    FLASH1 (rx)       : ORIGIN = 0x08010000, LENGTH = 1984K

    TCM (rwx)         : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)         : ORIGIN = 0x20020000, LENGTH = 384K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}
/* note CCM could be used for stack */
REGION_ALIAS("STACKRAM", TCM)

INCLUDE "stm32_flash_split.ld"

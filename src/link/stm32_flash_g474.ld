/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32G474 (Category 3) device with
**                512KByte FLASH and 96KByte SRAM and 32KByte CCM SRAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 10K
    FLASH_CUSTOM_DEFAULTS (r)  : ORIGIN = 0x08002800, LENGTH = 6K
    FLASH_CONFIG (r)  : ORIGIN = 0x08004000, LENGTH = 8K
    FLASH1 (rx)       : ORIGIN = 0x08006000, LENGTH = DEFINED(USE_CUSTOM_DEFAULTS_EXTENDED) ? 480K : 488K
    FLASH_CUSTOM_DEFAULTS_EXTENDED (r): ORIGIN = DEFINED(USE_CUSTOM_DEFAULTS_EXTENDED) ? 0x0807E000 : 0x08080000, LENGTH = DEFINED(USE_CUSTOM_DEFAULTS_EXTENDED) ? 8K : 0K


    SYSTEM_MEMORY (r) : ORIGIN = 0x1FFF0000, LENGTH = 64K
/* Below are the true lengths for normal and close coupled RAM
 *    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 96K
 *    CCM (xrw)         : ORIGIN = 0x20018000, LENGTH = 32K
 * Allow normal RAM overflow to occupy start of CCM, and only reserve 2560 bytes for  vector table and stack in CCM
 * CCM is aligned to a 512 byte boundary
 */
    RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 124K
    CCM (xrw)         : ORIGIN = 0x2001F000, LENGTH = 4K
    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", CCM)
REGION_ALIAS("FASTRAM", CCM)
REGION_ALIAS("VECTAB", CCM)

/* Put various bits and bobs of data into the free space after the vector table in sector 0 to save flash space. */

REGION_ALIAS("MOVABLE_FLASH", FLASH)

INCLUDE "stm32_flash_g4_split.ld"

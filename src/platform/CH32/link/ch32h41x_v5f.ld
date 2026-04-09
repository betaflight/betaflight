/*
    FC ONLY

    FLASH 
    0x00000000 - 0x00010000    Reserve for V3F 
    0x00010000 - 0x00014000    FLASH
    0x00014000 - 0x00018000    FLASH_CONFIG
    0x00018000 - 0x000F0000    FLASH_CODE    864KB
    SRAM                                    0x2017FC00
    0x20100000 - 0x20110000    64KB reserve for V3F CODE
    0x20110000 - 0x20118000    32K reserve for V3F
    0x20118000 - 0x2017FC00    479KB 
    0x2017FC00 - 0x20180000    1KB  reserve for simulation persist area
    DTCM 
    0x200C0000 - 256KB         We only use 192KB, the rest is used to store code
                               Code execution efficiency in DTCM is also higher than that in SRAM.         
    ITCM 
    0x200A0000 - 128KB         It can be merged with part of the DTCM memory, which is expanded to 192KB here

*/

/* FC ONLY */
/*
MEMORY
{
    FLASH (rx)         : ORIGIN = 0x08010000, LENGTH = 16K
    FLASH_CONFIG (r)   : ORIGIN = 0x08014000, LENGTH = 16K
    FLASH1 (rx)        : ORIGIN = 0x08018000, LENGTH = 864K
    SYSTEM_MEMORY (rx) : ORIGIN = 0x1FFF0000, LENGTH = 56K
    SRAM (xrw)         : ORIGIN = 0x20118000, LENGTH = 415K 
    DTCM (xrw)         : ORIGIN = (0x200E0000+512), LENGTH = (128K-512)  
    ITCM (xrw)         : ORIGIN = 0x200A0000, LENGTH = 256K 
}
*/

/* OSD & FC */
/*
    OSD & FC

    FLASH 
    0x00000000 - 0x00016000    Reserve for V3F  88KB
    0x00016000 - 0x00018000    FLASH            8K   for init.S
    0x00018000 - 0x0001C000    FLASH_CONFIG     16KB for config
    0x0001C000 - 0x000F0000    FLASH_CODE       848KB code load
    SRAM                                    0x2017FC00
    0x20100000 - 0x20116000    88KB reserve for V3F CODE
    0x20116000 - 0x20126000    64K reserve for V3F
    0x20126000 - 0x2017FC00    359KB 
    0x2017FC00 - 0x20180000    1KB  reserve for simulation persist area
    DTCM 
    0x200C0000 - 256KB         We only use 192KB, the rest is used to store code
                               Code execution efficiency in DTCM is also higher than that in SRAM.         
    ITCM 
    0x200A0000 - 128KB         It can be merged with part of the DTCM memory, which is expanded to 192KB here

*/

MEMORY
{
    FLASH (rx)         : ORIGIN = 0x08016000, LENGTH = 8K
    FLASH_CONFIG (r)   : ORIGIN = 0x08018000, LENGTH = 16K
    FLASH1 (rx)        : ORIGIN = 0x0801C000, LENGTH = 848K
    SYSTEM_MEMORY (rx) : ORIGIN = 0x1FFF0000, LENGTH = 56K
    SRAM (xrw)         : ORIGIN = 0x20126000, LENGTH = 359K 
    DTCM (xrw)         : ORIGIN = (0x200E0000+512), LENGTH = (128K-512)  
    ITCM (xrw)         : ORIGIN = 0x200A0000, LENGTH = 256K 
}


REGION_ALIAS("STACKRAM", DTCM)
REGION_ALIAS("VECTAB", DTCM)
REGION_ALIAS("RAM", DTCM)
REGION_ALIAS("FASTRAM", DTCM)

REGION_ALIAS("MOVABLE_FLASH", FLASH1)


INCLUDE "ch32_flash_h41x_split.ld"

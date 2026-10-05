/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * i.MX95 Cortex-M7, run in place from mcuboot slot 0 of the FlexSPI1 octal NOR after a 0x800 byte header.
 * No boot ROM structures, so their regions are empty. FLASH_CONFIG is the storage partition at 0xe20000,
 * followed by the 8 KB config backup.
 *   ITCM        0x00000000  256 KB  FAST_CODE, copied out of flash
 *   DTCM        0x20000000  256 KB  data, bss, stack and every DMA buffer
 *   OCRAM_TEXT  0x20480000    8 KB  OCRAM_CODE and OCRAM_RODATA, copied out of flash
 *   OCRAM2      0x20482000  240 KB  SLOW_DATA_ZERO_INIT
 *   PERSIST     0x204be000    8 KB  persistent objects
 */

MEMORY
{
    FLASH_CONFIG_BLOCK (r)  : ORIGIN = 0x28020000, LENGTH = 0x800
    FLASH_IVT          (r)  : ORIGIN = 0x28020800, LENGTH = 0
    FLASH              (rx) : ORIGIN = 0x28020800, LENGTH = 0x700000 - 0x800 - 0x1000
    FLASH_CONFIG       (r)  : ORIGIN = 0x28E20000, LENGTH = 0x2000

    ITCM    (rwx) : ORIGIN = 0x00000000, LENGTH = 256K
    DTCM    (rw)  : ORIGIN = 0x20000000, LENGTH = 256K
    OCRAM_TEXT (rx) : ORIGIN = 0x20480000, LENGTH = 8K
    OCRAM2  (rw)  : ORIGIN = 0x20482000, LENGTH = 240K
    PERSIST (rw)  : ORIGIN = 0x204be000, LENGTH = 8K
}

__has_boot_rom = 0;
__flash_size = 0x700000;    /* the slot, not the far end of FLASH_CONFIG */
__flexram_itcm_size = LENGTH(ITCM);
__flexram_dtcm_size = 256K;

/* The ranges the reset handler seeds. */
__itcm_region_start = ORIGIN(ITCM);
__itcm_region_end = ORIGIN(ITCM) + LENGTH(ITCM);
__dtcm_region_start = ORIGIN(DTCM);
__dtcm_region_end = ORIGIN(DTCM) + LENGTH(DTCM);

SECTIONS
{
    /* An M7 reset does not clear OCRAM. */
    .persistent_data (NOLOAD) :
    {
        __persistent_start = .;
        *(.persistent_data)
        *(.persistent_data*)
        . = ALIGN(8);
        __persistent_end = .;
    } > PERSIST
}

INCLUDE "nxp_itcm256k_sections.ld"

SECTIONS
{
    /* Code that writes the NOR must not fetch from it; the MPU maps this 8 KB read only. */
    .ocram_text :
    {
        . = ALIGN(4);
        __ocram_text_start = .;
        *(.ocram_text)
        *(.ocram_text*)
        *(.ocram_rodata)
        *(.ocram_rodata*)
        . = ALIGN(4);
        __ocram_text_end = .;
    } > OCRAM_TEXT AT > FLASH
    __ocram_text_load = LOADADDR(.ocram_text);
}

/* rpmsg DDR reserved by the device tree: two vrings, Linux's buffer pool, then the resource table. */
__rpmsg_vring0    = 0x88000000;
__rpmsg_vring1    = 0x88008000;
__rpmsg_shm_end   = 0x88220000;
__rpmsg_rsc_table = 0x88220000;

/* The debugger's flash program runs in the first 16 KB of DTCM. */
ASSERT(ADDR(.dtcm_data) >= ORIGIN(DTCM) + 0x4000,
       "DMA buffers in the first 16 KB of DTCM, where the flash program runs")

/* An M7 reset keeps OCRAM but the startup code wipes DTCM. */
ASSERT(__persistent_start >= ORIGIN(OCRAM2) && __persistent_end <= 0x20480000 + 256K,
       "the persistent block must be in OCRAM")

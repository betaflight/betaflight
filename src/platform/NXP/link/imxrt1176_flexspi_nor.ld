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
 * MIMXRT1176 running XIP from a 64 MB octal NOR on FlexSPI1 at 0x30000000.
 * FlexRAM is split into ITCM and DTCM by startup_imxrt11xx.S.
 *   ITCM   0x00000000  256 KB   FAST_CODE
 *   DTCM   0x20000000  256 KB   data, bss, stack and DMA buffers
 *   OCRAM2 0x20240000    1 MB   spare, OCRAM1 and OCRAM2 back to back
 */

MEMORY
{
    FLASH_CONFIG_BLOCK (r)  : ORIGIN = 0x30000000, LENGTH = 0x1000
    FLASH_IVT          (r)  : ORIGIN = 0x30001000, LENGTH = 0x1000
    FLASH              (rx) : ORIGIN = 0x30002000, LENGTH = 0x4000000 - 0x2000 - 0x2000
    FLASH_CONFIG       (r)  : ORIGIN = 0x30000000 + 0x4000000 - 0x2000, LENGTH = 0x2000

    ITCM   (rwx) : ORIGIN = 0x00000000, LENGTH = 256K
    DTCM   (rw)  : ORIGIN = 0x20000000, LENGTH = 256K
    OCRAM2 (rw)  : ORIGIN = 0x20240000, LENGTH = 1024K
}

/* The 11xx boot ROM reads the FlexSPI configuration block 0x400 into the flash. */
__fcb_offset = 0x400;

INCLUDE "nxp_itcm256k_sections.ld"

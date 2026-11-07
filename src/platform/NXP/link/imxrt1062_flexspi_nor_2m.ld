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
 * MIMXRT1062 running XIP from a 2 MB NOR on FlexSPI1 at 0x60000000.
 * FlexRAM is split into ITCM and DTCM by startup_imxrt10xx.S.
 *   ITCM   0x00000000  256 KB   FAST_CODE
 *   DTCM   0x20000000  256 KB   data, bss, stack and DMA buffers
 *   OCRAM2 0x20200000  512 KB   spare
 */

MEMORY
{
    FLASH_CONFIG_BLOCK (r)  : ORIGIN = 0x60000000, LENGTH = 0x1000
    FLASH_IVT          (r)  : ORIGIN = 0x60001000, LENGTH = 0x1000
    /* The top 4 KB sector is reserved, as on the 8 MB parts where it holds a locked factory FCB. */
    FLASH              (rx) : ORIGIN = 0x60002000, LENGTH = 0x200000 - 0x2000 - 0x2000 - 0x1000
    FLASH_CONFIG       (r)  : ORIGIN = 0x60000000 + 0x200000 - 0x3000, LENGTH = 0x2000

    ITCM   (rwx) : ORIGIN = 0x00000000, LENGTH = 256K
    DTCM   (rw)  : ORIGIN = 0x20000000, LENGTH = 256K
    OCRAM2 (rw)  : ORIGIN = 0x20200000, LENGTH = 512K
}

/* The whole part, for the FCFB sflashA1Size, as the configuration region stops below the top. */
__flash_size = 0x200000;

INCLUDE "nxp_itcm256k_sections.ld"

/* The configuration region plus the single reserved top sector must add up to
   the whole part. nxp_itcm256k_sections.ld asserts the rest of the flash layout. */
ASSERT(ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG) + 0x1000 == ORIGIN(FLASH_CONFIG_BLOCK) + __flash_size,
       "1062 flash: config region plus the reserved top sector must equal the 2 MB part")

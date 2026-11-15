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
 * CH32H41x QingKe V5F Flight Controller Memory Layout
 *
 * FLASH
 *   0x08000000 - 0x08016000    88 KB reserved (bootloader / V3F companion core)
 *   0x08016000 - 0x08018000     8 KB V5F startup & vector entry (.init)
 *   0x08018000 - 0x0801C000    16 KB configuration storage (FLASH_CONFIG)
 *   0x0801C000 - 0x080F0000   848 KB firmware code & initialized data (FLASH1)
 *
 * SRAM / TCM
 *   0x200A0000 - 0x200E0000   256 KB ITCM (128 KB ITCM + 128 KB shared TCM)
 *   0x200E0200 - 0x20100000 ~127.5 KB DTCM (512 B reserved at 0x200E0000 for hardware stack push)
 *   0x20100000 - 0x20126000   152 KB reserved SRAM (V3F companion core)
 *   0x20126000 - 0x2017FC00   359 KB V5F general SRAM
 *   0x2017FC00 - 0x20180000     1 KB persistent reset-state area
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

/* Ensure V5F firmware partitions never overlap bootloader, V3F companion-core,
 * or persistent reset-state regions. */
ASSERT(ORIGIN(FLASH) == 0x08016000, "H415 V5F entry partition mismatch")
ASSERT(_start == ORIGIN(FLASH), "H415 V5F entry must be first instruction")
ASSERT(ORIGIN(SRAM) >= 0x20126000, "H415 FC overlaps reserved low SRAM")
ASSERT(ORIGIN(SRAM) + LENGTH(SRAM) <= 0x2017FC00, "H415 persistent RAM overlap")

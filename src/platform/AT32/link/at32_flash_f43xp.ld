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
/******************************************************************************
**
**  File        : at32_flash_f43xP.ld
**
**  Abstract    : Linker script for AT32F435/7xP Device with
**                960KByte FLASH, 384KByte RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : Artery Tek AT32
**
**  Environment : Arm gcc toolchain
**
*****************************************************************************
*/

/*
    FLASH : 0x0800 0000 -- 0x083E FFFF
    MEM   : 0x2000 0000 -- 0x2007 FFFF
*/

_SRAM_SIZE = 192;

MEMORY
{
    FLASH (rx)         : ORIGIN = 0x08000000, LENGTH = 16K
    FLASH_CONFIG (r)   : ORIGIN = 0x08004000, LENGTH = 16K
    FLASH1 (rx)        : ORIGIN = 0x08008000, LENGTH = 928K
    SYSTEM_MEMORY (rx) : ORIGIN = 0x1FFF0000, LENGTH = 16K
    RAM (xrw)          : ORIGIN = 0x20000000, LENGTH = _SRAM_SIZE * 1024
    MEMORY_B1 (rx)     : ORIGIN = 0x60000000, LENGTH = 0K /* external ram */
}

REGION_ALIAS("STACKRAM", RAM)
REGION_ALIAS("VECTAB", RAM)

REGION_ALIAS("MOVABLE_FLASH", FLASH1)

INCLUDE "at32_flash_f4_split.ld"

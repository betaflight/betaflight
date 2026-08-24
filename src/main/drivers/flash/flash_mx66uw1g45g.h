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

#pragma once

#ifdef USE_FLASH_MX66UW1G45G

// Synthetic JEDEC ID. The chip is selected at build time via OCTOSPI_FLASH_CHIP
// because it is configured by the bootloader in 8-line OPI mode and cannot
// respond to 1/4-line JEDEC RDID. Passed to mx66uw1g45g_identify() by the
// dispatch in flash.c so the identify path keeps the same shape as
// JEDEC-probed chips.
#define MX66UW1G45G_JEDEC_ID    0xC2853BU

bool mx66uw1g45g_identify(flashDevice_t *fdevice, uint32_t jedecID);

#endif

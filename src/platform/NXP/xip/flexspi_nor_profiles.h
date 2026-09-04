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

#include "xip/flexspi_nor_config.h"

// Flash profiles a board selects with FLEXSPI_NOR_PROFILE in its target.h.
#define FLEXSPI_NOR_PROFILE_W25Q        1
#define FLEXSPI_NOR_PROFILE_MX25UM      2

// An octal part boots in single line SPI and is switched to octal mode later
// with a second block.
#if FLEXSPI_NOR_PROFILE == FLEXSPI_NOR_PROFILE_MX25UM
#define FLEXSPI_NOR_HAS_FAST_CONFIG     1

// The part in its fast mode; the boot ROM never reads it.
extern const flexspiNorConfig_t flexspiNorFastConfig;
#endif

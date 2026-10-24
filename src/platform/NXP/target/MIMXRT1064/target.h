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

// The MIMXRT1064 chip. Board wiring lives in a config under src/config.
#include "iomux_pad_def.h"

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "1064"
#endif

// Boot flash: the in package 4 MB W25Q32JV on FlexSPI2 (index 1), with DQS and three
// cycle chip select timing.
#define IMXRT_FLEXSPI_NOR_INSTANCE          1u

#define FLEXSPI_NOR_PROFILE                 FLEXSPI_NOR_PROFILE_W25Q
#define FLEXSPI_NOR_READ_SAMPLE_CLK_SRC     FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_DQSPAD
#define FLEXSPI_NOR_CS_HOLD_TIME            3
#define FLEXSPI_NOR_CS_SETUP_TIME           3
#define FLEXSPI_NOR_SERIAL_CLK_FREQ         FLEXSPI_SERIAL_CLKFREQ_133MHZ

// FlexSPI2 reaches the stacked die on bond wires, not on pads, so the boot
// flash bus has no pads to reserve.
#define NXP_NO_RESERVED_PINS

#include "../common/target_nxp.h"

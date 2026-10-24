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

// The MIMXRT1062 chip. Board wiring lives in a config under src/config; a board
// built around a Teensy module uses the TEENSY40 or TEENSY41 target instead.
#include "iomux_pad_def.h"

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "1062"
#endif

// Default boot flash: a Winbond W25Q on FlexSPI1 port A (index 0) at 0x60000000. No DQS,
// so the read sample clock loops back from SCK.
#define IMXRT_FLEXSPI_NOR_INSTANCE          0u

#define FLEXSPI_NOR_PROFILE                 FLEXSPI_NOR_PROFILE_W25Q
#define FLEXSPI_NOR_READ_SAMPLE_CLK_SRC     FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_SCKPAD
#define FLEXSPI_NOR_CS_HOLD_TIME            1
#define FLEXSPI_NOR_CS_SETUP_TIME           1
#define FLEXSPI_NOR_SERIAL_CLK_FREQ         FLEXSPI_SERIAL_CLKFREQ_133MHZ

// Boot flash pads (FlexSPI1 port A, SS0/SCLK/DATA0..3); firmware runs from them.
#define NXP_RESERVED_PINS \
    IO_TAG(GPIO_SD_B1_06), IO_TAG(GPIO_SD_B1_07), IO_TAG(GPIO_SD_B1_08), \
    IO_TAG(GPIO_SD_B1_09), IO_TAG(GPIO_SD_B1_10), IO_TAG(GPIO_SD_B1_11)

#include "../common/target_nxp.h"

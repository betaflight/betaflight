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

// The MIMXRT1176 chip. Board wiring lives in a config under src/config.
#include "iomux_pad_def.h"

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "1176"
#endif

// Default boot flash: octal Macronix MX25UM on FlexSPI1 port A plus the port B data nibble,
// at 0x30000000. The instance counts from zero, as the register headers do.
#define IMXRT_FLEXSPI_NOR_INSTANCE          0u

#define FLEXSPI_NOR_PROFILE                 FLEXSPI_NOR_PROFILE_MX25UM

// Boot flash pads: port A (DQS, SS0, SCLK, DATA0..3), the port B data nibble
// (DATA4..7) and GPIO_SD_B2_04, the unused port B clock. Firmware runs from
// this flash, so these must never be reused.
#define NXP_RESERVED_PINS \
    IO_TAG(GPIO_SD_B2_00), IO_TAG(GPIO_SD_B2_01), IO_TAG(GPIO_SD_B2_02), \
    IO_TAG(GPIO_SD_B2_03), IO_TAG(GPIO_SD_B2_04), IO_TAG(GPIO_SD_B2_05), \
    IO_TAG(GPIO_SD_B2_06), IO_TAG(GPIO_SD_B2_07), IO_TAG(GPIO_SD_B2_08), \
    IO_TAG(GPIO_SD_B2_09), IO_TAG(GPIO_SD_B2_10), IO_TAG(GPIO_SD_B2_11)

// pg/motor.c seeds only eight, so motors 9 to 12 need "resource motor" in the CLI.
#define MAX_SUPPORTED_MOTORS    12

#include "../common/target_nxp.h"

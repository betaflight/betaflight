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

// The PJRC Teensy 4.0 module itself; carrier wiring lives in a config under
// src/config. The TEENSY41 target includes this file.

#include "iomux_pad_def.h"

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "TN40"
#endif

// Boot flash: a Winbond W25Q on FlexSPI1 (index 0) at 0x60000000. No DQS, so the read
// sample clock loops back from SCK.
#define IMXRT_FLEXSPI_NOR_INSTANCE          0u

#define FLEXSPI_NOR_PROFILE                 FLEXSPI_NOR_PROFILE_W25Q
#define FLEXSPI_NOR_READ_SAMPLE_CLK_SRC     FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_SCKPAD
#define FLEXSPI_NOR_CS_HOLD_TIME            1
#define FLEXSPI_NOR_CS_SETUP_TIME           1
#define FLEXSPI_NOR_SERIAL_CLK_FREQ         FLEXSPI_SERIAL_CLKFREQ_133MHZ

// Pads that must keep their reset function: the boot flash (SD_B1_06..11), the
// JTAG port the onboard MKL02 reprograms the chip through (AD_B0_06..11), the
// boot mode straps (AD_B0_04, AD_B0_05), the MKL02's other lines (EMC_01,
// B0_13) and the 3.3V regulator enable (PMIC_ON_REQ).
#define NXP_RESERVED_PINS \
    IO_TAG(GPIO_SD_B1_06), IO_TAG(GPIO_SD_B1_07), IO_TAG(GPIO_SD_B1_08), \
    IO_TAG(GPIO_SD_B1_09), IO_TAG(GPIO_SD_B1_10), IO_TAG(GPIO_SD_B1_11), \
    IO_TAG(GPIO_AD_B0_06), IO_TAG(GPIO_AD_B0_07), IO_TAG(GPIO_AD_B0_08), \
    IO_TAG(GPIO_AD_B0_09), IO_TAG(GPIO_AD_B0_10), IO_TAG(GPIO_AD_B0_11), \
    IO_TAG(GPIO_AD_B0_04), IO_TAG(GPIO_AD_B0_05), \
    IO_TAG(GPIO_EMC_01), IO_TAG(GPIO_B0_13), \
    IO_TAG(SNVS_PMIC_ON_REQ)

// The orange LED on pin 13, lit when driven high. A carrier config may name its
// own LED0_PIN instead.
#ifndef LED0_PIN
#define LED0_PIN                GPIO_B0_03           // pin 13
#define LED0_INVERTED
#endif

#include "../common/target_nxp.h"

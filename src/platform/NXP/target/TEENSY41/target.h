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

// The PJRC Teensy 4.1 module: a 4.0 plus a bigger boot flash and a micro SD
// socket. Overrides of the 4.0's #ifndef blocks go above the include.

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "TN41"
#endif

#include "../TEENSY40/target.h"

// The micro SD socket on USDHC1, four bits wide; card detect shares DAT3.
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           true
#define SDIO_CK_PIN             GPIO_SD_B0_01        // pin 44
#define SDIO_CMD_PIN            GPIO_SD_B0_00        // pin 45
#define SDIO_D0_PIN             GPIO_SD_B0_02        // pin 43
#define SDIO_D1_PIN             GPIO_SD_B0_03        // pin 42
#define SDIO_D2_PIN             GPIO_SD_B0_04        // pin 47
#define SDIO_D3_PIN             GPIO_SD_B0_05        // pin 46

#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD

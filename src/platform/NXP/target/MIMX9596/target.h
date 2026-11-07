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

// The Cortex-M7 of the i.MX95. Board wiring lives in a config under src/config.
#include "iomux_pad_def.h"

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "IMX95"
#endif

// The octal NOR bus the image runs from; debug.c claims the SWD pads.
#ifndef NXP_RESERVED_PINS
#define NXP_RESERVED_PINS \
    IO_TAG(XSPI1_DATA0), IO_TAG(XSPI1_DATA1), IO_TAG(XSPI1_DATA2), IO_TAG(XSPI1_DATA3), \
    IO_TAG(XSPI1_DATA4), IO_TAG(XSPI1_DATA5), IO_TAG(XSPI1_DATA6), IO_TAG(XSPI1_DATA7), \
    IO_TAG(XSPI1_SCLK), IO_TAG(XSPI1_SS0_B), IO_TAG(XSPI1_DQS)
#endif

#define MAX_SUPPORTED_MOTORS    8

#include "../common/target_nxp.h"

// fc/init.c starts SPIDEV_1 to SPIDEV_6 only, so LPSPI7 and LPSPI8 are not available.
#undef SPIDEV_COUNT
#define SPIDEV_COUNT 6
#define USE_SPI_DEVICE_5
#define USE_SPI_DEVICE_6

#undef I2CDEV_COUNT
#define I2CDEV_COUNT 8
#define USE_I2C_DEVICE_5
#define USE_I2C_DEVICE_6
#define USE_I2C_DEVICE_7
#define USE_I2C_DEVICE_8
// eDMA setup costs about ten interrupt entries; the interrupt path takes one per byte.
#define I2C_DMA_MIN_LEN 10

// The M7 has no USB or SD card; the configuration is kept in the NOR storage partition.
#undef USE_VCP
#undef USE_USB_MSC
#undef USE_SDCARD
#undef USE_SDCARD_SDIO

// Restores the configuration backup at boot when a save was cut short.
#define ENABLE_CONFIG_FLASH_INIT

// With Linux on the A55, a tty to Linux is the VCP and blackbox logs go to Linux files.
#ifdef USE_RPMSG
#define USE_VCP
#define USE_BLACKBOX_VIRTUAL
#endif

// The SAR ADC converts against a 1.8 V reference; the core temperature comes from the System Manager and there is no Vref channel.
#define ADC_VOLTAGE_REFERENCE_MV    1800

// The LED strip runs on FlexPWM, which this part does not have.
#undef USE_LED_STRIP


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

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "235B"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight RP2350B"
#endif

#ifdef PICO_TRACE
#include "pico_trace.h"
#define bprintf tprintf
#else
#define bprintf(fmt,...)
#endif

#ifndef RP2350B
#define RP2350B
#endif

//#define USE_MULTICORE

#define USE_UART0
#define USE_UART1
#define UART_RX_BUFFER_SIZE 1024
#define UART_TX_BUFFER_SIZE 1024
#define UARTHARDWARE_MAX_PINS 12
#define UART_TRAIT_AF_PORT 1

#define USE_SPI
#define SPIDEV_COUNT 2
#define USE_SPI_DEVICE_0
#define USE_SPI_DEVICE_1
#define USE_SPI_DMA_ENABLE_LATE
#define MAX_SPI_PIN_SEL 6

#define USE_I2C
#define I2CDEV_COUNT 2
#define USE_I2C_DEVICE_0
#define USE_I2C_DEVICE_1

#define USE_ADC

#define USE_VCP

// Enable MSC with SD card SPI backend for RP2350B target
#define USE_USB_MSC
#define USE_SDCARD
#define USE_SDCARD_SPI

#undef USE_SOFTSERIAL1
#undef USE_SOFTSERIAL2
#undef USE_TRANSPONDER
#undef USE_FLASH
#undef USE_FLASH_CHIP

#undef USE_TIMER
#undef USE_RCC

// Assume on-board flash (see linker files)
#define CONFIG_IN_FLASH

// Pico flash writes are all aligned and in batches of FLASH_PAGE_SIZE (256)
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE   FLASH_PAGE_SIZE
#define FLASH_CONFIG_BUFFER_TYPE            uint8_t

/* DMA Settings */
#define DMA_IRQ_CORE_NUM 1 // Use core 1 for DMA IRQs
#undef USE_DMA_SPEC // not yet required - possibly won't be used at all

// Radio RX
// SERIALRX CRSF supported, also TELEMETRY_CRSF
// #undef USE_CRSF
// #undef USE_SERIALRX_CRSF

// Various untested or unsupported elements are undefined below

#undef USE_RX_SPI
#undef USE_RX_PWM
#undef USE_RX_PPM
#undef USE_RX_CC2500
#undef USE_RX_EXPRESSLRS
#undef USE_RX_SX1280
#undef USE_SERIALRX_GHST
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_JETIEXBUS
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS
#undef USE_SERIALRX_FPORT

#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_IBUS_EXTENDED
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_LTM
#undef USE_TELEMETRY_MAVLINK
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_SRXL

#undef USE_SERIAL_4WAY_BLHELI_INTERFACE
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#undef USE_MULTI_GYRO

#undef USE_RANGEFINDER_HCSR04
#undef USE_MAG
#undef USE_MAG_HMC5883
#undef USE_MAG_SPI_HMC5883
#undef USE_VTX_RTC6705
#undef USE_VTX_RTC6705_SOFTSPI
#undef USE_SRXL
#undef USE_SPEKTRUM
#undef USE_SPEKTRUM_BIND

#undef USE_SERIAL_PASSTHROUGH

#undef USE_MSP_UART
#undef USE_MSP_DISPLAYPORT

#undef USE_DSHOT_TELEMETRY
#undef USE_ESC_SENSOR

#undef USE_VTX
#undef USE_VTX_TRAMP
#undef USE_VTX_SMARTAUDIO
#undef USE_SPEKTRUM_VTX_CONTROL
#undef USE_VTX_COMMON
#undef USE_FLASH_M25P16
#undef USE_FLASH_W25N01G
#undef USE_FLASH_W25N02K
#undef USE_FLASH_W25M
#undef USE_FLASH_W25M512
#undef USE_FLASH_W25M02G
#undef USE_FLASH_W25Q128FV
#undef USE_FLASH_PY25Q128HA
#undef USE_FLASH_W25Q64FV

#undef USE_RPM_LIMIT

#undef USE_SERVOS

#undef USE_OSD_HD

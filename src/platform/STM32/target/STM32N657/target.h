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
#define TARGET_BOARD_IDENTIFIER "SN65"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight - STM32N657"
#endif

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4

// USE_VCP is per-board (CONFIG-side) — leave it to per-config config.h
// so individual boards can opt in/out without forking target.h.

#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
// #define USE_UART7

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_5
#define USE_SPI_DEVICE_6

#define TARGET_IO_PORTA     0xffff
#define TARGET_IO_PORTB     0xffff
#define TARGET_IO_PORTC     0xffff
#define TARGET_IO_PORTD     0xffff
#define TARGET_IO_PORTE     0xffff
#define TARGET_IO_PORTF     0xffff
#define TARGET_IO_PORTG     0xffff
#define TARGET_IO_PORTH     0xffff
#define TARGET_IO_PORTN     0xffff
#define TARGET_IO_PORTO     0xffff
// TARGET_IO_PORTP / Q are deliberately omitted: ports P (gpioid 15) and
// Q (gpioid 16) overflow the 4-bit port nibble in ioTag_t. Flight-
// controller boards rarely use the largest pin-count N6 packages, so
// widening ioTag_t past uint8_t isn't worth the cross-cutting churn.

// XSPI2 is used for boot flash at 0x70000000.
//
// USE_OCTOSPI + FLASH_OCTOSPI_INSTANCE are always needed so
// memoryMappedModeInit() can verify XSPI2->CR.FMODE on boot. The other
// flags below drive flashInit / octoSpiInitDevice, which BF doesn't
// require for the bring-up XIP scenario (config in RAM, no flashfs):
// the FSBL stub leaves XSPI memory-mapped, BF runs XIP, XSPI never
// needs to be poked again. A per-config #define BF_N6_NO_FLASH_CHIP
// (set in src/config/.../config.h) skips the chip-driver path which
// otherwise probes the wrong JEDEC ID and bus-faults the chip.
#define USE_OCTOSPI
#define USE_FLASH_MEMORY_MAPPED
#define FLASH_OCTOSPI_INSTANCE  XSPI2

#ifndef BF_N6_NO_FLASH_CHIP
#define USE_OCTOSPI_DEVICE_1
#define USE_FLASH_W25Q128FV
#define USE_FLASH_CHIP
#endif

// Provide a default so that this target builds on the build server.
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH) && !defined(CONFIG_IN_MEMORY_MAPPED_FLASH)
#define CONFIG_IN_MEMORY_MAPPED_FLASH
#endif
#define EEPROM_SIZE     5120

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

// #define USE_BEEPER

#if !defined(ENABLE_SDIO_INIT)
#define ENABLE_SDIO_INIT 1
#endif
#if !defined(ENABLE_SDIO_PIN_CONFIG)
#define ENABLE_SDIO_PIN_CONFIG 1
#endif

#ifdef USE_SDCARD
#define USE_SDCARD_SPI
#define USE_SDCARD_SDIO
#endif

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
#define USE_SPI_DMA_ENABLE_LATE

// USE_USB_DETECT is per-board (CONFIG-side) — pairs with USE_VCP and the
// USB_DETECT_PIN choice; opt in from per-config config.h.

#define USE_ESCSERIAL

#define USE_ADC
#define USE_EXTI
#define USE_TIMER_UP_CONFIG

// Transponder not yet supported on N6
#undef USE_TRANSPONDER

#define FLASH_PAGE_SIZE ((uint32_t)0x20000) // 128K sectors

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_CACHE_MGMT)
#define USE_LED_STRIP_CACHE_MGMT
#endif

// On N6 the boot path is boot ROM → OpenBootloader (lib/main/STM32/n6_obl)
// → BF. OBL hands BF a running IWDG and decides DFU-vs-retry on the next
// boot from RCC->RSR. BF's only role is to refresh the IWDG from a
// periodic task (see platform/bf_obl_contract.h).
#ifndef ENABLE_BF_OBL
#define ENABLE_BF_OBL 1
#endif

// IOTraversePins / IOConfigGPIO writes wedge on at least one RIFSC-
// restricted port (the loop never returns). Skip the boot-time
// unused-pin sweep on N6 until the IO layer learns to skip restricted
// ports cleanly.
#ifndef ENABLE_UNUSED_PINS_INIT
#define ENABLE_UNUSED_PINS_INIT 0
#endif

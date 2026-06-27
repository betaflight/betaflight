/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Betaflight is distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "H562"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight - STM32H562"
#endif

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4

#define USE_VCP

//#define USE_SOFTSERIAL

#define UNIFIED_SERIAL_PORT_COUNT       1

#define USE_UART1
//#define USE_UART2
//#define USE_UART3
//#define USE_UART4
//#define USE_UART5
//#define USE_UART6
//#define USE_UART7
//#define USE_UART8

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
//#define USE_SPI_DEVICE_5
//#define USE_SPI_DEVICE_6

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
//#define TARGET_IO_PORTG 0xffff

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

#define USE_BEEPER

#ifdef USE_SDCARD
#define USE_SDCARD_SPI
#define USE_SDCARD_SDIO
#endif

// Tie SDIO init/pin config to actually using SDIO. Otherwise the CLI
// resourceTable[] keeps PG_SDIO_PIN_CONFIG entries, but pg/sdio.c is
// gated on USE_SDCARD_SDIO so the PG never registers, and `dump`
// faults inside printResource on a NULL pgFind() result.
#ifdef USE_SDCARD_SDIO
#if !defined(ENABLE_SDIO_INIT)
#define ENABLE_SDIO_INIT 1
#endif
#if !defined(ENABLE_SDIO_PIN_CONFIG)
#define ENABLE_SDIO_PIN_CONFIG 1
#endif
#endif

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
//#define USE_SPI_DMA_ENABLE_LATE

#define USE_USB_DETECT

#define USE_ESCSERIAL

#define USE_ADC
#define USE_EXTI
#define USE_TIMER_UP_CONFIG

#define FLASH_PAGE_SIZE ((uint32_t)0x2000) // 8K sectors

// CONFIG_IN_RAM bring-up needs more than the default 4 KiB to hold every PG that
// gets compiled in for a 2 MiB flash target (VTX table alone is ~290 B). Bump to
// 8 KiB so writeSettingsToEEPROM doesn't overrun eepromData[] and tear the
// next-PG size field, which makes isEEPROMStructureValid() walk off the end.
#ifndef EEPROM_SIZE
#define EEPROM_SIZE 8192
#endif

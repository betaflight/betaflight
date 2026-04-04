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
#define USBD_PRODUCT_STRING     "Betaflight STM32N657"
#endif

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4

#define USE_VCP

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
#define TARGET_IO_PORTP     0xffff
#define TARGET_IO_PORTQ     0xffff

// Provide a default so that this target builds on the build server.
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH)
#define CONFIG_IN_RAM
#endif
#define EEPROM_SIZE     5120

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

// #define USE_BEEPER

// SDIO not yet supported on N6
//#ifdef USE_SDCARD
//#define USE_SDCARD_SPI
//#define USE_SDCARD_SDIO
//#endif

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
#define USE_SPI_DMA_ENABLE_LATE

#define USE_USB_DETECT

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

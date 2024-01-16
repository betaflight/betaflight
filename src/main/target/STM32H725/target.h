/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
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
#define TARGET_BOARD_IDENTIFIER "SH72"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight STM32H725"
#endif

#if !defined(USE_I2C)
#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4
#define USE_I2C_DEVICE_5
#define I2C_FULL_RECONFIGURABILITY
#endif

// Provide a default so that this target builds on the build server.
#if !defined(USE_SPI)
#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_5
#define USE_SPI_DEVICE_6
#define SPI_FULL_RECONFIGURABILITY
#endif

#define USE_SPI_DMA_ENABLE_LATE

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8
#define USE_UART9
#define USE_LPUART1

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 10)

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

#define USE_BEEPER

#ifdef USE_SDCARD
#define USE_SDCARD_SPI
#define USE_SDCARD_SDIO
#endif


#define USE_VCP

#ifdef USE_SOFTSERIAL
#define UNIFIED_SERIAL_PORT_COUNT       3
#else
#define UNIFIED_SERIAL_PORT_COUNT       1
#endif

#define USE_USB_DETECT

#define USE_ESCSERIAL

#define USE_ADC

#define USE_EXTI
#define USE_TIMER_UP_CONFIG

#define FLASH_PAGE_SIZE ((uint32_t)0x20000) // 128K sectors

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_CACHE_MGMT)
#define USE_LED_STRIP_CACHE_MGMT
#endif

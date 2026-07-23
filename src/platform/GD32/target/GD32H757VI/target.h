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
#define TARGET_BOARD_IDENTIFIER "H757"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight - GD32H757"
#endif

#ifndef GD32H757
#define GD32H757
#endif

#define USE_I2C_DEVICE_0
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define I2C0_CLOCKSPEED 400
#define I2C1_CLOCKSPEED 400
#define I2C2_CLOCKSPEED 400
#define I2C3_CLOCKSPEED 400

#define USE_VCP

#define USE_SOFTSERIAL

#ifdef USE_SOFTSERIAL
#define UNIFIED_SERIAL_PORT_COUNT       3
#else
#define UNIFIED_SERIAL_PORT_COUNT       1
#endif

#define USE_UART0
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7

#ifdef USE_UART0
#define SERIAL_UART_FIRST_INDEX    0
#endif

#define USE_INVERTER

#define USE_SPI_DEVICE_0
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_5

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

// #define USE_DSHOT_BITBAND

#define USE_BEEPER

#ifdef USE_SDCARD
#ifndef USE_SDCARD_SDIO
#define USE_SDCARD_SPI
#endif
#if !defined(ENABLE_SDIO_INIT)
#define ENABLE_SDIO_INIT 1
#endif
#if !defined(ENABLE_SDIO_PIN_CONFIG)
#define ENABLE_SDIO_PIN_CONFIG 1
#endif
#endif

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
#define USE_SPI_DMA_ENABLE_LATE

#define USE_USB_DETECT

#define USE_ESCSERIAL

#define USE_ADC
#ifndef ADC_INSTANCE
#define ADC_INSTANCE                ADC0
#endif

#define USE_EXTI

// #define USE_TIMER_UP_CONFIG  //TODO

#define FLASH_PAGE_SIZE ((uint32_t)0x1000) // 4K sectors

#if !defined(ADC0_DMA_OPT)
#define ADC0_DMA_OPT (DMA_OPT_UNUSED)
#endif

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_CACHE_MGMT)
#define USE_LED_STRIP_CACHE_MGMT
#endif

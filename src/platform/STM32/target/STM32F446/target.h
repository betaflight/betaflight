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
#define TARGET_BOARD_IDENTIFIER "F446"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING             "Betaflight STM32F446"
#endif

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define USE_VCP

#define USE_UART1
#define USE_UART2
// #define USE_UART6

#define USE_INVERTER

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

#define TARGET_IO_PORTA                 (0xffff & ~(BIT(14)|BIT(13)))
#define TARGET_IO_PORTB                 (0xffff & ~(BIT(2)))
#define TARGET_IO_PORTC                 (0xffff & ~(BIT(15)|BIT(14)|BIT(13)))
#define TARGET_IO_PORTD                 BIT(2)

// #define USABLE_TIMER_CHANNEL_COUNT      8
// #define USED_TIMERS                     (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(8))

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

#define USE_DSHOT_BITBAND

#ifdef USE_SDCARD
#define USE_SDCARD_SPI
#define USE_SDCARD_SDIO
#endif

#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
#define USE_SPI_DMA_ENABLE_EARLY

#define USE_USB_DETECT

#define USE_ADC

#define USE_EXTI

#define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors

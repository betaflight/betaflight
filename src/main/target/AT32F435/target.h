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

#define TARGET_BOARD_IDENTIFIER "A435"

#define USBD_PRODUCT_STRING     "Betaflight AT32F435"

#ifndef AT32F435
#define AT32F435
#endif

//#define USE_I2C_DEVICE_1

//#define USE_UART1
//#define USE_UART2
//#define USE_UART3

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 3)

#define USE_INVERTER

//#define USE_SPI_DEVICE_1
//#define USE_SPI_DEVICE_2

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff

//#define USE_I2C
//#define I2C_FULL_RECONFIGURABILITY

//#define USE_SPI
//#define SPI_FULL_RECONFIGURABILITY

//#define USE_USB_DETECT
//#define USE_VCP
#define USE_SOFTSERIAL1
#define USE_SOFTSERIAL2

#define UNIFIED_SERIAL_PORT_COUNT       3

#define USE_ADC

#define USE_CUSTOM_DEFAULTS

#undef USE_TIMER
#undef USE_GPIO
#undef USE_DMA
#undef USE_PWM

#if defined(AT32F435ZMT7) || defined(AT32F435RMT7)
#define FLASH_PAGE_SIZE   ((uint32_t)0x1000)  // 4k sectors
#elif defined(AT32F435RGT7)
#define FLASH_PAGE_SIZE   ((uint32_t)0x0800)  // 2K page sectors
#endif

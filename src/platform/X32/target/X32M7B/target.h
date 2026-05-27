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
#define TARGET_BOARD_IDENTIFIER "M7B"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight X32M7B"
#endif

#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4
#define USE_I2C_DEVICE_5
#define USE_I2C_DEVICE_6
#define USE_I2C_DEVICE_7
#define USE_I2C_DEVICE_8
#define USE_I2C_DEVICE_9
#define USE_I2C_DEVICE_10

#define USE_USB_DETECT
#define USE_VCP
#define USE_USB_MSC
#define USE_USBHS1

#define USE_UART
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8
#define USE_UART9
#define USE_UART10
#define USE_UART11
#define USE_UART12
#define USE_UART13
#define USE_UART14
#define USE_UART15

#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define USE_SPI_DEVICE_5
#define USE_SPI_DEVICE_6
#define USE_SPI_DEVICE_7

#define QUADSPIDEV_COUNT 1

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff
#define TARGET_IO_PORTH 0xffff
#define TARGET_IO_PORTI 0xffff
#define TARGET_IO_PORTJ 0xffff
#define TARGET_IO_PORTK 0xffff

#define USE_I2C
#define I2C_FULL_RECONFIGURABILITY

#define USE_BEEPER

#ifdef USE_SDCARD
#define USE_SDCARD_SPI
#define USE_SDCARD_SDIO
#define SDIO_DEVICE SDIODEV_1
#define SDIO_USE_4BIT 1
#define SDIO_CK_PIN PC12
#define SDIO_CMD_PIN PD2
#define SDIO_D0_PIN PC8
#define SDIO_D1_PIN PC9
#define SDIO_D2_PIN PC10
#define SDIO_D3_PIN PC11
#if !defined(ENABLE_SDIO_INIT)
#define ENABLE_SDIO_INIT 1
#endif
#if !defined(ENABLE_SDIO_PIN_CONFIG)
#define ENABLE_SDIO_PIN_CONFIG 1
#endif
#endif


#define USE_GPS


#define USE_SPI
#define SPI_FULL_RECONFIGURABILITY
#define USE_SPI_DMA_ENABLE_LATE

#define USE_ESCSERIAL

#define USE_ADC
#define USE_EXTI
#define USE_TIMER_UP_CONFIG

#undef USE_TRANSPONDER
#ifndef DEFAULT_DSHOT_BITBANG
#define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_AUTO
#endif
#ifndef DEFAULT_DSHOT_BURST
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_OFF
#endif
#ifndef DEFAULT_DSHOT_TELEMETRY
#define DEFAULT_DSHOT_TELEMETRY DSHOT_TELEMETRY_OFF
#endif

#define FLASH_PAGE_SIZE ((uint32_t)0x1000U)

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_CACHE_MGMT)
#define USE_LED_STRIP_CACHE_MGMT
#endif

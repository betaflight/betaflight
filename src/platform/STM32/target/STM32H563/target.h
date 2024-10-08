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

#define TARGET_BOARD_IDENTIFIER "SH56"

#define USBD_PRODUCT_STRING     "Betaflight STM32H563"

#undef USE_PWM
#undef USE_PWM_OUTPUT
#undef USE_DSHOT
#undef USE_ADC
#undef USE_TIMER
#undef USE_DMA
#undef USE_FLASH
#undef USE_FLASH_CHIP
#undef USE_FLASHFS
#undef USE_FLASH_TOOLS
#undef USE_FLASH_M25P16
#undef USE_FLASH_W25N01G
#undef USE_FLASH_W25M
#undef USE_FLASH_W25M512
#undef USE_FLASH_W25M02G
#undef USE_FLASH_W25Q128FV
#undef USE_FLASH_PY25Q128HA

#undef USE_TRANSPONDER
#undef USE_SDCARD
#undef USE_LED_STRIP
#undef USE_SOFTSERIAL
#undef USE_VCP
#undef USE_ESCSERIAL
#undef USE_SPI
#undef USE_I2C
#undef USE_UART
#undef USE_USB_DETECT
#undef USE_BEEPER
#undef USE_EXTI
#undef USE_TIMER_UP_CONFIG
#undef USE_RX_SPI
#undef USE_RX_CC2500
#undef USE_BARO
#undef USE_I2C_GYRO
#undef USE_SPI_GYRO
#undef USE_GYRO
#undef USE_ACC
#undef USE_MAG
#undef USE_MAX7456
#undef USE_VTX_RTC6705
#undef USE_VTX_RTC6705_SOFTSPI
#undef USE_CLI
#undef USE_CAMERA_CONTROL
#undef USE_RX_PWM
#undef USE_LED_STRIP
#undef USE_TRANSPONDER
#undef USE_SERIAL_4WAY_BLHELI_INTERFACE
#undef USE_SERIAL_4WAY_SK_BOOTLOADER
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_MOTOR
#undef USE_SERVO

#define USE_VIRTUAL_GYRO



//#define USE_I2C_DEVICE_1
//#define USE_I2C_DEVICE_2
//#define USE_I2C_DEVICE_3
//#define USE_I2C_DEVICE_4

//#define USE_VCP

//#define USE_SOFTSERIAL

#define UNIFIED_SERIAL_PORT_COUNT       0

#define USE_UART1
//#define USE_UART2
//#define USE_UART3
//#define USE_UART4
//#define USE_UART5
//#define USE_UART6
//#define USE_UART7
//#define USE_UART8

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 1)

//#define USE_SPI_DEVICE_1
//#define USE_SPI_DEVICE_2
//#define USE_SPI_DEVICE_3
//#define USE_SPI_DEVICE_4
//#define USE_SPI_DEVICE_5
//#define USE_SPI_DEVICE_6

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
//#define TARGET_IO_PORTG 0xffff

//#define USE_I2C
//#define I2C_FULL_RECONFIGURABILITY

//#define USE_BEEPER

#ifdef USE_SDCARD
#define USE_SDCARD_SPI
#define USE_SDCARD_SDIO
#endif

//#define USE_SPI
//#define SPI_FULL_RECONFIGURABILITY
//#define USE_SPI_DMA_ENABLE_LATE

//#define USE_USB_DETECT

//#define USE_ESCSERIAL

//#define USE_ADC
//#define USE_EXTI
//#define USE_TIMER_UP_CONFIG

#define FLASH_PAGE_SIZE ((uint32_t)0x20000) // 128K sectors

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_CACHE_MGMT)
#define USE_LED_STRIP_CACHE_MGMT
#endif

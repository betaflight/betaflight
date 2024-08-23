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

/*
 * This target won't actually build a target that you can boot, it is meant as
 * a base target ONLY.
 *
 * When defining a target that uses this as a base you currently:
 *
 * 1) *must* define ALL the SPI instances that the target has, including their pins.
 * 2) *must* define the storage subsystem used to boot, e.g. see CONFIG_IN_xxx.
 * 3) *must* define all the settings required for the config storage system to
 *    be able to load the config at boot time BEFORE it has actually loaded a config.
 *    e.g. for OCTOQSPI define the QSPI instance, pins, mode, etc,
 *         for SDCARD define the SD card bus (SPI/SDIO), pins, etc.
 *
 */

#pragma once

#ifndef TARGET_BOARD_IDENTIFIER
#define TARGET_BOARD_IDENTIFIER "S730"
#endif

#ifndef USBD_PRODUCT_STRING
#define USBD_PRODUCT_STRING     "Betaflight STM32H730"
#endif

#if !defined(USE_I2C)
#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4
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

#define USE_VCP

#define UNIFIED_SERIAL_PORT_COUNT       1

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
#define USE_LPUART1

#define SERIAL_PORT_COUNT       (UNIFIED_SERIAL_PORT_COUNT + 11)

// Disable OCTOSPI pins PB2/CLK, PB6/NCS, PD11/IO0, PD12/IO1, PD13/IO3, PE2/IO2
// PE7/IO4, PE8/IO5, PE9/IO6, PE10/IO7

#if !defined(TARGET_IO_PORTA)
#define TARGET_IO_PORTA 0xffff
#endif
#if !defined(TARGET_IO_PORTB)
#define TARGET_IO_PORTB (0xffff & ~(BIT(2)|BIT(6)))
#endif
#if !defined(TARGET_IO_PORTC)
#define TARGET_IO_PORTC 0xffff
#endif
#if !defined(TARGET_IO_PORTD)
#define TARGET_IO_PORTD (0xffff & ~(BIT(11)|BIT(12)|BIT(13)))
#endif
#if !defined(TARGET_IO_PORTE)
#define TARGET_IO_PORTE (0xffff & ~(BIT(2)|BIT(7)|BIT(8)|BIT(9)|BIT(10)))
#endif
#if !defined(TARGET_IO_PORTF)
#define TARGET_IO_PORTF 0xffff
#endif
#if !defined(TARGET_IO_PORTG)
#define TARGET_IO_PORTG 0xffff
#endif
#if !defined(TARGET_IO_PORTH)
#define TARGET_IO_PORTH 0xffff
#endif

#define USE_BEEPER

#define USE_USB_ID
#define USE_USB_DETECT

#define USE_ESCSERIAL

#define USE_ADC

// Provide a default so that this target builds on the build server.
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH)
#define CONFIG_IN_RAM
#endif

#define USE_EXTI
#define USE_TIMER_UP_CONFIG

#if !(defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_MEMORY_MAPPED_FLASH) || defined(CONFIG_IN_RAM) || defined(CONFIG_IN_SDCARD))
#error "The configured MCU only has one flash page which contains the bootloader, no spare flash pages available, use external storage for persistent config or ram for target testing"
#endif

#define FLASH_PAGE_SIZE ((uint32_t)0x20000) // 128K sectors

#if defined(USE_LED_STRIP) && !defined(USE_LED_STRIP_CACHE_MGMT)
#define USE_LED_STRIP_CACHE_MGMT
#endif

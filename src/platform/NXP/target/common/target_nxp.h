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

// Bonded pads only, counted from the generated pad table of each family.
#if defined(IMXRT11XX)
// GPIO1 to GPIO6 and GPIO13; port G is the always on block.
#define TARGET_IO_PORTA 0xffffffff
#define TARGET_IO_PORTB 0xffffffff
#define TARGET_IO_PORTC 0xffffffff
#define TARGET_IO_PORTD 0xffffffff
#define TARGET_IO_PORTE 0x0001ffff
#define TARGET_IO_PORTF 0x0000ffff
#define TARGET_IO_PORTG 0x00001fff
#elif defined(IMXRT10XX)
// GPIO1 to GPIO5.
#define TARGET_IO_PORTA 0xffffffff
#define TARGET_IO_PORTB 0xffffffff
#define TARGET_IO_PORTC 0x0fffffff
#define TARGET_IO_PORTD 0xffffffff
#define TARGET_IO_PORTE 0x00000007
#else
#error "Unsupported NXP family: no bonded pad masks for the GPIO ports"
#endif

// The unused pin sweep would float unnamed XIP flash pads and fault the next fetch.
#if !defined(NXP_RESERVED_PINS) && !defined(NXP_NO_RESERVED_PINS)
#error "target.h must name the boot flash pads in NXP_RESERVED_PINS, or define NXP_NO_RESERVED_PINS if it has none"
#endif

// 20 ms at 1 Mbit, so serial passthrough survives a host that pauses on USB.
#define UART_RX_BUFFER_SIZE 2048

// LPUART1..LPUART8, and four more on the 11xx, where LPUART11 and LPUART12 have no eDMA.
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8
#if defined(IMXRT11XX)
#define USE_UART9
#define USE_UART10
#define USE_UART11
#define USE_UART12
#endif
// serial_uart_impl.h needs an AF trait; the mux comes from the generated pin table, so af is unused.
#define UART_TRAIT_AF_PORT 1
#define UARTHARDWARE_MAX_PINS 6

// PIOUART0 is a UART on the FlexIO shifters and timers DShot leaves free. A
// board opts in with USE_PIOUART0 and its pins in the config, which is read first.
#if defined(USE_PIOUART0) || defined(USE_PIOUART1)
#if !NXP_HAS_FLEXIO_UART
#error "USE_PIOUARTn needs a family header that defines NXP_HAS_FLEXIO_UART to 1"
#endif
#define SERIAL_PIOUART_FIRST_INDEX 0
#endif

#define USE_SPI
#define SPIDEV_COUNT 4
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3
#define USE_SPI_DEVICE_4
#define MAX_SPI_PIN_SEL 4
#define USE_SPI_DMA_ENABLE_LATE

#define USE_I2C
#define I2CDEV_COUNT 4
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3
#define USE_I2C_DEVICE_4
// The LPI2C driver reads the pull-up and clock speed from the config, so the CLI may set them.
#define I2C_FULL_RECONFIGURABILITY

#define USE_ADC

// Core temperature from TEMPMON (10xx) or TMPSNS (11xx); there is no band gap channel.
#define USE_ADC_INTERNAL

#define USE_VCP

// Config in the last boot flash sectors, written through the boot ROM FlexSPI NOR API in
// 256 byte pages of 4096 byte sectors. The ROM reads its source a word at a time.
#define CONFIG_IN_FLASH
#define FLASH_CONFIG_STREAMER_BUFFER_SIZE   256
#define FLASH_CONFIG_BUFFER_TYPE            uint32_t

// Motor outputs are driven by FlexIO (DShot) and FlexPWM, so the timer and RCC
// layers do not exist on this platform.
#undef USE_TIMER
#undef USE_RCC

// The bit-bang DShot fallback needs USE_TIMER.
#undef USE_DSHOT_BITBANG
#define USE_DSHOT_TELEMETRY

// One eDMA engine, 32 equal channels with round robin arbitration, and a DMAMUX
// that puts any request source on any of them, so there is nothing to move.
#undef USE_DMA_SPEC

// The request source is named per channel in a DMAMUX register rather than
// fixed by the channel, which puts the request numbers in uartHardware_t.
#define DMA_TRAIT_MUX 1

#undef USE_SOFTSERIAL1
#undef USE_SOFTSERIAL2
#undef USE_TRANSPONDER

// SPI NOR and NAND flash are not supported. The leaf options go too, as common_post.h
// rebuilds USE_FLASH_CHIP from them.
#undef USE_FLASH_TOOLS
#undef USE_FLASH_M25P16
#undef USE_FLASH_FM25V02A
#undef USE_FLASH_W25N01G
#undef USE_FLASH_W25N02K
#undef USE_FLASH_W25M
#undef USE_FLASH_W25M512
#undef USE_FLASH_W25M02G
#undef USE_FLASH_W25Q128FV
#undef USE_FLASH_PY25Q128HA
#undef USE_FLASH
#undef USE_FLASH_CHIP

// The analog OSD is not supported on this platform yet.
#undef USE_MAX7456

// USE_SDCARD is left alone: common_pre.h already set it, and this leaf option
// is what picks the SDIO driver over the SPI one.
#define USE_SDCARD_SDIO
#undef USE_SDCARD_SPI

// The blackbox SD cache has to hold more than the 250 ms the driver waits
// before it calls a write late.
#ifndef AFATFS_NUM_CACHE_SECTORS
#define AFATFS_NUM_CACHE_SECTORS 32
#endif

// A boot mode of its own, in usb/usb_msc_imxrt.c.
#define USE_USB_MSC

// Burst DShot is an STM32 timer and DMA feature.
#undef USE_DSHOT_DMAR

// Both of these capture the receiver input with a timer.
#undef USE_RX_PWM
#undef USE_RX_PPM

// ExpressLRS needs a timer it reprograms every packet; the SX1280 and SX127X drivers serve only it.
#undef USE_RX_EXPRESSLRS
#undef USE_RX_SX1280
#undef USE_RX_SX127X

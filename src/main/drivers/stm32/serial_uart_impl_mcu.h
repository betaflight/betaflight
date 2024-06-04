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

// Configuration constants
#if defined(STM32F4)
#define UARTDEV_COUNT_MAX 6
#define UARTHARDWARE_MAX_PINS 4
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif
#ifndef UART_TX_BUFFER_SIZE
#ifdef USE_MSP_DISPLAYPORT
#define UART_TX_BUFFER_SIZE     1280
#else
#define UART_TX_BUFFER_SIZE     256
#endif
#endif
#elif defined(STM32F7)
#define UARTDEV_COUNT_MAX 8
#define UARTHARDWARE_MAX_PINS 4
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif
#ifndef UART_TX_BUFFER_SIZE
#ifdef USE_MSP_DISPLAYPORT
#define UART_TX_BUFFER_SIZE     1280
#else
#define UART_TX_BUFFER_SIZE     256
#endif
#endif
#elif defined(STM32H7)
#define UARTDEV_COUNT_MAX 11 // UARTs 1 to 10 + LPUART1
#define UARTHARDWARE_MAX_PINS 5
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif
#ifndef UART_TX_BUFFER_SIZE
#ifdef USE_MSP_DISPLAYPORT
#define UART_TX_BUFFER_SIZE     1280
#else
#define UART_TX_BUFFER_SIZE     256
#endif
#endif
#elif defined(STM32H5)
#define UARTDEV_COUNT_MAX 11 // UARTs 1 to 10 + LPUART1
#define UARTHARDWARE_MAX_PINS 5
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif
#ifndef UART_TX_BUFFER_SIZE
#ifdef USE_MSP_DISPLAYPORT
#define UART_TX_BUFFER_SIZE     1280
#else
#define UART_TX_BUFFER_SIZE     256
#endif
#endif
#elif defined(STM32G4)
#define UARTDEV_COUNT_MAX 11  // UARTs 1 to 5 + LPUART1 (index 10)
#define UARTHARDWARE_MAX_PINS 3
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     256
#endif
#ifndef UART_TX_BUFFER_SIZE
#ifdef USE_MSP_DISPLAYPORT
#define UART_TX_BUFFER_SIZE     1280
#else
#define UART_TX_BUFFER_SIZE     256
#endif
#endif
#else
#error unknown MCU family
#endif

// For H7 and G4  , {tx|rx}DMAChannel are DMAMUX input index for  peripherals (DMA_REQUEST_xxx); H7:RM0433 Table 110, G4:RM0440 Table 80.
// For F4 and F7, these are 32-bit channel identifiers (DMA_CHANNEL_x)
// so STM32 do not use UART_MUX_CAPABLE

#define UART_AF_CAPABLE

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define UART_TXRXIRQ_CAPABLE
#endif

#if !defined(STM32F4) // Don't support pin swap.
#define UART_PINSWAP_CAPABLE
#endif

#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
#define UART_REG_RXD(base) ((base)->RDR)
#define UART_REG_TXD(base) ((base)->TDR)
#elif defined(STM32F4)
#define UART_REG_RXD(base) ((base)->DR)
#define UART_REG_TXD(base) ((base)->DR)
#endif

#if defined(STM32H7)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM            // D2 SRAM
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM            // D2 SRAM
#elif defined(STM32G4)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM_W          // SRAM MPU NOT_BUFFERABLE
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM_R          // SRAM MPU NOT CACHABLE
#elif defined(STM32F7)
#define UART_TX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT // DTCM RAM
#define UART_RX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT // DTCM RAM
#elif defined(STM32F4) || defined(STM32H5)
#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE
#endif

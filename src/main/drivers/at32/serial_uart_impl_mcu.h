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
#define UARTDEV_COUNT_MAX 8  // UARTs 1 to 5 + LPUART1 (index 9)
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

// For at32f435/7 DmaChannel is the dmamux ,need to call dmamuxenable using dmamuxid
#define UART_USES_DMAMUX
#define UART_USES_PIN_AF
#define UART_PINSWAP_CAPABLE

#define UART_REG_RXD(base) ((base)->dt)
#define UART_REG_TXD(base) ((base)->dt)

#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE

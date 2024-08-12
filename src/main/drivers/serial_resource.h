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

/*
  #defines used for serial port resource access (pin/dma/inversion)
  target/serial_post.h normalizes enabled port definitions, values are just remaned here
 */

// use _MAX value here, resource command needs linear mapping
//  (UART8 is always at index RESOURCE_UART_OFFSET + 7, no matter which other ports are enabled)
#define RESOURCE_UART_COUNT SERIAL_UART_MAX
#define RESOURCE_LPUART_COUNT SERIAL_LPUART_MAX
#define RESOURCE_SOFTSERIAL_COUNT SERIAL_SOFTSERIAL_MAX
#define RESOURCE_SERIAL_COUNT (RESOURCE_UART_COUNT + RESOURCE_LPUART_COUNT + RESOURCE_SOFTSERIAL_COUNT)
// resources are stored in one array, in UART,LPUART,SOFTSERIAL order. Code does assume this ordering,
//  do not change it without adaptine code.
#define RESOURCE_UART_OFFSET 0
#define RESOURCE_LPUART_OFFSET RESOURCE_UART_COUNT
#define RESOURCE_SOFTSERIAL_OFFSET (RESOURCE_UART_COUNT + RESOURCE_LPUART_COUNT)

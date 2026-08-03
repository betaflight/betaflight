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

#include <stdint.h>

// Minimal poll-mode UART trace for bring-up debug on platforms where SWD
// or USB-VCP isn't usable (e.g. Secured-lifecycle STM32N6 chips). Off by
// default; enable per-board via `ENABLE_DEBUG_UART 1` plus the pin /
// peripheral selectors below. When disabled the API compiles to no-ops
// so call sites can stay in place.
//
// Required per-board defines when ENABLE_DEBUG_UART is 1:
//   DEBUG_UART       CMSIS instance pointer  (e.g. UART7, USART1)
//   DEBUG_UART_GPIO  CMSIS GPIO pointer      (e.g. GPIOE)
//   DEBUG_UART_PIN   TX pin number 0..15
//   DEBUG_UART_AF    Alt-function number 0..15 for the chosen pin
//
// Optional:
//   DEBUG_UART_BAUD  default 115200

#ifndef ENABLE_DEBUG_UART
#define ENABLE_DEBUG_UART 0
#endif

#if ENABLE_DEBUG_UART

void debugUartInit(void);
void debugUartPutc(char c);
void debugUartPuts(const char *s);
void debugUartPutHex8(uint8_t v);
void debugUartPutHex32(uint32_t v);

#else

#define debugUartInit()         ((void)0)
#define debugUartPutc(c)        ((void)(c))
#define debugUartPuts(s)        ((void)(s))
#define debugUartPutHex8(v)     ((void)(v))
#define debugUartPutHex32(v)    ((void)(v))

#endif

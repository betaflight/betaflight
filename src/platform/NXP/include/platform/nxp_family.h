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

// NXP family capability macros, one block per family.

#if defined(IMXRT10XX)

#define NXP_GPIO_PORT_COUNT         5   // GPIO1..GPIO5 (GPIO6..9 alias the same pads)
#define NXP_LPUART_COUNT            8

#define NXP_EDMA_CHANNEL_COUNT      32
#define NXP_HAS_CLASSIC_ADC         1
#define NXP_HAS_FLEXIO_UART         0

// Boot ROM API table pointer, at a fixed address on the 10xx parts.
#define IMXRT_BOOTLOADER_TREE_ADDR  0x0020001cu

#elif defined(IMXRT11XX)

// GPIO1..GPIO6 are ports A..F and GPIO13 is port G; GPIO7..GPIO12 alias GPIO1..GPIO6.
#define NXP_GPIO_PORT_COUNT         7
#define NXP_LPUART_COUNT            12

#define NXP_EDMA_CHANNEL_COUNT      32
#define NXP_HAS_CLASSIC_ADC         0

// The 108 MHz DShot root misses 115200, so a PIOUART needs a FlexIO instance without motors.
#define NXP_HAS_FLEXIO_UART         1
#define NXP_FLEXIO_UART_CLOCK_HZ    48000000u

// The ROM API table address depends on silicon revision; see romapi_imxrt.c.

#else
#error "Unsupported NXP family: define IMXRT10XX or IMXRT11XX (or add the family block here)"
#endif

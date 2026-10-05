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

#if defined(IMX95)

// The Cortex-M7 of the i.MX95. GPIO1..GPIO5 are RGPIO ports A..E; clocks and
// pads belong to the System Manager and are asked for over SCMI.
#define NXP_GPIO_PORT_COUNT         5
#define NXP_LPUART_COUNT            8

#define NXP_HAS_RGPIO               1
#define NXP_HAS_SAR_ADC             1
#define NXP_HAS_TPM                 1
#define NXP_HAS_EDMA5               1
#define NXP_CLOCKS_OVER_SCMI        1
#define NXP_HAS_CLASSIC_ADC         0
// FlexIO timers can count the clock divided by 16 or 256, which UART rates need.
// A board opts in to the FlexIO UART with USE_PIOUART0 in its config.
#define NXP_HAS_FLEXIO_UART         1
// Each LPSPI SCK phase must be within 3 ns of half the period, which only an even SCKDIV gives.
#define NXP_LPSPI_EVEN_SCKDIV       1

// Ownership defaults; a board config narrows them.

// The SCMI mailbox the System Manager answers this agent on.
#ifndef NXP_SCMI_MU
#define NXP_SCMI_MU                 MU5_MUA
#endif

// Bit n set lets the M7 drive LPUARTn. An instance the TRDC does not grant the M7 faults on
// access, so the owned list is a board fact.
#ifndef NXP_LPUART_INSTANCE_MASK
#define NXP_LPUART_INSTANCE_MASK    0x1fe
#endif

// Bit n set lets the M7 drive TPMn.
#ifndef NXP_TPM_INSTANCE_MASK
#define NXP_TPM_INSTANCE_MASK       0x7e
#endif

// The eDMA2 channels the M7 owns, a contiguous range.
#ifndef NXP_EDMA2_CHANNEL_FIRST
#define NXP_EDMA2_CHANNEL_FIRST     0
#endif
#ifndef NXP_EDMA2_CHANNEL_COUNT
#define NXP_EDMA2_CHANNEL_COUNT     (64 - NXP_EDMA2_CHANNEL_FIRST)
#endif

// Bit n set lets the M7 use eDMA3 channel n, which serves only request n.
#ifndef NXP_EDMA3_CHANNEL_MASK
#define NXP_EDMA3_CHANNEL_MASK      0x7fffffffUL
#endif

// The FlexIO instance DShot runs on, 1 or 2.
#ifndef NXP_DSHOT_FLEXIO_INSTANCE
#define NXP_DSHOT_FLEXIO_INSTANCE   1
#endif

// The rate the DShot FlexIO root is moved to at init; 0 leaves the root as the
// System Manager set it. The DShot bit rates must divide it to within one percent.
#ifndef NXP_DSHOT_FLEXIO_CLOCK_HZ
#define NXP_DSHOT_FLEXIO_CLOCK_HZ   133333333u
#endif

// The rate every LPSPI root is moved to at its first enable, at most 133.33 MHz; 0
// leaves the roots as the System Manager set them.
#ifndef NXP_LPSPI_CLOCK_HZ
#define NXP_LPSPI_CLOCK_HZ          133333333u
#endif

#elif defined(IMXRT10XX)

#define NXP_GPIO_PORT_COUNT         5   // GPIO1..GPIO5 (GPIO6..9 alias the same pads)
#define NXP_LPUART_COUNT            8

#define NXP_EDMA_CHANNEL_COUNT      32
#define NXP_HAS_EDMA5               0
#define NXP_HAS_CLASSIC_ADC         1
#define NXP_HAS_FLEXIO_UART         0

// Boot ROM API table pointer, at a fixed address on the 10xx parts.
#define IMXRT_BOOTLOADER_TREE_ADDR  0x0020001cu

#elif defined(IMXRT11XX)

// GPIO1..GPIO6 are ports A..F and GPIO13 is port G; GPIO7..GPIO12 alias GPIO1..GPIO6.
#define NXP_GPIO_PORT_COUNT         7
#define NXP_LPUART_COUNT            12

#define NXP_EDMA_CHANNEL_COUNT      32
#define NXP_HAS_EDMA5               0
#define NXP_HAS_CLASSIC_ADC         0

// The 108 MHz DShot root misses 115200, so a PIOUART needs a FlexIO instance without motors.
#define NXP_HAS_FLEXIO_UART         1
#define NXP_FLEXIO_UART_CLOCK_HZ    48000000u

// The ROM API table address depends on silicon revision; see romapi_imxrt.c.

#else
#error "Unsupported NXP family: define IMX95, IMXRT10XX or IMXRT11XX (or add the family block here)"
#endif

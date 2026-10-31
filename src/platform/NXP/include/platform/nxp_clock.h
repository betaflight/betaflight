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

/*
 * Peripheral clock gates and rates. The family clock file in startup/ sets the
 * source and divider of every root; drivers only open gates and read rates.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Instances of one peripheral are contiguous and in order, so
// NXP_CLK_LPSPI1 + device names a device's clock.
typedef enum {
    NXP_CLK_LPUART1 = 0,
    NXP_CLK_LPUART2,
    NXP_CLK_LPUART3,
    NXP_CLK_LPUART4,
    NXP_CLK_LPUART5,
    NXP_CLK_LPUART6,
    NXP_CLK_LPUART7,
    NXP_CLK_LPUART8,
#if defined(IMXRT11XX)
    NXP_CLK_LPUART9,
    NXP_CLK_LPUART10,
    NXP_CLK_LPUART11,
    NXP_CLK_LPUART12,
#endif

    NXP_CLK_LPSPI1,
    NXP_CLK_LPSPI2,
    NXP_CLK_LPSPI3,
    NXP_CLK_LPSPI4,
#if defined(IMXRT11XX)
    NXP_CLK_LPSPI5,
    NXP_CLK_LPSPI6,
#endif

    NXP_CLK_LPI2C1,
    NXP_CLK_LPI2C2,
    NXP_CLK_LPI2C3,
    NXP_CLK_LPI2C4,
#if defined(IMXRT11XX)
    NXP_CLK_LPI2C5,
    NXP_CLK_LPI2C6,
#endif

    NXP_CLK_FLEXIO1,
    NXP_CLK_FLEXIO2,

    NXP_CLK_FLEXPWM1,
    NXP_CLK_FLEXPWM2,
    NXP_CLK_FLEXPWM3,
    NXP_CLK_FLEXPWM4,

    NXP_CLK_ADC1,
    NXP_CLK_ADC2,

    NXP_CLK_CAN1,
    NXP_CLK_CAN2,
    NXP_CLK_CAN3,

    NXP_CLK_GPT1,
    NXP_CLK_GPT2,
#if defined(IMXRT11XX)
    NXP_CLK_GPT3,
    NXP_CLK_GPT4,
    NXP_CLK_GPT5,
    NXP_CLK_GPT6,
#endif

    NXP_CLK_USDHC1,

    NXP_CLK_EDMA,
    NXP_CLK_SNVS,
    NXP_CLK_USBOH3,

    // The FlexPWM external clock path. Both are gate only, with no rate.
    NXP_CLK_TMR3,
    NXP_CLK_XBARA1,

    NXP_CLK_COUNT
} nxpClock_e;

// Open the clock gate of one peripheral. Calling it again is harmless. False
// means the clock was refused and the caller must not touch the hardware.
bool nxpClockGateOpen(nxpClock_e peripheral);

static inline bool nxpClockGateEnable(nxpClock_e peripheral)
{
    return nxpClockGateOpen(peripheral);
}

// The functional clock rate of one peripheral in hertz, read from the clock
// tree; zero means no usable rate, and the caller must not use the hardware.
uint32_t nxpClockHz(nxpClock_e peripheral);

#if defined(NXP_FLEXIO_UART_CLOCK_HZ) && SERIAL_PIOUART_COUNT > 0
// Move a FlexIO root to NXP_FLEXIO_UART_CLOCK_HZ, which divides every UART rate
// from 93750 to 2000000 baud within 5 parts per thousand.
void nxpClockFlexioUartRoot(nxpClock_e flexio);
#endif

// Bring up the locked 480 MHz clock and analog supply of the USB PHY.
void imxrtUsbPhyPllInit(void);

#ifdef __cplusplus
}
#endif

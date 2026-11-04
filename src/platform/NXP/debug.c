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

#include "platform.h"
#include "build/debug.h"
#include "drivers/io.h"

#include "iomux_pad_def.h"

// Claim the SWD pads so unusedPinsInit() does not turn them into GPIO inputs.
#if defined(IMXRT11XX)
#define NXP_SWDIO_PIN       GPIO_LPSR_15       // JTAG_MUX_TMS
#define NXP_SWCLK_PIN       GPIO_LPSR_14       // JTAG_MUX_TCK
#elif defined(IMXRT10XX)
#define NXP_SWDIO_PIN       GPIO_AD_B0_06
#define NXP_SWCLK_PIN       GPIO_AD_B0_07
#else
#error "Unsupported i.MXRT family: no SWD pads named"
#endif

void debugInit(void)
{
    IO_t io = IOGetByTag(DEFIO_TAG_E(NXP_SWDIO_PIN));  // SWDIO
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(NXP_SWCLK_PIN));       // SWCLK
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }
}

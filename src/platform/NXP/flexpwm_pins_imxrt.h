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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"

#include "flexpwm_imxrt.h"
#include "iomux_nxp.h"
#include "iomux_signal_nxp.h"

// The FlexPWM A or B output a pad carries, or NULL; pad ownership is checked
// by the caller.
const imxrtFlexpwmPin_t *flexpwmFindPad(ioTag_t tag);

// The register block of a FlexPWM instance; the vendor table leads with a
// null entry, so the instance number indexes it directly.
static inline PWM_Type *flexpwmPadReg(const imxrtFlexpwmPin_t *pad)
{
    static PWM_Type *const instances[] = PWM_BASE_PTRS;

    return instances[pad->instance];
}

static inline bool flexpwmPadIsChannelB(const imxrtFlexpwmPin_t *pad)
{
    return pad->channel == IMXRT_FLEXPWM_CHANNEL_B;
}

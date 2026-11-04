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

// FlexPWM pad search shared by motors, servos and the beeper.

#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "flexpwm_pins_imxrt.h"
#include "iomux_signal_def.h"

const imxrtFlexpwmPin_t *flexpwmFindPad(ioTag_t tag)
{
    const imxrtFlexpwmPin_t *pad = IMXRT_FLEXPWM_PIN(imxrtFlexpwmPads, tag);

    // The X output has no pulse width registers.
    if (!pad || pad->channel == IMXRT_FLEXPWM_CHANNEL_X
        || pad->instance > FLEXPWM_INSTANCE_COUNT) {
        return NULL;
    }

    return pad;
}

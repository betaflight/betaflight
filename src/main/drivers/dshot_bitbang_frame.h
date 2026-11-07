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

#include "common/time.h"

// How long a bitbang output frame may run before it counts as stalled: twice
// the frame, so a frame that is merely late is left to finish. A frame is
// `slots` timer paced DMA beats at `slotRateHz` (DShot600: 51 at 1.8 MHz, so
// ~28 us and a ~56 us limit).
static inline timeDelta_t bbOutputFrameTimeoutUs(uint32_t slots, uint32_t slotRateHz)
{
    return (timeDelta_t)(slots * 1000000U / slotRateHz * 2);
}

// An output frame that was started but whose transfer complete interrupt has
// not arrived within the limit. Its stream will not finish by itself, so the
// next frame must reload it rather than enable it as it stands.
static inline bool bbOutputFrameStalled(bool outputPending, timeUs_t nowUs, timeUs_t sentUs, timeDelta_t timeoutUs)
{
    return outputPending && cmpTimeUs(nowUs, sentUs) > timeoutUs;
}

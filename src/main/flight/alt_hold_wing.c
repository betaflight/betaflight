/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_WING

#include "math.h"

#ifdef USE_ALTITUDE_HOLD

#include "build/debug.h"
#include "common/maths.h"
#include "config/config.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "rx/rx.h"
#include "pg/autopilot.h"

#include "alt_hold.h"

void altHoldReset(void)
{
}

void altHoldInit(void)
{
}

void updateAltHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
}

bool isAltHoldActive(void) {
    return false;
}

#endif // USE_ALTITUDE_HOLD
#endif // USE_WING

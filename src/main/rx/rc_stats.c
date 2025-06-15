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

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RC_STATS

#include "build/debug.h"
#include "fc/core.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "rx/rc_stats.h"

timeUs_t previousTimeUs = 0;
bool throttleEverRaisedAfterArming = false;
uint32_t counter = 0;
uint32_t totalTrottleNumber = 0;
timeUs_t fullThrottleTimeUs = 0;
uint32_t fullThrottleCounter = 0;
int8_t previousThrottlePercent = 0;

void rcStatsUpdate(timeUs_t currentTimeUs)
{
    uint32_t deltaT = cmpTimeUs(currentTimeUs, previousTimeUs);
    previousTimeUs = currentTimeUs;
    const int8_t throttlePercent = calculateThrottlePercent();

    if (ARMING_FLAG(ARMED) && !IS_RC_MODE_ACTIVE(BOXCRASHFLIP) && !throttleEverRaisedAfterArming) {
        if (abs(throttlePercent) >= 15) { // start counting stats if throttle was raised >= 15% after arming
            throttleEverRaisedAfterArming = true;
        }
    }

    if (ARMING_FLAG(ARMED) && !IS_RC_MODE_ACTIVE(BOXCRASHFLIP) && throttleEverRaisedAfterArming) {
        counter++;
        totalTrottleNumber += throttlePercent;

        if (abs(throttlePercent) == 100) {
            fullThrottleTimeUs += deltaT;
            if (abs(previousThrottlePercent) != 100) {
                fullThrottleCounter ++;
            }
        }
    }

    DEBUG_SET(DEBUG_RC_STATS, 0, lrintf(RcStatsGetAverageThrottle()));

    previousThrottlePercent = throttlePercent;
}

uint32_t RcStatsGetFullThrottleCounter(void)
{
    return fullThrottleCounter;
}

timeUs_t RcStatsGetFullThrottleTimeUs(void)
{
    return fullThrottleTimeUs;
}

int8_t RcStatsGetAverageThrottle(void)
{
    return (float)totalTrottleNumber/(float)counter + 0.5f; // rounding
}

void NotifyRcStatsArming(void)
{
    throttleEverRaisedAfterArming = false;
}

#endif // USE_RC_STATS

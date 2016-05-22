/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "rx/rx.h"
#include "io/rc_controls.h"
#include "io/escservo.h"

#include "io/rc_curves.h"

#define PITCH_LOOKUP_LENGTH 7
#define YAW_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 12

static int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];      // lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupYawRC[YAW_LOOKUP_LENGTH];              // lookup table for expo & RC rate YAW
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE
int16_t lookupThrottleRCMid;                         // THROTTLE curve mid point

void generatePitchRollCurve(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < PITCH_LOOKUP_LENGTH; i++) {
        lookupPitchRollRC[i] = (2500 + controlRateConfig->rcExpo8 * (i * i - 25)) * i * (int32_t) controlRateConfig->rcRate8 / 2500;
    }
}

void generateYawCurve(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < YAW_LOOKUP_LENGTH; i++) {
        lookupYawRC[i] = (2500 + controlRateConfig->rcYawExpo8 * (i * i - 25)) * i / 25;
    }
}

void generateThrottleCurve(controlRateConfig_t *controlRateConfig, escAndServoConfig_t *escAndServoConfig)
{
    lookupThrottleRCMid = escAndServoConfig->minthrottle + (int32_t)(escAndServoConfig->maxthrottle - escAndServoConfig->minthrottle) * controlRateConfig->thrMid8 / 100; // [MINTHROTTLE;MAXTHROTTLE]

    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        const int16_t tmp = 10 * i - controlRateConfig->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - controlRateConfig->thrMid8;
        if (tmp < 0)
            y = controlRateConfig->thrMid8;
        lookupThrottleRC[i] = 10 * controlRateConfig->thrMid8 + tmp * (100 - controlRateConfig->thrExpo8 + (int32_t) controlRateConfig->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = escAndServoConfig->minthrottle + (int32_t) (escAndServoConfig->maxthrottle - escAndServoConfig->minthrottle) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}

int16_t rcLookupPitchRoll(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    return lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
}

int16_t rcLookupYaw(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    return (lookupYawRC[tmp2] + (tmp - tmp2 * 100) * (lookupYawRC[tmp2 + 1] - lookupYawRC[tmp2]) / 100);
}

int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
}

int16_t rcLookupThrottleMid(void)
{
    return lookupThrottleRCMid;
}

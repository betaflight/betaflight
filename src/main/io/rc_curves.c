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

#include "config/parameter_group.h"

#include "rx/rx.h"
#include "io/rate_profile.h"
#include "io/rc_controls.h"
#include "io/motor_and_servo.h"

#include "io/rc_curves.h"

int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupYawRC[YAW_LOOKUP_LENGTH];     // lookup table for expo & RC rate YAW
int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE


void generatePitchRollCurve()
{
    uint8_t i;

    for (i = 0; i < PITCH_LOOKUP_LENGTH; i++)
        lookupPitchRollRC[i] = (2500 + currentControlRateProfile->rcExpo8 * (i * i - 25)) * i * (int32_t) currentControlRateProfile->rcRate8 / 2500;
}

void generateYawCurve()
{
    uint8_t i;

    for (i = 0; i < YAW_LOOKUP_LENGTH; i++)
        lookupYawRC[i] = (2500 + currentControlRateProfile->rcYawExpo8 * (i * i - 25)) * i / 25;
}

void generateThrottleCurve()
{
    uint8_t i;

    for (i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - currentControlRateProfile->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - currentControlRateProfile->thrMid8;
        if (tmp < 0)
            y = currentControlRateProfile->thrMid8;
        lookupThrottleRC[i] = 10 * currentControlRateProfile->thrMid8 + tmp * (100 - currentControlRateProfile->thrExpo8 + (int32_t) currentControlRateProfile->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = motorAndServoConfig()->minthrottle + (int32_t) (motorAndServoConfig()->maxthrottle - motorAndServoConfig()->minthrottle) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}

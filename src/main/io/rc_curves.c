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

#include "config/config.h"

/*
    maps input from [-500, 500] to [-500, 500]*(rate/100) with expo
    max expo (100) is cubic, i.e., x^3
*/
int16_t rcLookup(int32_t input, uint8_t expo, uint8_t rate)
{
    float inputf = (float)input / 500.0f;
    float expof = (float)expo / 100.0f;

    float expoAppliedInputf = inputf * ((1.0f - expof) + expof * inputf * inputf);

    return (int16_t)(expoAppliedInputf * 5.0f * (float)rate);
}

/*
    maps input from [0, 1000] to [min throttle, max throttle] with expo
*/
int16_t rcLookupThrottle(int32_t input, uint8_t expo, controlRateConfig_t *controlRateConfig, escAndServoConfig_t *escAndServoConfig)
{
    float inputf = (float)input / 1000.0f; //[0, 1] where 0 = 0% thr and 1 = 100% thr

    if (controlRateConfig->thrExpoMethod == THR_EXPO_NO_EXPO || expo == 0) {
        //no expo, we use ints for min/max throttle for speed

        int16_t maxThrottle = escAndServoConfig->maxthrottle;
        int16_t minThrottle;
        if (feature(FEATURE_3D && IS_RC_MODE_ACTIVE(BOX3DDISABLESWITCH))) { //3D, bidirectional
            minThrottle = PWM_RANGE_MIN;
        } else { //normal (non-bidirecitonal)
            minThrottle = escAndServoConfig->minthrottle;
        }

        return minThrottle + (int16_t)((float)(maxThrottle-minThrottle)*inputf);
    } else {
        float expof = (float)expo / 100.0f; //[0, 1] where 0 = no expo and 1 = max expo

        float maxThrottle = escAndServoConfig->maxthrottle;
        float minThrottle;
        if (feature(FEATURE_3D && IS_RC_MODE_ACTIVE(BOX3DDISABLESWITCH))) { //3D, bidirectional
            minThrottle = PWM_RANGE_MIN;
        } else { //normal (non-bidirecitonal)
            minThrottle = escAndServoConfig->minthrottle;
        }

        if (controlRateConfig->thrExpoMethod == THR_EXPO_RC) { //max expo is cubic, i.e., x^3, same as rc
            float expoAppliedInputf = inputf * ((1.0f - expof) + expof * inputf * inputf); //[0, 1] input with expo applied
            return (int16_t)(minThrottle + (maxThrottle - minThrottle) * expoAppliedInputf);
        } else { //THR_EXPO_FLATSPOT, max expo has a flatspot at thrMid
            float thrMidf = (float)controlRateConfig->thrMid8 / 100.0f;
            float thrf = inputf - thrMidf;

            //normalize to [-1, 1]
            if (!(controlRateConfig->thrMid8 == 0 || controlRateConfig->thrMid8 == 100)) {
                thrf /= thrf > 0 ? (1.0f - thrMidf) : thrMidf;
            }

            //apply expo
            thrf *= ((1.0f - expof) + expof * thrf * thrf); //[-1, 1]

            //find mid throttle pwm
            float midThrottle = minThrottle + (maxThrottle - minThrottle) * thrMidf;

            //scale back
            if (thrf >= 0) {
                thrf *= maxThrottle - midThrottle;
            } else {
                thrf *= midThrottle - minThrottle;
            }

            return (int16_t)(midThrottle + thrf);
        }
    }
}

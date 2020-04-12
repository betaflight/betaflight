/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "flight/imu.h"
#include "flight/propwash_control.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#define GRAVITY_IN_THRESHOLD 0.5f
#define GRAVITY_OUT_THRESHOLD 0.8f
#define ROLL_MAX_ANGLE    900
#define PITCH_MAX_ANGLE   650

#define ANTI_PROPWASH_THROTTLE_FILTER_CUTOFF 10
#define THROTTLE_THRESHOLD                   0.01f
#define MAX_DTERM_BOOST                      1.5f
//#define MAX_DLPF_BOOST    0.5f

static pt1Filter_t antiPropwashThrottleLpf;
static float antiPropwashThrottleHpf;

bool isInPropwashZone = false;

void initAntiPropwashThrottleFilter(void) {
    pt1FilterInit(&antiPropwashThrottleLpf, pt1FilterGain(ANTI_PROPWASH_THROTTLE_FILTER_CUTOFF, gyro.targetLooptime * 1e-6f));
}

void checkPropwash(void) {
    const float zAxisAcc = acc.accADC[Z] * acc.dev.acc_1G_rec;

    if (isInPropwashZone == false && zAxisAcc < GRAVITY_IN_THRESHOLD && ABS(attitude.raw[FD_ROLL]) < ROLL_MAX_ANGLE && ABS(attitude.raw[FD_PITCH]) < PITCH_MAX_ANGLE) {
        isInPropwashZone = true;
    } else if (zAxisAcc > GRAVITY_OUT_THRESHOLD) {
        isInPropwashZone = false;
    }
    DEBUG_SET(DEBUG_PROPWASH, 0, lrintf(zAxisAcc * 100));
    DEBUG_SET(DEBUG_PROPWASH, 1, isInPropwashZone);
}

void updateAntiPropwashThrottleFilter(float throttle) {
    antiPropwashThrottleHpf = throttle - pt1FilterApply(&antiPropwashThrottleLpf, throttle);

    DEBUG_SET(DEBUG_PROPWASH, 2, lrintf(antiPropwashThrottleHpf * 1000));
}

bool canApplyBoost(void) {
    bool boost;
    if (isInPropwashZone && antiPropwashThrottleHpf > THROTTLE_THRESHOLD) {
        boost = true;
    } else {
        boost = false;
    }
    DEBUG_SET(DEBUG_PROPWASH, 3, boost);
    return boost;
}

//float computeBoostFactor() {
//    if (canApplyBoost) {
        //const float zAxisAcc = acc.accADC[Z] * acc.dev.acc_1G_rec;
        //const float xAxisAcc = acc.accADC[X] * acc.dev.acc_1G_rec;
        //const float accel = sqrt(powf(zAxisAcc, 2) + powf(xAxisAcc, 2));
        //return MIN(1 + accel * MAX_DTERM_BOOST, 1 + MAX_DTERM_BOOST);
//        DEBUG_SET(DEBUG_PROPWASH, 2, lrintf(MAX_DTERM_BOOST * 100));
//        return MAX_DTERM_BOOST;
//    } else {
//        return 1.0f;
//    }
//}

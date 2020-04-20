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

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#define GRAVITY_IN_THRESHOLD 0.5f
#define GRAVITY_OUT_THRESHOLD 0.9f
#define ROLL_MAX_ANGLE    900
#define PITCH_MAX_ANGLE   650

#define ANTI_PROPWASH_THROTTLE_FILTER_CUTOFF 5

PG_REGISTER_WITH_RESET_TEMPLATE(propwashControlConfig_t, propwashControlConfig, PG_PROPWASH_CONTROL, 1);

PG_RESET_TEMPLATE(propwashControlConfig_t, propwashControlConfig,
    .propwash_control_sensitivity = 30,
    .propwash_control_d_boost = 0,
);

static pt1Filter_t antiPropwashThrottleLpf;
static float antiPropwashThrottleHpf;

bool isInPropwash = false;
bool boost = false;
float gForce = 0.0f;

void initAntiPropwashThrottleFilter(void) {
    pt1FilterInit(&antiPropwashThrottleLpf, pt1FilterGain(ANTI_PROPWASH_THROTTLE_FILTER_CUTOFF, gyro.targetLooptime * 1e-6f));
}

void checkPropwash(void) {
    const float zAxisAcc = acc.accADC[Z] * acc.dev.acc_1G_rec;

    gForce = (zAxisAcc > 0) ? sqrtf(sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y])) * acc.dev.acc_1G_rec : 0.0f;

    if (isInPropwash == false && gForce < GRAVITY_IN_THRESHOLD && ABS(attitude.raw[FD_ROLL]) < ROLL_MAX_ANGLE && ABS(attitude.raw[FD_PITCH]) < PITCH_MAX_ANGLE) {
        isInPropwash = true;
    } else if (gForce > GRAVITY_OUT_THRESHOLD || ABS(attitude.raw[FD_ROLL]) > ROLL_MAX_ANGLE) {
        isInPropwash = false;
    }
    DEBUG_SET(DEBUG_PROPWASH, 0, isInPropwash * 1000);
}

void updateAntiPropwashThrottleFilter(float throttle) {
    antiPropwashThrottleHpf = throttle - pt1FilterApply(&antiPropwashThrottleLpf, throttle);

    DEBUG_SET(DEBUG_PROPWASH, 1, lrintf(antiPropwashThrottleHpf * 1000));
}

bool canApplyBoost(void) {

    checkPropwash();
    const float throttleTreshold = propwashControlConfig()->propwash_control_sensitivity / 1000.0f;

    if (isInPropwash && boost == false && antiPropwashThrottleHpf > throttleTreshold) {
        boost = true;
    } else if (antiPropwashThrottleHpf < throttleTreshold / 10.0f) {
        boost = false;
    }
    DEBUG_SET(DEBUG_PROPWASH, 2, boost);
    return boost;
}

float computeBoostFactor() {

    const float boostPercent = propwashControlConfig()->propwash_control_d_boost / 100.0f;

    const float dBoostGainFactor = MAX(1.0f - 1.0f * gForce / GRAVITY_OUT_THRESHOLD, 0.0f);
    const float dBoostGain = boostPercent * dBoostGainFactor + 1.0f;

    return dBoostGain;
}

bool isInPropwashZone(void) {
    return isInPropwash;
}

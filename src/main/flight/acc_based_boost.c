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
#include "flight/acc_based_boost.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#define GRAVITY_IN_THRESHOLD 0.5f
#define GRAVITY_OUT_THRESHOLD 0.9f
#define ROLL_MAX_ANGLE    900
#define PITCH_MAX_ANGLE   750

#define ANTI_PROPWASH_THROTTLE_FILTER_CUTOFF 5

PG_REGISTER_WITH_RESET_TEMPLATE(accBasedBoostConfig_t, accBasedBoostConfig, PG_ACC_BASED_BOOST, 1);

PG_RESET_TEMPLATE(accBasedBoostConfig_t, accBasedBoostConfig,
    .acc_based_boost_sensitivity = 30,
    .acc_based_boost_percent = 0,
);

static pt1Filter_t accBasedBoostThrottleLpf;
static float accBasedBoostThrottleHpf;

bool isInPropwash = false;
bool boost = false;
float gForce = 0.0f;

void initAccBasedBoostThrottleFilter(void) {
    pt1FilterInit(&accBasedBoostThrottleLpf, pt1FilterGain(ANTI_PROPWASH_THROTTLE_FILTER_CUTOFF, gyro.targetLooptime * 1e-6f));
}

bool isNotUpsideDown() {
    return (ABS(attitude.raw[FD_ROLL]) < ROLL_MAX_ANGLE);
}

bool isUpsideDown() {
    return (ABS(attitude.raw[FD_ROLL]) > ROLL_MAX_ANGLE);
}

bool IsNotTooAngulated() {
    return (ABS(attitude.raw[FD_PITCH]) < PITCH_MAX_ANGLE);
}

FAST_CODE_NOINLINE void checkPropwash(void) {
    const float zAxisAcc = acc.accADC[Z] * acc.dev.acc_1G_rec;

    gForce = (zAxisAcc > 0) ? sqrtf(sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y])) * acc.dev.acc_1G_rec : 0.0f;

    if (!isInPropwash && gForce < GRAVITY_IN_THRESHOLD && isNotUpsideDown() && IsNotTooAngulated()) {
        isInPropwash = true;
    } else if (gForce > GRAVITY_OUT_THRESHOLD || isUpsideDown()) {
        isInPropwash = false;
        boost = false;
    }
    DEBUG_SET(DEBUG_ACC_BASED_BOOST, 0, isInPropwash * 1000);
}

FAST_CODE_NOINLINE void updateAccBasedBoostThrottleFilter(float throttle) {
    accBasedBoostThrottleHpf = throttle - pt1FilterApply(&accBasedBoostThrottleLpf, throttle);

    DEBUG_SET(DEBUG_ACC_BASED_BOOST, 1, lrintf(accBasedBoostThrottleHpf * 1000));
}

FAST_CODE_NOINLINE bool canApplyBoost(void) {
    
    checkPropwash();
    const float throttleTreshold = accBasedBoostConfig()->acc_based_boost_sensitivity / 1000.0f;

    if (isInPropwash && !boost && accBasedBoostThrottleHpf > throttleTreshold) {
        boost = true;
    } else if (!isInPropwash || accBasedBoostThrottleHpf < throttleTreshold / 10.0f) {
        boost = false;
    }
    DEBUG_SET(DEBUG_ACC_BASED_BOOST, 2, boost);
    return boost;
}

FAST_CODE_NOINLINE float computeBoost() {
    const float boostPercent = accBasedBoostConfig()->acc_based_boost_percent / 100.0f;
    const float boostGain = MAX(1.0f - 1.0f * gForce / GRAVITY_OUT_THRESHOLD, 0.0f);

    return 1.0f + boostPercent * boostGain;
}

bool isInPropwashZone(void) {
    return isInPropwash;
}

bool isAccBasedBoostEnabled(void) {
    return boost;
}

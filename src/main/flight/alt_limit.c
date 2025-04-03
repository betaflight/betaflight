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

#include "math.h"

 
#ifdef USE_ALTITUDE_LIMIT

#include "build/debug.h"
#include "common/maths.h"
 #include "common/time.h"
#include "config/config.h"
 
#include "sensors/barometer.h"
#include "flight/position.h"
#include "fc/runtime_config.h"
#include "pg/alt_limit.h"
#include "alt_limit.h"

// static const float taskIntervalSeconds = HZ_TO_INTERVAL(ALTLIMIT_TASK_RATE_HZ); // i.e. 0.01 s

typedef struct {
    bool isActive;
    uint8_t mode;  // 0: no limit, 1: scaling throttle, 2: no throttle
    float ceiling;
    float buffer;
    float throttle_factor;
} altLimitState_t;

altLimitState_t altLimit;

void altLimitInit(void)
{
    altLimit.isActive = altLimitConfig()->active;
    altLimit.mode = 0;
    altLimit.ceiling = altLimitConfig()->ceiling;
    altLimit.buffer = altLimitConfig()->buffer;
    altLimit.throttle_factor = 1.0f;
}

bool altLimitWarn(void)
{
    // used only to display warning in OSD if requested but failing
    if (altLimit.mode > 0) {
        return true;
    } else {
        return false;
    }
}

static void altLimitUpdate(void)
{
    float altimeter = 0;
    altimeter = getAltitudeCm() / 100.0f;
    if (altimeter < (altLimit.ceiling - altLimit.buffer)) {
        altLimit.mode = 0;
        altLimit.throttle_factor = 1.0f;
    } else if (altimeter > altLimit.ceiling) {
        altLimit.mode = 2;
        altLimit.throttle_factor = 0.0f;
    } else {
        altLimit.mode = 1;
        altLimit.throttle_factor = 1.0f + (-1.0f * (altimeter - (altLimit.ceiling - altLimit.buffer)) / altLimit.buffer);
    }
}

void updateAltLimit(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);

    if (altLimit.isActive) {
        altLimitUpdate();
    }
}


float getAltLimitedThrottle(float throttle){
    return (throttle * altLimit.throttle_factor);
   //UNUSED(throttle);
   //return 0.0f;
}

bool isAltLimitActive(void){
    return altLimit.isActive;
}

#endif
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

#ifndef USE_WING

#ifdef USE_POSITION_HOLD

#include "math.h"
#include "build/debug.h"
#include "common/maths.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/compass.h"

#include "pg/pos_hold.h"
#include "pos_hold.h"

typedef struct posHoldState_s {
    bool isEnabled;
    bool isControlOk;
    bool areSensorsOk;
    float deadband;
    bool useStickAdjustment;
} posHoldState_t;

static posHoldState_t posHold;

void posHoldInit(void)
{
    posHold.deadband = posHoldConfig()->deadband * 0.01f;
    posHold.useStickAdjustment = posHoldConfig()->deadband;
}

static void posHoldCheckSticks(void)
{
    // if failsafe is active, eg landing mode, don't update the original start point
    if (!failsafeIsActive() && posHold.useStickAdjustment) {
        const bool sticksDeflected = (getRcDeflectionAbs(FD_ROLL) > posHold.deadband) || (getRcDeflectionAbs(FD_PITCH) > posHold.deadband);
        setSticksActiveStatus(sticksDeflected);
    }
}

static bool sensorsOk(void)
{
    if (!STATE(GPS_FIX)) {
        return false;
    }
    if (
#ifdef USE_MAG
        !compassIsHealthy() &&
#endif
        (!posHoldConfig()->posHoldWithoutMag || !canUseGPSHeading)) {
        return false;
    }
    return true;
}

void updatePosHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    // When AUTOPILOT_MODE is active, position holding is handled by the waypoint
    // system (loitering at a waypoint). Don't run separate position control here.
    if (FLIGHT_MODE(AUTOPILOT_MODE)) {
        posHold.isEnabled = false;
        return;
    }
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isEnabled) {
            resetPositionControl(&gpsSol.llh, POSHOLD_TASK_RATE_HZ); // sets target location to current location
            posHold.isControlOk = true;
            posHold.isEnabled = true;
        }
    } else {
        posHold.isEnabled = false;
    }

    if (posHold.isEnabled && posHold.isControlOk) {
        posHold.areSensorsOk = sensorsOk();
        if (posHold.areSensorsOk) {
            posHoldCheckSticks();
            posHold.isControlOk = positionControl(); // false only on sanity check failure
        }
    }
}

bool posHoldFailure(void) {
    // used only to display warning in OSD if requested but failing
    return FLIGHT_MODE(POS_HOLD_MODE) && (!posHold.isControlOk || !posHold.areSensorsOk);
}

#endif // USE_POSITION_HOLD

#endif // !USE_WING

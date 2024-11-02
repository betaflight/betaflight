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

#ifdef USE_POS_HOLD_MODE

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

#include "pos_hold.h"

typedef struct {
    bool posHoldIsOK;
    float deadband;
    bool useStickAdjustment;
} posHoldState_t;

static posHoldState_t posHold;

void posHoldInit(void)
{
    posHold.deadband = posHoldConfig()->pos_hold_deadband / 100.0f;
    posHold.useStickAdjustment = posHoldConfig()->pos_hold_deadband;
    posHold.posHoldIsOK = false;
}

void posHoldCheckSticks(void)
{
    // if failsafe is active, eg landing mode, don't update the original start point 
    if (!failsafeIsActive() && posHold.useStickAdjustment) {
        const bool sticksDeflected = (getRcDeflectionAbs(FD_ROLL) > posHold.deadband) || (getRcDeflectionAbs(FD_PITCH) > posHold.deadband);
        setSticksActiveStatus(sticksDeflected);
    }
}

void posHoldStart(void)
{
    static bool isInPosHoldMode = false;
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!isInPosHoldMode) {
            // start position hold mode
            posHold.posHoldIsOK = true; // true when started, false when autopilot code reports failure
            resetPositionControl(gpsSol.llh); // sets target location to current location
            isInPosHoldMode = true;
        }
    } else {
        // stop position hold mode
        posHold.posHoldIsOK = false;
        isInPosHoldMode = false;
    }
}

bool posHoldStatusChecks(void)
{
    if (!STATE(GPS_FIX)) {
        posHold.posHoldIsOK = false; // cannot continue, display POS_HOLD_FAIL warning in OSD
        return false; 
    }
    if (
#ifdef USE_MAG
        !compassIsHealthy() &&
#endif
        (!posHoldConfig()->pos_hold_without_mag || !canUseGPSHeading)) {
        posHold.posHoldIsOK = false;
        return false; 
    }
    return true;
}

void updatePosHold(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs); 
    // check for enabling Pod Hold, otherwise do as little as possible while inactive
    posHoldStart();
    if (posHold.posHoldIsOK && posHoldStatusChecks()) {
        posHoldCheckSticks();
        posHold.posHoldIsOK = positionControl();
    } else {
        autopilotAngle[AI_PITCH] = 0.0f;
        autopilotAngle[AI_ROLL] = 0.0f;
    }
}

bool posHoldFailure(void) {
    return (FLIGHT_MODE(POS_HOLD_MODE) && !posHold.posHoldIsOK);
}

#endif // USE_POS_HOLD

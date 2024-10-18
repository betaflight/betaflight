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
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/autopilot.h"
#include "rx/rx.h"

#include "pos_hold.h"

posHoldState_t posHold;

void posHoldReset(void)
{
    posHold.posHoldIsOK = true;
    posHold.targetLocation = gpsSol.llh;
    resetPositionControl(posHold.targetLocation);
}

void posHoldInit(void)
{
    posHold.isPosHoldRequested = false;
    posHold.deadband = rcControlsConfig()->pos_hold_deadband / 100.0f;
    posHold.useStickAdjustment = rcControlsConfig()->pos_hold_deadband;
}

void posHoldProcessTransitions(void)
{
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHold.isPosHoldRequested) {
            // initialise relevant values each time position hold starts
            posHoldReset();
            posHold.isPosHoldRequested = true;
        }
    } else {
        posHold.isPosHoldRequested = false;
    }
}

void posHoldUpdateTargetLocation(void)
{
    // if failsafe is active, eg landing mode, don't update the original start point 
    if (!failsafeIsActive()) {
        // otherwise if sticks are not centered, allow start point to be updated
        if (posHold.useStickAdjustment) {
            if ((getRcDeflectionAbs(FD_ROLL) > posHold.deadband) || (getRcDeflectionAbs(FD_PITCH) > posHold.deadband))  {
                // allow user to fly the quad, in angle mode, when sticks are outside the deadband
                // while sticks are outside the deadband,
                // keep updating the home location to the current GPS location each iteration
                posHoldReset();
            }
        }
    }
}

void posHoldNewGpsData(void) {
    if (posHold.isPosHoldRequested) {
        posHoldUpdateTargetLocation();
        if (posHold.posHoldIsOK) {
            posHold.posHoldIsOK = positionControl(posHold.useStickAdjustment, posHold.deadband);
        }
    } else {
        autopilotAngle[AI_PITCH] = 0.0f;
        autopilotAngle[AI_ROLL] = 0.0f;
    }
}

void posHoldUpdate(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs); 
    // check for enabling Alt Hold, otherwise do as little as possible while inactive
    posHoldProcessTransitions();
}

bool posHoldFailure(void) {
    return (posHold.isPosHoldRequested && !posHold.posHoldIsOK);
}

bool allowPosHoldWithoutMag(void) {
    return (posHoldConfig()->pos_hold_without_mag);
}

#endif // USE_POS_HOLD

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

posHoldState_t posHoldState;

void posHoldReset(void)
{
//    resetPositionControl(); - need to add this in position_control.c
    posHoldState.targetLocation = gpsSol.llh;
    posHoldState.targetAdjustRate = 0.0f;
}

void posHoldInit(void)
{
    posHoldState.isPosHoldActive = false;
    posHoldReset();
}

void posHoldProcessTransitions(void) {

    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        if (!posHoldState.isPosHoldActive) {
            posHoldReset();
            posHoldState.isPosHoldActive = true;
        }
    } else {
        posHoldState.isPosHoldActive = false;
    }
}

void posHoldUpdateTargetLocation(void)
{
    // The user can adjust the target position by flying around in level mode
    // We need to provide a big deadband in rc.c

    if (!failsafeIsActive()) {

    // most easily...
    // fly the quad, in angle mode, enabling a deadband via rc.c (?)
    // while sticks are inside the deadband,
    // set the target location to the current GPS location each iteration
    // posHoldState.targetLocation = currentLocation;
        posHoldState.targetLocation = gpsSol.llh;
    }

}

void posHoldUpdate(void)
{
    // check if the user has changed the target altitude using sticks
    if (posHoldConfig()->pos_hold_adjust_rate) {
        posHoldUpdateTargetLocation();
    }

    // run a function in autopilot.c to adjust position
    positionControl(posHoldState.targetLocation);
}

void updatePosHoldState(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs); 

    // check for enabling Alt Hold, otherwise do as little as possible while inactive
    posHoldProcessTransitions();

    if (posHoldState.isPosHoldActive) {
        posHoldUpdate();
    }
}

#endif // USE_POS_HOLD

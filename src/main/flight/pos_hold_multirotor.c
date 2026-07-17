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

#ifdef USE_NAV_MISSION
#include "flight/nav_mission.h"
#endif

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
    posHold.isEnabled = false;
    posHold.isControlOk = true;
    posHold.areSensorsOk = true;
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
#ifdef USE_NAV_MISSION
    // the mission owns its own engagement; it may claim the position engine
    // without the pilot's POS HOLD switch
    navMissionUpdate(currentTimeUs);
    const bool missionControls = navMissionIsControlling();
#else
    const bool missionControls = false;
#endif
    if (FLIGHT_MODE(POS_HOLD_MODE) || missionControls) {
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
#ifdef USE_NAV_MISSION
            if (missionControls) {
                // the mission has its own (larger) stick-abort threshold; don't
                // let trim-level deflection reset the target every cycle and
                // fight the carrot - that starves the mission of authority
                setSticksActiveStatus(false);
            }
#endif
            posHold.isControlOk = positionControl(); // false only on sanity check failure
        }
    }

    if (posHold.isEnabled && (!posHold.isControlOk || !posHold.areSensorsOk)) {
        // positionControl() is no longer being run, so autopilotAngle would
        // hold its last lean forever: decay it to wings-level instead
        positionControlRelax();
    }
}

bool posHoldFailure(void) {
    // used only to display warning in OSD if requested but failing
    return FLIGHT_MODE(POS_HOLD_MODE) && (!posHold.isControlOk || !posHold.areSensorsOk);
}

bool posHoldControlLost(void) {
    // ungated view of the latched sanity/sensor failure, for the mission layer
    return posHold.isEnabled && (!posHold.isControlOk || !posHold.areSensorsOk);
}

void posHoldRetrigger(void) {
    // drop the engine for one cycle: the next updatePosHold() re-runs the
    // enable edge, re-anchoring position control at the current location with
    // a fresh sanity radius and isControlOk = true
    posHold.isEnabled = false;
}

#endif // USE_POSITION_HOLD

#endif // !USE_WING

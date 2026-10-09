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

#ifdef USE_WING

#include "math.h"

#ifdef USE_ALTITUDE_HOLD

#include "build/debug.h"
#include "common/maths.h"
#include "config/config.h"
#include "drivers/time.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/landing_wing.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "flight/position_nav.h"

#include "rx/rx.h"
#include "pg/autopilot.h"
#include "scheduler/scheduler.h"

#include "alt_hold.h"

// Event driven off positionEstimatorUpdate(); without the estimator the task falls back to
// periodic scheduling so that mode entry and exit are still serviced.
#define ALTHOLD_FALLBACK_PERIOD_US (2 * TASK_PERIOD_HZ(ALTHOLD_TASK_RATE_HZ))
#define ALTHOLD_TARGET_LEAD_S       1.0f

typedef struct {
    bool isActive;
    float targetAltitudeCm;
    float targetVelocityCmS;
    float emergencyDescentRateCmS;  // > 0 while an emergency descent has been commanded
    landingWingTouchdown_t touchdown;
} altHoldState_t;

static altHoldState_t altHold;

float altHoldGetClimbRateCmS(void)
{
    return altHoldConfig()->climbRate * 10.0f;
}

void altHoldSetEmergencyDescent(bool active, float rateCmS)
{
    altHold.emergencyDescentRateCmS = active ? rateCmS : 0.0f;
}

// A failsafe or an emergency descent comes down to land where it is.
static bool descending(void)
{
    return failsafeIsActive() || altHold.emergencyDescentRateCmS > 0.0f;
}

static float altHoldMaxClimbRate(void)
{
    if (altHold.emergencyDescentRateCmS > 0.0f) {
        return fmaxf(altHold.emergencyDescentRateCmS, LANDING_WING_DESCENT_MIN_SINK_CMS);
    }
    if (failsafeIsActive()) {
        return fmaxf(altHoldGetClimbRateCmS(), LANDING_WING_DESCENT_MIN_SINK_CMS);
    }
    return altHoldGetClimbRateCmS();
}

static void altHoldReset(void)
{
    resetAltitudeControl();
    altHold.targetAltitudeCm = getAltitudeCmControl();
    altHold.targetVelocityCmS = 0.0f;
}

void altHoldInit(void)
{
    altHold.isActive = false;
    altHoldReset();
}

static void altHoldProcessTransitions(void)
{
    if (FLIGHT_MODE(ALT_HOLD_MODE)) {
        if (!altHold.isActive) {
            altHoldReset();
            altHold.isActive = true;
        }
    } else {
        if (altHold.isActive) {
            resetAltitudeControl();
            landingWingTouchdownReset(&altHold.touchdown);
            autopilotWingClearLimits(AP_WING_LIMITS_DESCENT);
        }
        altHold.isActive = false;
    }
}

// The pitch stick commands a climb rate and the altitude locks where it is centred; the throttle
// stick is not used. Failsafe and an emergency descent ignore the sticks and descend.
static void altHoldUpdateTargetAltitude(timeUs_t taskIntervalUs)
{
    float stickFactor = 0.0f;

    if (descending()) {
        stickFactor = -1.0f;
    } else {
        const float deadband = altHoldConfig()->deadband / 100.0f;
        const float deflection = getRcDeflection(FD_PITCH);
        if (fabsf(deflection) > deadband) {
            // stick forward is nose down
            stickFactor = -copysignf(scaleRangef(fabsf(deflection), deadband, 1.0f, 0.0f, 1.0f), deflection);
        }
    }

    const float maxClimbRate = altHoldMaxClimbRate();
    altHold.targetVelocityCmS = stickFactor * maxClimbRate;
    // the target never runs further ahead of the aircraft than it can catch up with, but may always come back towards it
    const float leadCm = altHold.targetAltitudeCm - getAltitudeCmControl();
    if (fabsf(leadCm) < maxClimbRate * ALTHOLD_TARGET_LEAD_S || leadCm * altHold.targetVelocityCmS < 0.0f) {
        altHold.targetAltitudeCm += altHold.targetVelocityCmS * US_TO_INTERVAL(taskIntervalUs);
    }
}

static void altHoldUpdate(timeUs_t taskIntervalUs)
{
    if (altHoldMaxClimbRate() > 0.0f) {
        altHoldUpdateTargetAltitude(taskIntervalUs);
    }

    float targetAltitudeCm = altHold.targetAltitudeCm;
    float targetAltitudeVelocity = altHold.targetVelocityCmS;
    float velLimitCmS = altHoldMaxClimbRate();
    bool landing = descending();

    // A nav leg owns the vertical channel while one is flying, and states the altitude, rate and
    // rate cap it is commanding right now.
    if (positionNavHasActiveTarget()) {
        const positionNavCommand_t *navCmd = positionNavGetActiveCommand();
        if (navCmd->includeAltitude) {
            targetAltitudeCm = positionNavGetTargetAltitudeCm();
            targetAltitudeVelocity = positionNavGetTargetVelocityCmS().z;
            velLimitCmS = positionNavGetVerticalRateLimitCmS();
            // track it, so alt hold does not revert to a pre-nav target altitude when the leg ends
            altHold.targetAltitudeCm = targetAltitudeCm;
            landing = false;
        }
    }

    bool landed = false;
    if (landing) {
        landed = landingWingDescend(AP_WING_LIMITS_DESCENT, &altHold.touchdown, micros(), -targetAltitudeVelocity);
    } else {
        landingWingTouchdownReset(&altHold.touchdown);
        autopilotWingClearLimits(AP_WING_LIMITS_DESCENT);
    }

    altitudeControl(targetAltitudeCm, taskIntervalUs, targetAltitudeVelocity, velLimitCmS);

    if (landed) {
        disarm(DISARM_REASON_LANDING);
    }
}

bool altHoldGroundContact(void)
{
    return altHold.isActive && altHold.touchdown.contact;
}

#ifdef UNIT_TEST
float altHoldGetTargetAltitudeCm(void)
{
    return altHold.targetAltitudeCm;
}
#endif

bool altHoldUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);

    if (positionEstimatorTakeUpdate(POS_EST_CONSUMER_ALTHOLD)) {
        return true;
    }

    return currentDeltaTimeUs >= ALTHOLD_FALLBACK_PERIOD_US;
}

void updateAltHold(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    altHoldProcessTransitions();

    if (altHold.isActive) {
        altHoldUpdate(autopilotTaskIntervalUs(TASK_PERIOD_HZ(ALTHOLD_TASK_RATE_HZ)));
    }
}

bool isAltHoldActive(void)
{
    return altHold.isActive;
}

#endif // USE_ALTITUDE_HOLD
#endif // USE_WING

/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

// positionNav is multirotor-only; flight plan execution follows suit.
#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)

#include "common/maths.h"
#include "common/time.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/flight_plan_nav.h"
#include "flight/position_estimator.h"
#include "flight/position_nav.h"

#include "io/gps.h"

#include "pg/autopilot.h"
#include "pg/flight_plan.h"

#define FP_MIN_CRUISE_MPS         1.0f
#define FP_FLYBY_COMPLETION_MPS   2.0f
#define FP_FLYOVER_COMPLETION_MPS 0.5f
#define FP_DELAY_MIN_CRUISE_MPS   0.1f

// Sanity limits while a leg is being flown. Progress must improve the best
// distance-to-target by FP_PROGRESS_EPSILON_M within FP_STALL_TIMEOUT_US, and
// the current distance may never exceed the best by FP_FLYAWAY_MARGIN_M.
// The DELAY modifier's cruise floor (0.1 m/s) still clears the stall window.
#define FP_PROGRESS_EPSILON_M     2.0f
#define FP_STALL_TIMEOUT_US       30000000u
#define FP_FLYAWAY_MARGIN_M       20.0f

// Landing descends toward a target far below the current position so vertical
// arrival can never trigger; touchdown detection is what ends the descent.
#define FP_LANDING_TARGET_DEPTH_M 200.0f
#define FP_LANDING_MIN_RATE_MPS   0.3f
// Fallback: start touchdown monitoring even if descent was never observed —
// only near the ground (landing initiated at ground level). At altitude a
// vehicle that cannot descend must keep trying, not disarm mid-air.
#define FP_LANDING_ESTABLISH_TIMEOUT_US 5000000u

static struct {
    flightPlanNavState_e state;
    uint8_t currentIndex;
    timeUs_t holdStartUs;
    uint16_t holdDurationDs;
    bool active;
    float zBiasM;               // estimator Z at engage, so waypoint Z shares the estimator baseline

    // Staged modifiers — populated by drainModifiers(), consumed by the next
    // positional dispatch. altOverride is one-shot; delay arms a timer that
    // clamps cruise during the next leg; yawRateCapDps is stored but not yet
    // consumed (needs autopilot yaw control to land first).
    bool     altOverridePending;
    int32_t  altOverrideCm;
    uint8_t  altOverrideMode;       // 0=Neutral, 1=Climbing, 2=Descending; informational
    bool     delayActive;
    uint16_t delayDurationDs;
    timeUs_t delayEndUs;
    float    yawRateCapDps;

    flightPlanAbortReason_e abortReason;
    bool rescueRequested;

    // Leg progress tracking for stall/flyaway detection
    float bestDistanceToTargetM;
    timeUs_t lastProgressUs;

    // Landing state
    timeUs_t landingStartUs;
    timeUs_t touchdownQuietStartUs;
    bool landingDescentEstablished;
} fp;

static flightPlanWaypointReachedFn reachedListener = NULL;

static void onWaypointReached(void *userData);

static const waypoint_t *currentWaypoint(void)
{
    const flightPlanConfig_t *plan = flightPlanConfig();
    if (fp.currentIndex >= plan->waypointCount) {
        return NULL;
    }
    return &plan->waypoints[fp.currentIndex];
}

static bool computeTargetEnuM(const waypoint_t *wp, vector3_t *out)
{
    gpsLocation_t origin;
    if (!positionEstimatorGetGpsOrigin(&origin)) {
        return false;
    }

    const gpsLocation_t wpLoc = {
        .lat = wp->latitude,
        .lon = wp->longitude,
        .altCm = wp->altitude,
    };

    vector2_t enuCm;
    GPS_distance2d(&origin, &wpLoc, &enuCm);

    // enuCm is a 2-axis horizontal earth-frame vector (efAxis_e); out is 3-axis ENU.
    out->v[ENU_E] = enuCm.v[EF_EAST] * 0.01f;
    out->v[ENU_N] = enuCm.v[EF_NORTH] * 0.01f;
    // Waypoint altitude is AMSL (GPS frame); the estimator's Z is zeroed to
    // whichever source armed it first (baro or GPS). zBiasM is the estimator
    // reading at engage time, which aligns the target Up with the feedback Up.
    out->v[ENU_U] = (wp->altitude - origin.altCm) * 0.01f + fp.zBiasM;
    return true;
}

// Walk fp.currentIndex past any consecutive modifier waypoints, applying their
// effect to staged fp state. Returns the first positional waypoint, or NULL if
// the plan ended (caller transitions to FP_NAV_COMPLETE). Bounded by
// MAX_WAYPOINTS so a pathological all-modifier mission can't spin.
static const waypoint_t *drainModifiers(void)
{
    for (uint8_t guard = 0; guard < MAX_WAYPOINTS; guard++) {
        const waypoint_t *wp = currentWaypoint();
        if (wp == NULL) {
            return NULL;
        }
        switch (wp->type) {
        case WAYPOINT_TYPE_ALT_CHANGE:
            fp.altOverridePending = true;
            fp.altOverrideCm = wp->altitude;
            fp.altOverrideMode = wp->pattern;
            break;
        case WAYPOINT_TYPE_DELAY:
            fp.delayActive = true;
            fp.delayDurationDs = wp->duration;
            fp.delayEndUs = micros() + (uint32_t)wp->duration * 100000u;
            break;
        case WAYPOINT_TYPE_YAW_RATE:
            // Stored only. Hook point: once autopilot_multirotor.c controls
            // yaw, call into a setter here to apply the cap.
            fp.yawRateCapDps = (float)wp->speed;
            break;
        default:
            return wp;
        }
        if (++fp.currentIndex >= flightPlanConfig()->waypointCount) {
            return NULL;
        }
    }
    return NULL;
}

static bool dispatchWaypoint(void)
{
    const waypoint_t *wp = drainModifiers();
    if (wp == NULL) {
        fp.state = FP_NAV_COMPLETE;
        positionNavClearTarget();
        return false;
    }

    // Apply the staged altitude override before computing the ENU target.
    // The override sticks for the whole leg — cleared in advanceToNext() once
    // we move on — so a retry from FP_NAV_IDLE (estimator origin not ready
    // yet) or a delay-expiry re-dispatch (cruise restore) both keep the
    // overridden altitude in place.
    waypoint_t effective = *wp;
    if (fp.altOverridePending) {
        effective.altitude = fp.altOverrideCm;
    }

    vector3_t targetEnuM;
    if (!computeTargetEnuM(&effective, &targetEnuM)) {
        // No GPS origin captured yet — stay IDLE and let flightPlanNavUpdate()
        // retry once the estimator has locked in.
        fp.state = FP_NAV_IDLE;
        return false;
    }

    const autopilotConfig_t *cfg = autopilotConfig();
    const bool isHoldOrLand = (effective.type == WAYPOINT_TYPE_HOLD) || (effective.type == WAYPOINT_TYPE_LAND);
    const float arrivalRadiusM = (isHoldOrLand ? cfg->waypointHoldRadius : cfg->waypointArrivalRadius) * 0.01f;

    float cruiseMps = (effective.speed > 0) ? effective.speed * 0.01f : cfg->maxVelocity * 0.01f;
    if (cruiseMps < FP_MIN_CRUISE_MPS) {
        cruiseMps = FP_MIN_CRUISE_MPS;
    }

    // DELAY: clamp cruise so traversal-time ≈ delay-seconds, scaled by the
    // *remaining* leg (current position → target). targetEnuM is origin →
    // target, so subtracting the current ENU estimate gives the leg vector
    // the position controller is actually going to drive. Skip when the
    // remaining leg is effectively zero to avoid div-by-near-zero.
    if (fp.delayActive) {
        const float delaySec = fp.delayDurationDs * 0.1f;
        const positionEstimate3d_t *est = positionEstimatorGetEstimate();
        const vector3_t legVec = {.v = {
            [ENU_E] = targetEnuM.v[ENU_E] - est->position.v[ENU_E] * 0.01f,
            [ENU_N] = targetEnuM.v[ENU_N] - est->position.v[ENU_N] * 0.01f,
            [ENU_U] = targetEnuM.v[ENU_U] - est->position.v[ENU_U] * 0.01f,
        }};
        const float legLenM = vector3Norm(&legVec);
        if (delaySec > 0.0f && legLenM > 0.1f) {
            const float scaledMps = MAX(FP_DELAY_MIN_CRUISE_MPS, legLenM / delaySec);
            cruiseMps = MIN(cruiseMps, scaledMps);
        }
    }

    // FLYBY: advance with residual velocity so we curve through.
    // FLYOVER/HOLD/LAND: require near-stop to count as arrived.
    const float completionMps = (effective.type == WAYPOINT_TYPE_FLYBY)
        ? FP_FLYBY_COMPLETION_MPS
        : FP_FLYOVER_COMPLETION_MPS;

    positionNavSetTargetEf(&targetEnuM, cruiseMps, arrivalRadiusM,
                           completionMps, true, onWaypointReached, NULL);

    fp.state = FP_NAV_TARGETING;
    fp.holdDurationDs = effective.duration;
    fp.bestDistanceToTargetM = FLT_MAX;
    fp.lastProgressUs = micros();
    return true;
}

static void abortMission(flightPlanAbortReason_e reason)
{
    fp.state = FP_NAV_ABORTED;
    fp.abortReason = reason;
    // Dropping the nav target makes positionControl() fall back to holding
    // position at the current location; the pilot exits via the mode switch.
    positionNavClearTarget();
}

static float distanceToNavTargetM(const positionEstimate3d_t *est)
{
    const positionNavCommand_t *cmd = positionNavGetActiveCommand();
    vector3_t deltaM = {.v = {
        [ENU_E] = cmd->targetPosEfM.v[ENU_E] - est->position.v[ENU_E] * 0.01f,
        [ENU_N] = cmd->targetPosEfM.v[ENU_N] - est->position.v[ENU_N] * 0.01f,
        [ENU_U] = cmd->includeAltitude ? cmd->targetPosEfM.v[ENU_U] - est->position.v[ENU_U] * 0.01f : 0.0f,
    }};
    return vector3Norm(&deltaM);
}

static void checkLegProgress(timeUs_t currentTimeUs, const positionEstimate3d_t *est)
{
    if (!positionNavHasActiveTarget()) {
        return;
    }

    const float distanceM = distanceToNavTargetM(est);

    if (distanceM < fp.bestDistanceToTargetM - FP_PROGRESS_EPSILON_M) {
        fp.bestDistanceToTargetM = distanceM;
        fp.lastProgressUs = currentTimeUs;
        return;
    }

    if (fp.bestDistanceToTargetM != FLT_MAX && distanceM > fp.bestDistanceToTargetM + FP_FLYAWAY_MARGIN_M) {
        abortMission(FP_ABORT_FLYAWAY);
        return;
    }

    if (cmpTimeUs(currentTimeUs, fp.lastProgressUs) >= (timeDelta_t)FP_STALL_TIMEOUT_US) {
        abortMission(FP_ABORT_STALLED);
    }
}

static void startLanding(timeUs_t currentTimeUs)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const vector3_t targetM = {.v = {
        [ENU_E] = est->position.v[ENU_E] * 0.01f,
        [ENU_N] = est->position.v[ENU_N] * 0.01f,
        [ENU_U] = est->position.v[ENU_U] * 0.01f - FP_LANDING_TARGET_DEPTH_M,
    }};

    const float descentMps = MAX(FP_LANDING_MIN_RATE_MPS, autopilotConfig()->landingDescentRate * 0.01f);
    positionNavSetTargetEf(&targetM, descentMps, 1.0f, 0.1f, true, NULL, NULL);

    fp.state = FP_NAV_LANDING;
    fp.landingStartUs = currentTimeUs;
    fp.touchdownQuietStartUs = 0;
    fp.landingDescentEstablished = false;
}

static void updateLanding(timeUs_t currentTimeUs)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float verticalVelocityCmS = est->velocity.v[ENU_U];
    const float commandedDescentCmS = MAX(FP_LANDING_MIN_RATE_MPS * 100.0f, autopilotConfig()->landingDescentRate);

    if (!fp.landingDescentEstablished
        && verticalVelocityCmS < -0.25f * commandedDescentCmS) {
        fp.landingDescentEstablished = true;
    }

    const bool monitorTouchdown = fp.landingDescentEstablished
        || (isBelowLandingAltitude()
            && cmpTimeUs(currentTimeUs, fp.landingStartUs) >= (timeDelta_t)FP_LANDING_ESTABLISH_TIMEOUT_US);
    if (!monitorTouchdown) {
        return;
    }

    // Touchdown: descent has stopped despite being commanded, and the vehicle
    // is not moving vertically. A descent at the commanded rate must never
    // count as quiet, whatever landingVelocityThreshold is configured to.
    const bool descentStopped = verticalVelocityCmS > -0.25f * commandedDescentCmS;
    if (descentStopped && fabsf(verticalVelocityCmS) < autopilotConfig()->landingVelocityThreshold) {
        if (fp.touchdownQuietStartUs == 0) {
            fp.touchdownQuietStartUs = currentTimeUs;
        } else if (cmpTimeUs(currentTimeUs, fp.touchdownQuietStartUs) >= (timeDelta_t)((uint32_t)autopilotConfig()->landingDetectionTime * 100000u)) {
            positionNavClearTarget();
            fp.state = FP_NAV_COMPLETE;
            disarm(DISARM_REASON_LANDING);
        }
    } else {
        fp.touchdownQuietStartUs = 0;
    }
}

static void checkGeofence(timeUs_t currentTimeUs)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    if (cfg->maxDistanceFromHomeM == 0 || !STATE(GPS_FIX_HOME)) {
        return;
    }
    if (GPS_distanceToHome <= cfg->maxDistanceFromHomeM) {
        return;
    }

    if (cfg->geofenceAction == AP_GEOFENCE_RTH) {
        fp.rescueRequested = true;
    } else {
        startLanding(currentTimeUs);
    }
}

static void advanceToNext(void)
{
    // The leg that's ending consumed any staged altitude override; clear it
    // before the next drainModifiers() pass so a new override (or none) is
    // staged cleanly for the upcoming leg.
    fp.altOverridePending = false;

    if (fp.currentIndex + 1 >= flightPlanConfig()->waypointCount) {
        fp.state = FP_NAV_COMPLETE;
        positionNavClearTarget();
        return;
    }
    fp.currentIndex++;
    // If dispatch fails (e.g. origin lost), state falls back to IDLE and the
    // update loop will retry; index has already advanced so we won't replay
    // the previous waypoint.
    dispatchWaypoint();
}

static void onWaypointReached(void *userData)
{
    UNUSED(userData);
    if (!fp.active) {
        return;
    }

    const waypoint_t *wp = currentWaypoint();
    if (wp == NULL) {
        fp.state = FP_NAV_COMPLETE;
        positionNavClearTarget();
        return;
    }

    if (reachedListener) {
        reachedListener(fp.currentIndex);
    }

    if (wp->type == WAYPOINT_TYPE_HOLD && fp.holdDurationDs > 0) {
        fp.state = FP_NAV_HOLDING;
        fp.holdStartUs = micros();
        return;
    }

    // TODO: WAYPOINT_TYPE_LAND — trigger descent/disarm path.
    // TODO: WAYPOINT_PATTERN_ORBIT / FIGURE8 sub-target generation for HOLD.
    advanceToNext();
}

static void clearModifierState(void)
{
    fp.altOverridePending = false;
    fp.altOverrideCm = 0;
    fp.altOverrideMode = 0;
    fp.delayActive = false;
    fp.delayDurationDs = 0;
    fp.delayEndUs = 0;
    fp.yawRateCapDps = 0.0f;
}

void flightPlanNavInit(void)
{
    fp.state = FP_NAV_IDLE;
    fp.currentIndex = 0;
    fp.active = false;
    fp.zBiasM = 0.0f;
    fp.abortReason = FP_ABORT_NONE;
    fp.rescueRequested = false;
    clearModifierState();
}

void flightPlanNavEngage(void)
{
    fp.currentIndex = 0;
    fp.state = FP_NAV_IDLE;
    fp.abortReason = FP_ABORT_NONE;
    clearModifierState();

    // Capture the estimator's current Z so subsequent waypoint altitudes are
    // referenced to the same baseline the position controller uses.
    fp.zBiasM = positionEstimatorGetAltitudeCm() * 0.01f;

    // HOLD waypoints keep the position target live for the hold duration; a
    // new target replaces the previous on advance anyway, so this module
    // never wants positionNav to auto-clear the target on reach.
    positionNavSetAutoClearOnReach(false);

    if (flightPlanConfig()->waypointCount == 0) {
        fp.state = FP_NAV_COMPLETE;
        fp.active = true;
        positionNavClearTarget();
        return;
    }

    fp.active = true;
    // Try to dispatch immediately; if the GPS origin isn't ready yet we stay
    // IDLE and flightPlanNavUpdate() will retry.
    dispatchWaypoint();
}

void flightPlanNavDisengage(void)
{
    fp.active = false;
    fp.state = FP_NAV_IDLE;
    positionNavClearTarget();
}

void flightPlanNavUpdate(timeUs_t currentTimeUs)
{
    if (!fp.active) {
        return;
    }

    // Retry dispatch if we engaged before the estimator had an origin.
    if (fp.state == FP_NAV_IDLE && flightPlanConfig()->waypointCount > 0) {
        dispatchWaypoint();
        return;
    }

    if (fp.state == FP_NAV_LANDING) {
        updateLanding(currentTimeUs);
        return;
    }

    if (fp.state == FP_NAV_TARGETING || fp.state == FP_NAV_HOLDING) {
        if (!positionEstimatorIsValidXY()) {
            abortMission(FP_ABORT_ESTIMATOR);
            return;
        }

        checkGeofence(currentTimeUs);
        if (fp.state == FP_NAV_LANDING || fp.rescueRequested) {
            return;
        }
    }

    // DELAY-window expiry: clear the cap and re-issue the current leg at the
    // unmodified cruise so we don't crawl for the rest of the leg.
    if (fp.delayActive && cmpTimeUs(currentTimeUs, fp.delayEndUs) >= 0) {
        fp.delayActive = false;
        if (fp.state == FP_NAV_TARGETING) {
            dispatchWaypoint();
        }
    }

    if (fp.state == FP_NAV_TARGETING) {
        checkLegProgress(currentTimeUs, positionEstimatorGetEstimate());
    }

    if (fp.state == FP_NAV_HOLDING) {
        const uint32_t holdUs = (uint32_t)fp.holdDurationDs * 100000u;
        const uint32_t elapsedUs = (uint32_t)(currentTimeUs - fp.holdStartUs);
        if (elapsedUs >= holdUs) {
            advanceToNext();
        }
    }
}

bool flightPlanNavIsActive(void)
{
    return fp.active;
}

flightPlanNavState_e flightPlanNavGetState(void)
{
    return fp.state;
}

uint8_t flightPlanNavGetCurrentIndex(void)
{
    return fp.currentIndex;
}

flightPlanAbortReason_e flightPlanNavGetAbortReason(void)
{
    return fp.abortReason;
}

bool flightPlanNavRescueRequested(void)
{
    return fp.rescueRequested;
}

void flightPlanNavClearRescueRequest(void)
{
    fp.rescueRequested = false;
}

void flightPlanNavSetReachedListener(flightPlanWaypointReachedFn fn)
{
    reachedListener = fn;
}

#endif // ENABLE_FLIGHT_PLAN && !USE_WING

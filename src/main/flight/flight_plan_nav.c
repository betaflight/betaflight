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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

// positionNav is multirotor-only; flight plan execution follows suit.
#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)

#include "common/maths.h"
#include "common/time.h"
#include "common/vector.h"

#include "drivers/time.h"

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

    out->x = enuCm.x * 0.01f;
    out->y = enuCm.y * 0.01f;
    // Waypoint altitude is AMSL (GPS frame); the estimator's Z is zeroed to
    // whichever source armed it first (baro or GPS). zBiasM is the estimator
    // reading at engage time, which aligns the target Z with the feedback Z.
    out->z = (wp->altitude - origin.altCm) * 0.01f + fp.zBiasM;
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

    // Apply one-shot altitude override before computing the ENU target.
    waypoint_t effective = *wp;
    if (fp.altOverridePending) {
        effective.altitude = fp.altOverrideCm;
        fp.altOverridePending = false;
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
    // leg length the executor is about to drive. Skip when the leg is
    // effectively zero (hold-to-hold) to avoid div-by-near-zero.
    if (fp.delayActive) {
        const float delaySec = fp.delayDurationDs * 0.1f;
        const float legLenM = vector3Norm(&targetEnuM);
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
    return true;
}

static void advanceToNext(void)
{
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
    clearModifierState();
}

void flightPlanNavEngage(void)
{
    fp.currentIndex = 0;
    fp.state = FP_NAV_IDLE;
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

    // DELAY-window expiry: clear the cap and re-issue the current leg at the
    // unmodified cruise so we don't crawl for the rest of the leg.
    if (fp.delayActive && cmpTimeUs(currentTimeUs, fp.delayEndUs) >= 0) {
        fp.delayActive = false;
        if (fp.state == FP_NAV_TARGETING) {
            dispatchWaypoint();
        }
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

void flightPlanNavSetReachedListener(flightPlanWaypointReachedFn fn)
{
    reachedListener = fn;
}

#endif // ENABLE_FLIGHT_PLAN && !USE_WING

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

static struct {
    flightPlanNavState_e state;
    uint8_t currentIndex;
    timeUs_t holdStartUs;
    uint16_t holdDurationDs;
    bool active;
    float zBiasM;               // estimator Z at engage, so waypoint Z shares the estimator baseline
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

static bool dispatchWaypoint(void)
{
    const waypoint_t *wp = currentWaypoint();
    if (wp == NULL) {
        fp.state = FP_NAV_COMPLETE;
        positionNavClearTarget();
        return false;
    }

    vector3_t targetEnuM;
    if (!computeTargetEnuM(wp, &targetEnuM)) {
        // No GPS origin captured yet — stay IDLE and let flightPlanNavUpdate()
        // retry once the estimator has locked in.
        fp.state = FP_NAV_IDLE;
        return false;
    }

    const autopilotConfig_t *cfg = autopilotConfig();
    const bool isHoldOrLand = (wp->type == WAYPOINT_TYPE_HOLD) || (wp->type == WAYPOINT_TYPE_LAND);
    const float arrivalRadiusM = (isHoldOrLand ? cfg->waypointHoldRadius : cfg->waypointArrivalRadius) * 0.01f;

    float cruiseMps = (wp->speed > 0) ? wp->speed * 0.01f : cfg->maxVelocity * 0.01f;
    if (cruiseMps < FP_MIN_CRUISE_MPS) {
        cruiseMps = FP_MIN_CRUISE_MPS;
    }

    // FLYBY: advance with residual velocity so we curve through.
    // FLYOVER/HOLD/LAND: require near-stop to count as arrived.
    const float completionMps = (wp->type == WAYPOINT_TYPE_FLYBY)
        ? FP_FLYBY_COMPLETION_MPS
        : FP_FLYOVER_COMPLETION_MPS;

    positionNavSetTargetEf(&targetEnuM, cruiseMps, arrivalRadiusM,
                           completionMps, true, onWaypointReached, NULL);

    fp.state = FP_NAV_TARGETING;
    fp.holdDurationDs = wp->duration;
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

void flightPlanNavInit(void)
{
    fp.state = FP_NAV_IDLE;
    fp.currentIndex = 0;
    fp.active = false;
    fp.zBiasM = 0.0f;
}

void flightPlanNavEngage(void)
{
    fp.currentIndex = 0;
    fp.state = FP_NAV_IDLE;

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

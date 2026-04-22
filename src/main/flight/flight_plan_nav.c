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

#if ENABLE_FLIGHT_PLAN

#include "common/time.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "flight/flight_plan_nav.h"
#include "flight/position_estimator.h"
#include "flight/position_nav.h"

#include "io/gps.h"

#include "pg/autopilot.h"
#include "pg/flight_plan.h"

#define FP_MIN_CRUISE_MPS       1.0f
#define FP_FLYBY_COMPLETION_MPS 2.0f
#define FP_FLYOVER_COMPLETION_MPS 0.5f

static struct {
    flightPlanNavState_e state;
    uint8_t currentIndex;
    timeUs_t holdStartUs;
    uint16_t holdDurationDs;
    bool active;
} fp;

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
    out->z = (wp->altitude - origin.altCm) * 0.01f;
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
        // no origin yet; stay idle and retry next engage
        fp.state = FP_NAV_IDLE;
        return false;
    }

    const autopilotConfig_t *cfg = autopilotConfig();
    const float arrivalRadiusM = cfg->waypointArrivalRadius * 0.01f;
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
    fp.currentIndex++;
    if (fp.currentIndex >= flightPlanConfig()->waypointCount) {
        fp.state = FP_NAV_COMPLETE;
        positionNavClearTarget();
        return;
    }
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
        return;
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
}

void flightPlanNavEngage(void)
{
    fp.active = true;
    fp.currentIndex = 0;
    fp.state = FP_NAV_IDLE;

    if (flightPlanConfig()->waypointCount == 0) {
        fp.state = FP_NAV_COMPLETE;
        return;
    }

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

    if (fp.state == FP_NAV_HOLDING) {
        const uint32_t holdUs = (uint32_t)fp.holdDurationDs * 100000u;
        if (cmpTimeUs(currentTimeUs, fp.holdStartUs) >= (timeDelta_t)holdUs) {
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

#endif // ENABLE_FLIGHT_PLAN

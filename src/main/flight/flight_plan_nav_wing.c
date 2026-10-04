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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if ENABLE_FLIGHT_PLAN && defined(USE_WING)

#include "build/debug.h"

#include "common/maths.h"
#include "common/vector.h"

#include "flight/autopilot.h"
#include "flight/flight_plan_nav_wing.h"
#include "flight/imu.h"
#include "flight/landing_wing.h"
#include "flight/position.h"
#include "flight/position_nav.h"

#include "pg/autopilot.h"
#include "pg/autopilot_wing.h"
#include "pg/flight_plan.h"

#define FPW_NO_ARRIVAL_M          -1.0f       // positionNav never completes a leg: the leg is reached here
#define FPW_ANY_SPEED_MPS         1000.0f
#define FPW_CAPTURE_ALTITUDE_M    3.0f
#define FPW_TAKEOFF_LINE_M        2000.0f
#define FPW_TAKEOFF_BANK_DEG      15.0f
#define FPW_COURSE_MIN_SPEED_CMS  500.0f
#define FPW_OVERSHOOT_TIME_S      2.0f

static struct {
    fpWingLeg_t leg;
    vector2_t lineStartEnuM;    // where this leg's line starts
    float altM;                 // the altitude this leg is flown at
    bool reached;               // the leg was reached: the next one starts at reachedEnuM
    vector2_t reachedEnuM;
    float landHeadingDeg;       // the course flown in to a LAND waypoint, < 0 for none
} wing;

static int8_t loiterDirection(void)
{
    return wingTurnSign(autopilotWingConfig()->loiterDirection);
}

static void setTarget(const vector2_t *targetEnuM, float altM, float vertRateMps, float startAltM)
{
    const vector3_t targetM = {{ targetEnuM->x, targetEnuM->y, altM }};
    positionNavSetTargetEf(&targetM, autopilotWingConfig()->cruiseSpeed * 0.1f, FPW_NO_ARRIVAL_M, FPW_ANY_SPEED_MPS, true, NULL, NULL);
    positionNavSetAltitudeArrivalRequired(false);
    positionNavSetVerticalProfile(vertRateMps, startAltM);
    wing.altM = altM;
}

void flightPlanWingLoiterAt(const vector2_t *centreEnuM, float altM, float vertRateMps, float startAltM)
{
    setTarget(centreEnuM, altM, vertRateMps, startAltM);
    positionNavSetTrackLoiter(autopilotWingLoiterRadiusM(), loiterDirection());
}

void flightPlanWingFlyLine(const vector2_t *startEnuM, const vector2_t *endEnuM, float altM, float vertRateMps, float startAltM)
{
    wing.lineStartEnuM = *startEnuM;
    setTarget(endEnuM, altM, vertRateMps, startAltM);
    positionNavSetTrackLine(startEnuM);
}

float flightPlanWingCommandedAltitudeM(void)
{
    return (positionNavHasActiveTarget() ? positionNavGetTargetAltitudeCm() : getAltitudeCmControl()) * 0.01f;
}

void flightPlanWingReset(void)
{
    wing.reached = false;
    landingWingStop();
    autopilotWingClearLimits(AP_WING_LIMITS_LEG);
}

void flightPlanWingDispatchLeg(const fpWingLeg_t *leg)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const vector2_t craftM = positionEstimateHorizontalM(est);
    vector2_t targetM = {{ leg->targetEnuM.v[ENU_E], leg->targetEnuM.v[ENU_N] }};
    const float speedCmS = positionEstimateGroundspeedCmS(est);

    wing.leg = *leg;
    autopilotWingClearLimits(AP_WING_LIMITS_LEG);

    switch (leg->type) {
    case WAYPOINT_TYPE_HOLD:
        if (leg->climbOut && speedCmS > FPW_COURSE_MIN_SPEED_CMS) {
            // a climb-out starts on its circle, turning the loiter's way, rather than spiralling out
            // from the middle of it
            const float offsetM = loiterDirection() * autopilotWingLoiterRadiusM() / speedCmS;
            targetM.x = craftM.x + offsetM * est->velocity.v[ENU_N];
            targetM.y = craftM.y - offsetM * est->velocity.v[ENU_E];
            wing.leg.targetEnuM.v[ENU_E] = targetM.x;
            wing.leg.targetEnuM.v[ENU_N] = targetM.y;
        }
        flightPlanWingLoiterAt(&targetM, leg->targetEnuM.v[ENU_U], leg->vertRateMps, leg->startAltM);
        break;
    case WAYPOINT_TYPE_LAND:
        // A wing's LAND waypoint is at ground level: it loiters over it at the height it arrives at
        // until the landing takes over.
        wing.landHeadingDeg = -1.0f;
        if (wing.reached && !leg->injectedLanding) {
            const vector2_t inM = {{ targetM.x - wing.reachedEnuM.x, targetM.y - wing.reachedEnuM.y }};
            if (vector2Norm(&inM) > autopilotConfig()->waypointArrivalRadius * 0.01f) {
                wing.landHeadingDeg = fmodf(RADIANS_TO_DEGREES(atan2_approx(inM.x, inM.y)) + 360.0f, 360.0f);
            }
        }
        flightPlanWingLoiterAt(&targetM, leg->startAltM, leg->vertRateMps, leg->startAltM);
        break;
    case WAYPOINT_TYPE_TAKEOFF: {
        // climb straight out along the course it is flying, gently banked
        const float courseRad = (speedCmS > FPW_COURSE_MIN_SPEED_CMS)
            ? atan2_approx(est->velocity.v[ENU_E], est->velocity.v[ENU_N])
            : DECIDEGREES_TO_RADIANS(attitude.values.yaw);
        const vector2_t aheadM = {{ craftM.x + FPW_TAKEOFF_LINE_M * sin_approx(courseRad), craftM.y + FPW_TAKEOFF_LINE_M * cos_approx(courseRad) }};
        flightPlanWingFlyLine(&craftM, &aheadM, leg->targetEnuM.v[ENU_U], leg->vertRateMps, leg->startAltM);
        autopilotWingLimits_t limits;
        autopilotWingDefaultLimits(&limits);
        limits.bankLimitDeg = FPW_TAKEOFF_BANK_DEG;
        autopilotWingSetLimits(AP_WING_LIMITS_LEG, &limits);
        break;
    }
    default:
        flightPlanWingFlyLine(wing.reached ? &wing.reachedEnuM : &craftM, &targetM, leg->targetEnuM.v[ENU_U], leg->vertRateMps, leg->startAltM);
        break;
    }
}

static bool isLoiterType(uint8_t type)
{
    return type == WAYPOINT_TYPE_HOLD || type == WAYPOINT_TYPE_LAND;
}

// On the loiter, or turning onto it.
static float loiterCaptureM(void)
{
    return autopilotWingLoiterRadiusM() + autopilotWingL1DistanceM();
}

static bool headingForNext(const positionEstimate3d_t *est, const vector2_t *craftM)
{
    if (!wing.leg.hasNext) {
        return true;
    }
    vector2_t toNextM;
    vector2Sub(&toNextM, &wing.leg.nextEnuM, craftM);
    return est->velocity.v[ENU_E] * toNextM.x + est->velocity.v[ENU_N] * toNextM.y > 0.0f;
}

// The angle the track turns through at the waypoint onto the leg after it, degrees.
static float turnAtWaypointDeg(const vector2_t *legM)
{
    const vector2_t nextLegM = {{ wing.leg.nextEnuM.x - wing.leg.targetEnuM.v[ENU_E], wing.leg.nextEnuM.y - wing.leg.targetEnuM.v[ENU_N] }};
    return RADIANS_TO_DEGREES(vector2Angle(legM, &nextLegM));
}

fpWingEvent_e flightPlanWingUpdateLeg(const positionEstimate3d_t *est, float *progressM)
{
    const vector2_t craftM = positionEstimateHorizontalM(est);
    const vector2_t targetM = {{ wing.leg.targetEnuM.v[ENU_E], wing.leg.targetEnuM.v[ENU_N] }};
    vector2_t toTargetM;
    vector2Sub(&toTargetM, &targetM, &craftM);
    const float distM = vector2Norm(&toTargetM);
    const float altErrorM = fabsf(est->position.v[ENU_U] * 0.01f - wing.altM);
    float turnDistM = 0.0f;
    bool reached;

    switch (wing.leg.type) {
    case WAYPOINT_TYPE_HOLD:
    case WAYPOINT_TYPE_LAND: {
        const float captureM = loiterCaptureM();
        *progressM = fmaxf(distM - captureM, 0.0f) + altErrorM;
        reached = distM < captureM && altErrorM < FPW_CAPTURE_ALTITUDE_M
            && (!wing.leg.climbOut || headingForNext(est, &craftM));
        break;
    }
    case WAYPOINT_TYPE_TAKEOFF:
        *progressM = altErrorM;
        reached = altErrorM < FPW_CAPTURE_ALTITUDE_M;
        break;
    default: {
        vector2_t legM;
        vector2Sub(&legM, &targetM, &wing.lineStartEnuM);
        const float legLengthM = vector2Norm(&legM);
        vector2_t fromStartM;
        vector2Sub(&fromStartM, &craftM, &wing.lineStartEnuM);
        const float toGoM = (legLengthM > 0.0f) ? legLengthM - vector2Dot(&fromStartM, &legM) / legLengthM : 0.0f;
        const float arrivalM = autopilotConfig()->waypointArrivalRadius * 0.01f;
        *progressM = distM;
        vector2_t nextOffsetM;
        vector2Sub(&nextOffsetM, &wing.leg.nextEnuM, &targetM);
        if (wing.leg.hasNext && isLoiterType(wing.leg.nextType) && vector2Norm(&nextOffsetM) < arrivalM) {
            // the loiter about the waypoint takes over as it is turned onto, not over its centre
            reached = distM < loiterCaptureM();
        } else if (wing.leg.type == WAYPOINT_TYPE_FLYBY && wing.leg.hasNext) {
            // starts the turn onto the next leg early enough to roll out on it
            turnDistM = fmaxf(arrivalM, autopilotWingTurnDistanceM(turnAtWaypointDeg(&legM)));
            reached = toGoM <= turnDistM || distM < turnDistM;
        } else {
            // passes over the waypoint, or abeam it
            reached = toGoM <= 0.0f || distM < arrivalM;
        }
        break;
    }
    }

    DEBUG_SET(DEBUG_FLIGHT_PLAN, 5, lrintf(turnDistM));  //!< Turn Distance [unit:m]

    if (!reached) {
        return FPW_FLYING;
    }
    // the next line starts at the waypoint, or where a takeoff's climb ended
    wing.reachedEnuM = (wing.leg.type == WAYPOINT_TYPE_TAKEOFF) ? craftM : targetM;
    wing.reached = true;
    return FPW_REACHED;
}

void flightPlanWingHold(void)
{
    autopilotWingClearLimits(AP_WING_LIMITS_LEG);
    if (!isLoiterType(wing.leg.type)) {
        flightPlanWingLoiterAt(&wing.reachedEnuM, wing.altM, wing.leg.vertRateMps, flightPlanWingCommandedAltitudeM());
    }
}

void flightPlanWingFinish(void)
{
    landingWingStop();
    autopilotWingClearLimits(AP_WING_LIMITS_LEG);
    if (wing.reached) {
        flightPlanWingLoiterAt(&wing.reachedEnuM, wing.altM, wing.leg.vertRateMps, flightPlanWingCommandedAltitudeM());
    } else {
        positionNavClearTarget();
    }
}

void flightPlanWingStartLanding(timeUs_t nowUs)
{
    landingWingSite_t site = {
        .touchdownEnuM = wing.leg.targetEnuM,
        .groundKnown = true,
        .headingDeg = wing.landHeadingDeg,
        .sinkRateMps = wing.leg.vertRateMps,
    };
    if (wing.leg.injectedLanding) {
        const vector2_t fromHomeM = {{ site.touchdownEnuM.v[ENU_E], site.touchdownEnuM.v[ENU_N] }};
        site.touchdownEnuM.v[ENU_U] = landingWingHomeGroundM();
        site.groundKnown = vector2Norm(&fromHomeM) <= landingWingHomeGroundRadiusM();
    }
    landingWingStart(&site, nowUs);
}

float flightPlanWingOvershootM(const positionEstimate3d_t *est)
{
    const float speedMps = positionEstimateGroundspeedCmS(est) * 0.01f;
    return 2.0f * autopilotWingMinTurnRadiusM() + autopilotWingL1DistanceM() + FPW_OVERSHOOT_TIME_S * speedMps;
}

uint16_t flightPlanWingOrbitPeriodDs(void)
{
    const float periodDs = 10.0f * 2.0f * M_PIf * autopilotWingLoiterRadiusM() / (autopilotWingConfig()->cruiseSpeed * 0.1f);
    return (periodDs >= (float)UINT16_MAX) ? UINT16_MAX : (uint16_t)lrintf(periodDs);
}

#endif // ENABLE_FLIGHT_PLAN && USE_WING

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
#include <string.h>

#include "platform.h"

// positionNav is multirotor-only; flight plan execution follows suit.
#if ENABLE_FLIGHT_PLAN && !defined(USE_WING)

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"
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
#ifdef USE_GPS_RESCUE
#include "pg/gps_rescue.h"
#endif
#if ENABLE_RESCUE_PLAN
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#endif

#define FP_MIN_CRUISE_MPS         1.0f
// Legs complete on acceptance-radius entry at any speed: the position
// controller integrates distance error to hold cruise and cannot unwind it
// fast enough to meet a near-stop speed gate at the waypoint (it would orbit
// instead). The post-completion hold-mode braking handles stopping.
#define FP_COMPLETION_ANY_MPS     1000.0f
#define FP_DELAY_MIN_CRUISE_MPS   0.1f

// Sanity limits while a leg is being flown. Progress must improve the best
// distance-to-target by FP_PROGRESS_EPSILON_M within FP_STALL_TIMEOUT_US, and
// the current distance may never exceed the best by the flyaway margin.
// The margin scales with the leg (approach speed, and therefore legitimate
// overshoot, grows with distance-to-target) between fixed bounds.
// The DELAY modifier's cruise floor (0.1 m/s) still clears the stall window.
#define FP_PROGRESS_EPSILON_M     2.0f
#define FP_STALL_TIMEOUT_US       30000000u
#define FP_FLYAWAY_MARGIN_MIN_M   20.0f
#define FP_FLYAWAY_MARGIN_MAX_M   100.0f
#define FP_FLYAWAY_LEG_FRACTION   0.25f

// Approach braking: caps the nav velocity target to sqrt(2*decel*distance) so
// legs decelerate into the waypoint instead of carrying cruise speed into the
// acceptance radius. Deliberately gentle: the position controller tracks
// velocity through an integrator, so the ramp must be slow enough to unwind.
#define FP_APPROACH_DECEL_MPS2    0.3f

// Landing descends toward a target far below the current position so vertical
// arrival can never trigger; touchdown detection is what ends the descent.
#define FP_LANDING_TARGET_DEPTH_M 200.0f
#define FP_LANDING_MIN_RATE_MPS   0.3f
// Fallback: start touchdown monitoring even if descent was never observed —
// only near the ground (landing initiated at ground level). At altitude a
// vehicle that cannot descend must keep trying, not disarm mid-air.
#define FP_LANDING_ESTABLISH_TIMEOUT_US 5000000u

// Injected runtime plans are small synthesised sequences (geofence RTH,
// failsafe rescue); MAVLink-scale plans stay in the PG store.
#define FP_INJECTED_PLAN_MAX 4

#if ENABLE_RESCUE_PLAN
// Legacy PITCH_FORWARD gives up after 15 s of heading recovery
#define FP_RESCUE_HEADING_TIMEOUT_US 15000000u
#endif

static struct {
    flightPlanNavState_e state;
    uint8_t currentIndex;
    timeUs_t holdStartUs;
    uint16_t holdDurationDs;
    bool active;
    float zBiasM;               // estimator-vs-GPS altitude frame offset, captured at engage

    // Staged modifiers — populated by drainModifiers(), consumed by the next
    // positional dispatch. altOverride is one-shot; delay arms a timer that
    // clamps cruise during the next leg; yawRateCapDps caps the autopilot yaw
    // controller from that point in the plan onward.
    bool     altOverridePending;
    int32_t  altOverrideCm;
    uint8_t  altOverrideMode;       // 0=Neutral, 1=Climbing, 2=Descending; informational
    bool     delayActive;
    uint16_t delayDurationDs;
    timeUs_t delayEndUs;
    float    yawRateCapDps;

    flightPlanAbortReason_e abortReason;

    // Injected runtime plan; while injectedCount > 0 it replaces the PG
    // mission as the executor's waypoint source.
    waypoint_t injected[FP_INJECTED_PLAN_MAX];
    uint8_t injectedCount;

#if ENABLE_RESCUE_PLAN
    // Failsafe rescue plan staged before the executor engages; drained by
    // flightPlanNavEngage() in place of the PG mission.
    waypoint_t staged[FP_INJECTED_PLAN_MAX];
    uint8_t stagedCount;
    bool isRescuePlan;          // the active injected plan is a failsafe rescue
    bool rescueHeadingHold;     // pitch-forward heading recovery in progress
    timeUs_t rescueHeadingStartUs;
#endif

    // Leg progress tracking for stall/flyaway detection
    float bestDistanceToTargetM;
    float flyawayMarginM;
    timeUs_t lastProgressUs;

    // Landing state
    timeUs_t landingStartUs;
    timeUs_t touchdownQuietStartUs;
    bool landingDescentEstablished;
} fp;

static flightPlanWaypointReachedFn reachedListener = NULL;

static void onWaypointReached(void *userData);
static void clearModifierState(void);

static uint8_t activePlanCount(void)
{
    return (fp.injectedCount > 0) ? fp.injectedCount : flightPlanConfig()->waypointCount;
}

static const waypoint_t *activePlanWaypoint(uint8_t index)
{
    if (index >= activePlanCount()) {
        return NULL;
    }
    return (fp.injectedCount > 0) ? &fp.injected[index] : &flightPlanConfig()->waypoints[index];
}

static const waypoint_t *currentWaypoint(void)
{
    return activePlanWaypoint(fp.currentIndex);
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
    // whichever source armed it first (baro or GPS). zBiasM is the frame
    // offset between the two, which aligns the target Up with the feedback Up.
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
            fp.yawRateCapDps = (float)wp->speed;
            autopilotSetYawRateLimit(fp.yawRateCapDps);
            break;
        default:
            return wp;
        }
        if (++fp.currentIndex >= activePlanCount()) {
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

    positionNavSetTargetEf(&targetEnuM, cruiseMps, arrivalRadiusM,
                           FP_COMPLETION_ANY_MPS, true, onWaypointReached, NULL);
    positionNavSetAccelLimits(0.0f, FP_APPROACH_DECEL_MPS2);
    // En-route waypoints advance on horizontal arrival; a vehicle that cannot
    // reach the commanded altitude must not orbit forever. HOLD and LAND are
    // station-keeping targets and keep the altitude gate.
    positionNavSetAltitudeArrivalRequired(isHoldOrLand);

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

    if (fp.bestDistanceToTargetM == FLT_MAX) {
        fp.flyawayMarginM = constrainf(distanceM * FP_FLYAWAY_LEG_FRACTION,
                                       FP_FLYAWAY_MARGIN_MIN_M, FP_FLYAWAY_MARGIN_MAX_M);
    }

    if (distanceM < fp.bestDistanceToTargetM - FP_PROGRESS_EPSILON_M) {
        fp.bestDistanceToTargetM = distanceM;
        fp.lastProgressUs = currentTimeUs;
        return;
    }

    if (fp.bestDistanceToTargetM != FLT_MAX && distanceM > fp.bestDistanceToTargetM + fp.flyawayMarginM) {
        abortMission(FP_ABORT_FLYAWAY);
        return;
    }

    if (cmpTimeUs(currentTimeUs, fp.lastProgressUs) >= (timeDelta_t)FP_STALL_TIMEOUT_US) {
        abortMission(FP_ABORT_STALLED);
    }
}

static void startLanding(timeUs_t currentTimeUs, float targetEastM, float targetNorthM)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const vector3_t targetM = {.v = {
        [ENU_E] = targetEastM,
        [ENU_N] = targetNorthM,
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

// Descend at the leg's target (the waypoint itself), not wherever the arrival
// gate tripped. If the position command has been wiped (a position-control
// re-init), a zeroed target would descend at the GPS origin — re-fly the LAND
// waypoint instead.
static void startLandingAtNavTarget(timeUs_t currentTimeUs)
{
    if (!positionNavHasActiveTarget()) {
        dispatchWaypoint();
        return;
    }
    const positionNavCommand_t *cmd = positionNavGetActiveCommand();
    startLanding(currentTimeUs, cmd->targetPosEfM.v[ENU_E], cmd->targetPosEfM.v[ENU_N]);
}

// Geofence RTH response: [fly home at return altitude, land at home]. Return
// altitude never commands an en-route descent (rescue's ALT_MODE_MAX
// behaviour). Falls back to landing in place if the plan can't be injected.
static void injectReturnHomePlan(timeUs_t currentTimeUs)
{
#ifdef USE_GPS_RESCUE
    const int32_t returnAltCm = MAX(GPS_home_llh.altCm + (int32_t)gpsRescueConfig()->returnAltitudeM * 100, gpsSol.llh.altCm);
    const uint16_t speedCmS = gpsRescueConfig()->groundSpeedCmS;
#else
    const int32_t returnAltCm = gpsSol.llh.altCm;
    const uint16_t speedCmS = 0;    // waypoint default: autopilot maxVelocity
#endif
    const waypoint_t plan[] = {
        {
            .latitude = GPS_home_llh.lat,
            .longitude = GPS_home_llh.lon,
            .altitude = returnAltCm,
            .speed = speedCmS,
            .type = WAYPOINT_TYPE_FLYOVER,
        },
        {
            .latitude = GPS_home_llh.lat,
            .longitude = GPS_home_llh.lon,
            .altitude = returnAltCm,
            .speed = speedCmS,
            .type = WAYPOINT_TYPE_LAND,
        },
    };

    if (!flightPlanNavInjectPlan(plan, ARRAYLEN(plan))) {
        const positionEstimate3d_t *est = positionEstimatorGetEstimate();
        startLanding(currentTimeUs, est->position.v[ENU_E] * 0.01f, est->position.v[ENU_N] * 0.01f);
    }
}

#if ENABLE_RESCUE_PLAN
// Failsafe rescue mission: [climb-in-place HOLD at the current position ->
// FLYOVER home -> LAND home]. The leading HOLD gates on altitude arrival
// (dispatchWaypoint sets altitudeArrivalRequired for HOLD), so the climb
// completes before the return leg starts — legacy ATTAIN_ALT semantics.
static uint8_t buildRescuePlan(waypoint_t out[FP_INJECTED_PLAN_MAX])
{
    if (!STATE(GPS_FIX_HOME) || !STATE(GPS_FIX)) {
        return 0;
    }

    const int32_t homeAltCm = GPS_home_llh.altCm;
    const int32_t currentAltCm = gpsSol.llh.altCm;
    const int32_t climbCm = (int32_t)gpsRescueConfig()->initialClimbM * 100;
    const int32_t fixedCm = (int32_t)gpsRescueConfig()->returnAltitudeM * 100;

    int32_t returnAltCm;
    if (GPS_distanceToHome < gpsRescueConfig()->minStartDistM) {
        // Legacy close-range branch: modest headroom rather than a full climb
        returnAltCm = MAX(homeAltCm + 750, currentAltCm + climbCm);
    } else {
        switch (gpsRescueConfig()->altitudeMode) {
        case GPS_RESCUE_ALT_MODE_FIXED:
            returnAltCm = homeAltCm + fixedCm;
            break;
        case GPS_RESCUE_ALT_MODE_CURRENT:
            returnAltCm = currentAltCm + climbCm;
            break;
        case GPS_RESCUE_ALT_MODE_MAX:
        default:
            // Legacy tracks max altitude relative to the arming point; home
            // altitude anchors it back to the AMSL frame waypoints use.
            returnAltCm = homeAltCm + (int32_t)gpsRescueGetMaxAltitudeCm() + climbCm;
            break;
        }
    }
    returnAltCm = MAX(returnAltCm, currentAltCm); // never command an en-route descent

    const uint16_t speedCmS = gpsRescueConfig()->groundSpeedCmS;
    out[0] = (waypoint_t){
        .latitude = gpsSol.llh.lat,
        .longitude = gpsSol.llh.lon,
        .altitude = returnAltCm,
        .type = WAYPOINT_TYPE_HOLD,
    };
    out[1] = (waypoint_t){
        .latitude = GPS_home_llh.lat,
        .longitude = GPS_home_llh.lon,
        .altitude = returnAltCm,
        .speed = speedCmS,
        .type = WAYPOINT_TYPE_FLYOVER,
    };
    out[2] = (waypoint_t){
        .latitude = GPS_home_llh.lat,
        .longitude = GPS_home_llh.lon,
        .altitude = returnAltCm,
        .speed = speedCmS,
        .type = WAYPOINT_TYPE_LAND,
    };
    return 3;
}

bool flightPlanNavStageRescuePlan(void)
{
    waypoint_t plan[FP_INJECTED_PLAN_MAX];
    const uint8_t count = buildRescuePlan(plan);
    if (count == 0) {
        return false;
    }

    if (fp.active) {
        // Already flying (rx-loss during a mission): replace it immediately.
        memcpy(fp.injected, plan, count * sizeof(plan[0]));
        fp.injectedCount = count;
        fp.currentIndex = 0;
        fp.abortReason = FP_ABORT_NONE;
        fp.isRescuePlan = true;
        fp.rescueHeadingHold = false;
        fp.stagedCount = 0;
        clearModifierState();
        dispatchWaypoint();
        return true;
    }

    memcpy(fp.staged, plan, count * sizeof(plan[0]));
    fp.stagedCount = count;
    return true;
}

bool flightPlanNavIsRescuePlanActive(void)
{
    return fp.active && fp.isRescuePlan;
}
#endif // ENABLE_RESCUE_PLAN

static void checkGeofence(timeUs_t currentTimeUs)
{
    // An injected plan is itself the geofence response; evaluating the fence
    // while flying it would re-trigger every cycle.
    if (fp.injectedCount > 0) {
        return;
    }

    const autopilotConfig_t *cfg = autopilotConfig();
    if (cfg->maxDistanceFromHomeM == 0 || !STATE(GPS_FIX_HOME)) {
        return;
    }
    if (GPS_distanceToHome <= cfg->maxDistanceFromHomeM) {
        return;
    }

    if (cfg->geofenceAction == AP_GEOFENCE_RTH) {
        injectReturnHomePlan(currentTimeUs);
    } else {
        const positionEstimate3d_t *est = positionEstimatorGetEstimate();
        startLanding(currentTimeUs, est->position.v[ENU_E] * 0.01f, est->position.v[ENU_N] * 0.01f);
    }
}

static void advanceToNext(void)
{
    // The leg that's ending consumed any staged altitude override; clear it
    // before the next drainModifiers() pass so a new override (or none) is
    // staged cleanly for the upcoming leg.
    fp.altOverridePending = false;

    if (fp.currentIndex + 1 >= activePlanCount()) {
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

    // Listener indices refer to the PG mission; injected-plan progress is
    // meaningless to a MAVLink partner tracking the uploaded plan.
    if (reachedListener && fp.injectedCount == 0) {
        reachedListener(fp.currentIndex);
    }

#if ENABLE_RESCUE_PLAN
    // Rescue climb complete but the IMU heading is untrusted: hold here and
    // pitch forward so GPS course-over-ground can teach the estimator its
    // heading before the return leg (legacy PITCH_FORWARD semantics).
    if (fp.isRescuePlan && fp.currentIndex == 0 && activePlanCount() > 1 && !imuIsHeadingValid()) {
        fp.rescueHeadingHold = true;
        fp.rescueHeadingStartUs = micros();
        pitchForwardOverride(true);
        return;
    }
#endif

    // A LAND duration is a pre-descent loiter; the hold-expiry path starts
    // the descent instead of advancing.
    const bool holdsOnArrival = (wp->type == WAYPOINT_TYPE_HOLD) || (wp->type == WAYPOINT_TYPE_LAND);
    if (holdsOnArrival && fp.holdDurationDs > 0) {
        fp.state = FP_NAV_HOLDING;
        fp.holdStartUs = micros();
        return;
    }

    if (wp->type == WAYPOINT_TYPE_LAND) {
        // Touchdown completes the plan, so a mid-plan LAND is terminal.
        startLandingAtNavTarget(micros());
        return;
    }

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
    autopilotSetYawRateLimit(0.0f);
}

void flightPlanNavInit(void)
{
    fp.state = FP_NAV_IDLE;
    fp.currentIndex = 0;
    fp.active = false;
    fp.zBiasM = 0.0f;
    fp.abortReason = FP_ABORT_NONE;
    fp.injectedCount = 0;
#if ENABLE_RESCUE_PLAN
    fp.stagedCount = 0;
    fp.isRescuePlan = false;
    fp.rescueHeadingHold = false;
#endif
    clearModifierState();
}

void flightPlanNavEngage(void)
{
    fp.currentIndex = 0;
    fp.state = FP_NAV_IDLE;
    fp.abortReason = FP_ABORT_NONE;
    // Engagement always starts the PG mission; an injected plan does not
    // survive a switch cycle (no resume).
    fp.injectedCount = 0;
    clearModifierState();

    // Reconcile the estimator's altitude baseline with the GPS frame waypoint
    // altitudes are computed in. The pure frame offset (estimator reading
    // minus GPS height above origin) is valid at any engagement altitude —
    // a failsafe rescue engages mid-flight, where the raw estimator reading
    // would shift the whole plan up by the current height.
    gpsLocation_t origin;
    if (positionEstimatorGetGpsOrigin(&origin)) {
        fp.zBiasM = positionEstimatorGetAltitudeCm() * 0.01f - (gpsSol.llh.altCm - origin.altCm) * 0.01f;
    } else {
        fp.zBiasM = positionEstimatorGetAltitudeCm() * 0.01f;
    }

    // HOLD waypoints keep the position target live for the hold duration; a
    // new target replaces the previous on advance anyway, so this module
    // never wants positionNav to auto-clear the target on reach.
    positionNavSetAutoClearOnReach(false);

#if ENABLE_RESCUE_PLAN
    fp.isRescuePlan = false;
    fp.rescueHeadingHold = false;
    if (fp.stagedCount > 0) {
        // A staged failsafe rescue plan replaces the PG mission entirely;
        // rescue waypoint 0 is dispatched, never PG waypoint 0.
        memcpy(fp.injected, fp.staged, fp.stagedCount * sizeof(fp.injected[0]));
        fp.injectedCount = fp.stagedCount;
        fp.stagedCount = 0;
        fp.isRescuePlan = true;
        fp.active = true;
        dispatchWaypoint();
        return;
    }
#endif

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
    fp.injectedCount = 0;
#if ENABLE_RESCUE_PLAN
    fp.stagedCount = 0;
    fp.isRescuePlan = false;
    if (fp.rescueHeadingHold) {
        fp.rescueHeadingHold = false;
        pitchForwardOverride(false);
    }
#endif
    clearModifierState();
    positionNavClearTarget();
}

bool flightPlanNavInjectPlan(const waypoint_t *waypoints, uint8_t count)
{
    if (!fp.active || waypoints == NULL || count == 0 || count > FP_INJECTED_PLAN_MAX) {
        return false;
    }

    memcpy(fp.injected, waypoints, count * sizeof(*waypoints));
    fp.injectedCount = count;
    fp.currentIndex = 0;
    fp.abortReason = FP_ABORT_NONE;
#if ENABLE_RESCUE_PLAN
    fp.isRescuePlan = false;
#endif
    clearModifierState();
    // If dispatch fails (GPS origin lost) the state falls back to IDLE and
    // the update loop retries against the injected plan.
    dispatchWaypoint();
    return true;
}

bool flightPlanNavIsInjectedPlanActive(void)
{
    return fp.active && fp.injectedCount > 0;
}

void flightPlanNavUpdate(timeUs_t currentTimeUs)
{
    if (!fp.active) {
        return;
    }

#if ENABLE_RESCUE_PLAN
    if (fp.rescueHeadingHold) {
        // Leg progress/stall checks are meaningless while deliberately
        // pitching forward away from the climb waypoint.
        if (imuIsHeadingValid()) {
            pitchForwardOverride(false);
            fp.rescueHeadingHold = false;
            advanceToNext();
        } else if (cmpTimeUs(currentTimeUs, fp.rescueHeadingStartUs) >= (timeDelta_t)FP_RESCUE_HEADING_TIMEOUT_US) {
            pitchForwardOverride(false);
            fp.rescueHeadingHold = false;
            abortMission(FP_ABORT_HEADING);
        }
        return;
    }
#endif

    // Retry dispatch if we engaged before the estimator had an origin.
    if (fp.state == FP_NAV_IDLE && activePlanCount() > 0) {
        dispatchWaypoint();
        return;
    }

    // The position controller wipes the nav command when it (re)initialises —
    // which happens right after mission engagement, since AUTOPILOT_MODE
    // activates POS_HOLD_MODE and its first update calls resetPositionControl().
    // Re-issue the current leg whenever the dispatched target has been lost.
    if (fp.state == FP_NAV_TARGETING && !positionNavHasActiveTarget()) {
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
        if (fp.state == FP_NAV_LANDING) {
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
            const waypoint_t *wp = currentWaypoint();
            if (wp != NULL && wp->type == WAYPOINT_TYPE_LAND) {
                startLandingAtNavTarget(currentTimeUs);
            } else {
                advanceToNext();
            }
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

void flightPlanNavSetReachedListener(flightPlanWaypointReachedFn fn)
{
    reachedListener = fn;
}

#endif // ENABLE_FLIGHT_PLAN && !USE_WING

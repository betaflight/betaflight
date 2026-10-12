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

#include "build/debug.h"

#if ENABLE_FLIGHT_PLAN

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "flight/alt_hold.h"
#include "flight/autopilot.h"
#include "flight/flight_plan_nav.h"
#ifdef USE_WING
#include "flight/flight_plan_nav_wing.h"
#include "flight/landing_wing.h"
#include "pg/autopilot_wing.h"
#endif
#include "flight/imu.h"
#include "flight/pid.h"
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
// Legs complete on acceptance-radius entry at any speed: an en-route leg is
// flown through, and a point leg leaves its stop to the leg after it or to the
// position hold that takes over. The rescue's first leg, which the return or
// the landing sets off from, gates on its speed itself.
#define FP_COMPLETION_ANY_MPS     1000.0f
#define FP_DELAY_MIN_CRUISE_MPS   0.1f

// Sanity limits while a leg is being flown. Progress must improve the best
// distance-to-target by FP_PROGRESS_EPSILON_M within FP_STALL_TIMEOUT_US, and
// the current distance may never exceed the best by the flyaway margin.
// The margin scales with the leg (approach speed, and therefore legitimate
// overshoot, grows with distance-to-target) between fixed bounds.
// The DELAY modifier's cruise floor (0.1 m/s) still clears the stall window.
#define FP_PROGRESS_EPSILON_M     2.0f
#ifdef USE_WING
// A wing flies a leg it was handed going the wrong way out and round, and may loiter a minute
// climbing to a hold.
#define FP_STALL_TIMEOUT_US       60000000u
#define FP_FLYAWAY_MARGIN_MIN_M   60.0f
#define FP_FLYAWAY_MARGIN_MAX_M   300.0f
// Without a position a wing circles where it is, for this long before the mission gives up.
#define FP_WING_POSITION_LOSS_TIMEOUT_US 30000000u
#else
#define FP_STALL_TIMEOUT_US       30000000u
#define FP_FLYAWAY_MARGIN_MIN_M   20.0f
#define FP_FLYAWAY_MARGIN_MAX_M   100.0f
#endif
#define FP_FLYAWAY_LEG_FRACTION   0.25f
// Attitude takes this long to swing round and stand the craft on its brake; the speed carried
// through it is distance the fence has to allow for on top of the braking distance itself.
#define FP_BRAKE_REVERSAL_S       1.0f
// Where the craft comes to rest is judged on the attitude taking this long to come round onto the
// brake, which it spends still carrying its speed.
#define FP_BRAKE_RESPONSE_S       0.15f
#define FP_BRAKE_MIN_ANGLE_DEG    10.0f   // ap_max_angle's own lower bound, so a zeroed config cannot divide by zero

// Approach braking: caps the nav velocity target to sqrt(2*decel*distance) so
// legs decelerate into the waypoint instead of carrying cruise speed into the
// acceptance radius. Deliberately gentle, so a craft held to a reference
// walked at this velocity arrives slowly enough to stop on the point.
#define FP_APPROACH_DECEL_MPS2    0.3f

// Heading-fault (magnetometer) detector. While the nose is on command and the
// craft is moving, a course-over-ground that disagrees with the fused heading
// past FP_HDG_FAULT_ERR_DEG for FP_HDG_FAULT_TIME_S means the heading estimate
// is wrong — the sideways-flyaway precursor. A real crab angle this large
// cannot hold a leg line at this ground speed, so wind cannot trip it. The trip
// parks the craft wings-level rather than in position hold, which would lean on
// the same bad heading and fly it away.
#define FP_HDG_FAULT_ERR_DEG        70.0f
#define FP_HDG_FAULT_TIME_S         2.0f
#define FP_HDG_FAULT_MIN_SPEED_CMS  300.0f
#define FP_HDG_FAULT_ALIGNED_DEG    30.0f

// Leg-line carrot path tracking (en-route pass-through legs). Tunable
// magnitudes live in autopilotConfig (nav_corner_speed etc.); these are the
// fixed shape limits. Ported from the field-proven tracker in PR #15442.
#define FP_PASS_MAX_M            12.0f   // largest pass-through gate radius
#define FP_CARROT_LEAD_MIN_M     6.0f    // the carrot steers onto the line toward a point at least this far ahead of it
#define FP_GATE_RADIUS_SCALE     1.5f    // gate radius = corner speed (m/s) * this
#define FP_OVERRUN_LAT_M         8.0f    // a fast gate miss still counts within this lateral corridor
#define FP_CARROT_NO_ARRIVAL_M   -1.0f   // acceptance radius sentinel: positionNav never self-completes a carrot leg
// The GPS-fed position estimate steps a metre or two on every fix. Shaping the
// carrot's speed profile directly from it turns that noise into lean and
// throttle twitches at cruise. Filter the measurements that shape the profile
// (not the sensors themselves); gate detection and pre-turn timing stay on the
// raw estimate so a crossing still counts, and the nose still starts swinging,
// the moment they happen.
#define FP_MEAS_FILTER_S         0.20f   // PT1 tau on the along-track position the trapezoid is keyed on
// The craft answers a change in the carrot's velocity this much later: the along-track filter above
// plus the position controller's own velocity lag. The corner brake starts this far ahead of the
// profile so the craft, not just the carrot, crosses the gate at corner speed.
#define FP_CARROT_BRAKE_LAG_S    0.50f

// A leg that points the nose at its target does not translate until the nose is on the leg — the
// legacy rescue's rotate-then-fly-home guarantee, which is why it never flew home tail first.
// Closer than FP_YAW_BEARING_MIN_M there is no meaningful bearing to steer to, so the nose is left
// where it is rather than chased around a point the craft is sitting on.
#define FP_YAW_ALIGN_DEG          30.0f
#define FP_YAW_BEARING_MIN_M      2.0f
// Safety valve on the gate, as the legacy rotate phase had: a nose that will not come round (a
// failed compass, a yaw controller that has stood down) must not leave the craft parked in the air.
// Give up waiting and fly the leg anyway.
#define FP_YAW_ALIGN_TIMEOUT_US   10000000u
// Nor does a leg that brakes the craft to a stop swing the nose until the craft has first slowed
// below this, or that wait has run out: one taking over a moving craft afresh, the rescue climb, and
// a point leg taken over at a gate. Turning it while braking hard sweeps the brake sideways, and a
// craft carried off its line misses the gates it was meant to cross. A carrot carried on through a
// gate sheds speed at its own acceleration, so its nose swings at once.
#define FP_YAW_SWING_MAX_SPEED_MPS 1.5f
// A leg turning in from the craft's own motion sheds speed at the rescue's lean down to this and
// holds it while it comes round, nose following the carrot at this rate, so the craft leaves the turn
// fast enough for the ground course to go on correcting a compass-less heading: the course only does
// so in straight, pitched flight above walking pace, and through the turn the gyro carries it. The
// lean stays capped until the craft's own course is on the waypoint.
#define FP_TURN_IN_MPS             4.0f
#define FP_TURN_IN_RATE_DPS        30.0f
#define FP_TURN_IN_DONE_DEG        15.0f

// Landing descends toward a target held this far below the craft for the whole
// descent, so vertical arrival can never trigger however high it starts;
// touchdown detection is what ends the descent. The leg's descent rate, not this
// depth, sets how fast the craft comes down.
#define FP_LANDING_TARGET_DEPTH_M 200.0f
#define FP_LANDING_MIN_RATE_MPS   0.3f
// Fallback: start touchdown monitoring even if descent was never observed —
// only near the ground (landing initiated at ground level). At altitude a
// vehicle that cannot descend must keep trying, not disarm mid-air.
#define FP_LANDING_ESTABLISH_TIMEOUT_US 5000000u

// Injected runtime plans are small synthesised sequences (geofence RTH,
// failsafe rescue); MAVLink-scale plans stay in the PG store.
#define FP_INJECTED_PLAN_MAX 4

// HOLD patterns chase a time-parametrised carrot around the hold point. The
// angular rate cap keeps small radii flyable (full cruise on the default 2 m
// radius would demand a 2.5 rad/s spin) and holds the rotation slow enough
// for the position->velocity->attitude chain to phase-track: each loop adds
// lag, and a carrot the vehicle cannot angularly follow balloons the pursuit
// orbit far outside the ring. The position-to-velocity gain means the vehicle
// self-paces roughly patternSpeed metres behind the carrot.
#define FP_PATTERN_MAX_RATE_RADS  0.25f
#define FP_PATTERN_UPDATE_US      200000u
// Legs complete on radius entry at any speed, so the hold is entered carrying
// cruise momentum. The pattern waits for the reached command's own braking to
// park the vehicle first — a carrot started at cruise entry speed balloons
// the pursuit orbit far outside the ring. The timeout bounds a windy day.
#define FP_PATTERN_START_SPEED_MPS  1.5f
#define FP_PATTERN_START_TIMEOUT_US 5000000u

// The rescue brakes home at a steady deceleration, slowing from speed^2 / 2a out, so it arrives
// rather than tailing off towards home. It comes to rest this far from home, and inside it the
// descent stops chasing home and comes straight down: wind and a wandering position estimate would
// otherwise walk the last stretch around the landing spot. The craft lands about this far off, so
// it is no wider than that wander.
#define FP_RESCUE_APPROACH_DECEL_MPS2 1.0f
#define FP_RESCUE_LAND_STILL_RADIUS_M 0.3f

// The rescue's first leg stops the craft where it will come to rest, to climb there or, near home,
// to land there. It completes below this ground speed, or once it has been in place this long
// without getting there: a craft that will not quite settle (gusts, a poor compass) must still get
// on with the rescue rather than stall it.
#define FP_RESCUE_STOP_STILL_MPS 0.5f
#define FP_RESCUE_STOP_SETTLE_S  3.0f
// It brakes at no steeper a lean than this. The position controller lets a brake off in step with
// the speed it sheds, so off a steeper one faster than the attitude can follow: the lean swings back
// past level as the craft stops, and the pitch rate spikes.
#define FP_RESCUE_STOP_MAX_ANGLE_DEG 35.0f

#if ENABLE_RESCUE_PLAN
// Legacy PITCH_FORWARD gives up after 15 s of heading recovery
#define FP_RESCUE_HEADING_TIMEOUT_US 15000000u
// The IMU trusts its heading on a filtered confidence while the course is still pulling it in, so the
// pitch-forward carries on until the heading has held this close to the course at speed for a moment.
// Bounded after the trust, as every second at the pitch-forward lean adds about 7 m/s.
#define FP_RESCUE_PITCH_FORWARD_DEG      35.0f
#define FP_RESCUE_HEADING_SETTLE_DEG     5.0f
#define FP_RESCUE_HEADING_SETTLE_CMS     300
#define FP_RESCUE_HEADING_SETTLE_US      500000u
#define FP_RESCUE_HEADING_SETTLE_MAX_US  2000000u
// A pitch-forward that sets off still sliding sideways teaches the IMU a course that is not the nose,
// and with no heading nothing can brake the slide, so at altitude the level climb waits for drag to
// take it out. Five seconds sheds 5 m/s to this on a drag time constant of 3 s; a wind that keeps the
// craft drifting must not hold it there for good.
#define FP_RESCUE_DRIFT_STILL_CMS  100.0f
#define FP_RESCUE_DRIFT_TIMEOUT_US 5000000u
#ifdef USE_WING
#define FP_WING_RESCUE_MIN_CLIMB_CM  500
#endif
#endif

static struct {
    flightPlanNavState_e state;
    uint8_t currentIndex;
    uint8_t pendingStartIndex;  // MISSION_SET_CURRENT while idle; consumed by engage
    timeUs_t holdStartUs;
    timeUs_t holdLastUpdateUs;
    uint64_t holdElapsedUs;
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
    bool rescueBlindClimb;      // heading unknown: climbing level before the pitch-forward
    bool rescueClimbed;         // the blind climb is at altitude, waiting out the drift since rescueClimbedUs
    timeUs_t rescueClimbedUs;
    bool rescueHeadingHold;     // pitch-forward heading recovery in progress
    timeUs_t rescueHeadingStartUs;
    bool rescueHeadingTrusted;  // the IMU trusts its heading now, since rescueHeadingTrustedUs
    timeUs_t rescueHeadingTrustedUs;
    timeUs_t rescueHeadingOffCourseUs;  // last seen off the course
    bool rescueDescentActive;   // altitude-only fallback descent (no executor)
#endif

    // HOLD pattern sub-target generation
    bool patternPending;        // waiting for the arrival braking to settle
    bool patternActive;
    uint8_t patternType;        // waypointPattern_e
    vector3_t patternCentreM;   // hold point, ENU metres
    float patternPhaseRad;
    float patternSpeedMps;      // carrot path speed
    float patternCruiseMps;     // chase speed cap (the leg's cruise)
    timeUs_t patternLastUpdateUs;

    // Leg progress tracking for stall/flyaway detection
    float bestDistanceToTargetM;
    float flyawayMarginM;
    timeUs_t lastProgressUs;

    // Heading-fault detector integrator, and the timestamp the per-cycle dt is
    // derived from (flightPlanNavUpdate's cadence is not a fixed rate).
    float hdgFaultTimeS;
    timeUs_t lastUpdateUs;

    // Leg-line carrot tracking for en-route (FLYOVER/FLYBY) pass-through legs.
    // The executor flies a carrot along the leg line and positionNav commands its
    // velocity; the executor owns gate detection and advancement. State carries
    // across a corner so the profile is continuous (only engage/retry re-anchor it).
    bool      legIsPassGate;    // current leg uses carrot leg-line tracking
    float     legArriveRadiusM; // this leg's arrival radius, resolved at dispatch
    float     legApproachDecelMps2; // brake onto the waypoint at this deceleration, 0 = off
    bool      legValid;         // the leg line is anchored
    vector3_t legTargetEnuM;    // the point this leg flies to (E,N,U metres)
    float     legCruiseMps;     // this leg's cruise cap
    float     legVertRateMps;   // this leg's climb/descent rate
    uint8_t   legYawBehaviour;  // waypointYaw_e for this leg
    float     legYawHoldDeg;    // heading captured at dispatch for WAYPOINT_YAW_HOLD
    bool      legYawGated;      // a face-the-target leg is still swinging the nose onto the leg
    bool      legYawHolding;    // and is station-keeping at the craft meanwhile (point legs)
    timeUs_t  legYawGateStartUs;// when that wait started
    bool      legYawTimedOut;   // the nose never came round: fly the leg regardless
    bool      legYawBraked;     // the nose may swing: a carrot leg was handed over, or the craft has since slowed
    uint8_t   legYawIndex;      // the waypoint the gate state above belongs to
    vector2_t legStartEnuM;     // anchor of the leg line (E,N metres)
    float     carrotSpeedMps;   // slewed carrot speed: the leg's speed profile
    vector2_t carrotEnuM;       // the carrot: where the craft is held to, walked on by positionNav between updates
    vector2_t carrotVelMps;     // and the velocity it is flown at, which the craft is commanded to fly
    bool      carrotValid;      // the carrot above is live; a leg without one anchors it afresh
    bool      turnInRequested;  // the next leg starts from the craft's own motion
    bool      legTurnIn;        // this leg is still coming round onto its line from it, at speed
    bool      legTurnCapped;    // and its lean capped until the craft's course is on the waypoint
    vector2_t legAnchorEnuM;    // the gate just crossed: where the next leg's line starts
    bool      legAnchorValid;
    bool      inPreTurn;        // blending the nose onto the next leg (excluded from the heading-fault check)
    bool      dispatchAfresh;   // the next dispatch re-targets rather than carrying on the leg being flown
    float     alongFiltM;       // PT1-filtered along-track position; shapes the trapezoid
    bool      measFiltValid;

    // Landing state
    float landingRateMps;       // the rate this landing was commanded to descend at
    timeUs_t landingStartUs;
    timeUs_t touchdownQuietStartUs;
    bool landingDescentEstablished;

#ifdef USE_WING
    timeUs_t positionLostUs;    // when the position estimate went, 0 while it is valid
#endif
} fp;

static flightPlanWaypointReachedFn reachedListener = NULL;

static void onWaypointReached(void *userData);
static void clearModifierState(void);
static void clearLegYawState(void);
#ifndef USE_WING
static void updateLegYaw(const positionEstimate3d_t *est);
static bool legNoseOnTarget(const positionEstimate3d_t *est, const vector3_t *targetEnuM);
#endif

#ifdef USE_WING
static void resetWingPlan(void)
{
    fp.positionLostUs = 0;
    flightPlanWingReset();
}
#endif

// A new plan, or a new start in one: nothing of the leg flown before carries over.
static void resetLegState(void)
{
    clearModifierState();
    clearLegYawState();
#ifdef USE_WING
    resetWingPlan();
#endif
}

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

// The next positional (non-modifier) waypoint at or after `from`, or NULL if the
// plan has none left. Modifier records (ALT_CHANGE/DELAY/YAW_RATE) carry no
// coordinates, so the corner geometry and the pass-through/last-leg decision
// must look past them without consuming modifier state (drainModifiers does that).
static uint8_t nextPositionalIndex(uint8_t from)
{
    for (uint8_t i = from; i < activePlanCount(); i++) {
        const waypoint_t *wp = activePlanWaypoint(i);
        if (wp != NULL && wp->type < WAYPOINT_TYPE_ALT_CHANGE) {
            return i;
        }
    }
    return activePlanCount();
}

static const waypoint_t *nextPositionalWaypoint(uint8_t from)
{
    return activePlanWaypoint(nextPositionalIndex(from));
}

static bool isStationKeepingType(uint8_t type)
{
    return type == WAYPOINT_TYPE_HOLD || type == WAYPOINT_TYPE_LAND || type == WAYPOINT_TYPE_TAKEOFF;
}

#ifndef USE_WING
// En-route FLYOVER/FLYBY waypoints (never the last, never station-keeping)
// are pass-through gates flown with leg-line carrot tracking; everything else
// keeps the precise point-target arrival + hold.
// "Last" means no further positional waypoint (trailing modifiers don't count);
// only a leg with a real next waypoint becomes a carved pass-through gate.
// A face-the-target leg is flown as a carrot leg even when it is the last one: the carrot is
// what can be held at the craft while the nose comes round, without the frozen target tripping
// the arrival test.
static bool flownAsCarrotLeg(const waypoint_t *wp, uint8_t index)
{
    const bool isLastWaypoint = (nextPositionalWaypoint(index + 1) == NULL);
    const bool faceTarget = (wp->yawBehaviour == WAYPOINT_YAW_FACE_TARGET);
    return !isStationKeepingType(wp->type) && (!isLastWaypoint || faceTarget);
}
#endif // !USE_WING

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

// Reconcile the estimator's altitude baseline with the GPS frame waypoint
// altitudes are computed in. The pure frame offset (estimator reading
// minus GPS height above origin) is valid at any engagement altitude —
// a failsafe rescue engages mid-flight, where the raw estimator reading
// would shift the whole plan up by the current height. Taken again whenever
// a plan built from this instant's GPS replaces one already flying: the two
// frames drift apart over a flight.
static void captureAltitudeFrame(void)
{
    gpsLocation_t origin;
    if (positionEstimatorGetGpsOrigin(&origin)) {
        fp.zBiasM = positionEstimatorGetAltitudeCm() * 0.01f - (gpsSol.llh.altCm - origin.altCm) * 0.01f;
    } else {
        fp.zBiasM = positionEstimatorGetAltitudeCm() * 0.01f;
    }
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

// The plan is over. Drop the target and hand the nose back: a leg that commanded a heading would
// otherwise keep the autopilot steering to it until the mode is switched off.
static void completePlan(void)
{
    fp.state = FP_NAV_COMPLETE;
#ifdef USE_WING
    flightPlanWingFinish();
#else
    positionNavClearTarget();
#endif
    autopilotSetNavHeadingOverride(false, 0.0f);
}

// The rate this leg climbs or descends at. A waypoint that states one owns it; otherwise a LAND leg
// takes the configured landing descent rate and every other leg the configured alt hold climb rate.
// Resolved once, here: nothing downstream re-derives it from what the executor happens to be doing.
// Zero (wing, which has no configured climb rate) leaves the leg's cruise speed as the bound.
static float legVertRateMps(const waypoint_t *wp)
{
    if (wp->vertRate > 0) {
        return wp->vertRate * 0.01f;
    }
    return ((wp->type == WAYPOINT_TYPE_LAND) ? (float)autopilotConfig()->landingDescentRate
                                             : altHoldGetClimbRateCmS()) * 0.01f;
}

// Where a leg taking over from one still flying starts its altitude ramp: starting at the craft
// steps the altitude target by however far the craft sits off the commanded altitude.
static float commandedAltitudeM(const positionEstimate3d_t *est)
{
    if (positionNavHasActiveTarget() && positionNavGetActiveCommand()->includeAltitude) {
        return positionNavGetTargetAltitudeCm() * 0.01f;
    }
    return est->position.v[ENU_U] * 0.01f;
}

#ifndef USE_WING
static float brakingDecelMps2(float angleDeg)
{
    return G_ACCELERATION * tanf(DEGREES_TO_RADIANS(fmaxf(angleDeg, FP_BRAKE_MIN_ANGLE_DEG)));
}

static float rescueStopMaxAngleDeg(void)
{
    return fminf((float)autopilotConfig()->maxAngle, FP_RESCUE_STOP_MAX_ANGLE_DEG);
}

// Where braking at angleDeg brings the craft to rest on something moving at movingMps.
static vector2_t restPointM(const positionEstimate3d_t *est, const vector2_t *movingMps, float angleDeg)
{
    vector2_t closingMps = {
        .x = est->velocity.v[ENU_E] * 0.01f - movingMps->x,
        .y = est->velocity.v[ENU_N] * 0.01f - movingMps->y,
    };
    const float closingSpeedMps = vector2Norm(&closingMps);
    vector2Scale(&closingMps, &closingMps, closingSpeedMps / (2.0f * brakingDecelMps2(angleDeg)) + FP_BRAKE_RESPONSE_S);
    return (vector2_t){ .x = est->position.v[ENU_E] * 0.01f + closingMps.x,
                        .y = est->position.v[ENU_N] * 0.01f + closingMps.y };
}

// The carrot at carrotM, but never further than the position controller's reach from the craft or
// from where the craft comes to rest on it: a craft that cannot keep up, or is blown off the line,
// drags the carrot along rather than leaving it to run away, and one braking onto it is not pulled
// back from where it stops.
static void placeCarrot(const positionEstimate3d_t *est, const vector2_t *carrotM)
{
    const float reachM = NAV_ERROR_DISTANCE_LIMIT * 0.01f;
    const vector2_t craftM = { .x = est->position.v[ENU_E] * 0.01f, .y = est->position.v[ENU_N] * 0.01f };
    const vector2_t restM = restPointM(est, &fp.carrotVelMps, autopilotConfig()->maxAngle);
    vector2_t fromCraftM;
    vector2_t fromRestM;
    vector2Sub(&fromCraftM, carrotM, &craftM);
    vector2Sub(&fromRestM, carrotM, &restM);
    const bool nearerRest = vector2Norm(&fromRestM) < vector2Norm(&fromCraftM);
    vector2_t gapM = nearerRest ? fromRestM : fromCraftM;
    const float gapLenM = vector2Norm(&gapM);
    if (gapLenM > reachM) {
        vector2Scale(&gapM, &gapM, reachM / gapLenM);
    }
    vector2Add(&fp.carrotEnuM, nearerRest ? &restM : &craftM, &gapM);
}

// positionNav walks the carrot at its velocity between the executor's updates.
static void readBackCarrot(void)
{
    const positionNavCommand_t *cmd = positionNavGetActiveCommand();
    fp.carrotEnuM.x = cmd->targetPosEfM.v[ENU_E];
    fp.carrotEnuM.y = cmd->targetPosEfM.v[ENU_N];
}

// The rate a point leg's commanded velocity ramps at out of the one before it: nav_accel, but never
// slower than its braking curve or its approach sheds speed, or it arrives hot.
static float pointLegAccelMps2(void)
{
    return fmaxf(fmaxf(autopilotConfig()->navAccel * 0.01f, FP_APPROACH_DECEL_MPS2), fp.legApproachDecelMps2);
}
#endif // !USE_WING

static bool isRescueStop(void)
{
#if ENABLE_RESCUE_PLAN
    return fp.isRescuePlan && fp.currentIndex == 0;
#else
    return false;
#endif
}

#if ENABLE_RESCUE_PLAN && !defined(USE_WING)
// Position hold cannot run: it has no heading to turn its commands into lean with.
static bool headingUnknown(void)
{
    return positionEstimatorIsHeadingRequired() && !imuIsHeadingValid();
}
#endif

#if ENABLE_RESCUE_PLAN
// Hands the attitude back to position control from either stage of the heading recovery.
static void endHeadingRecovery(void)
{
    if (fp.rescueBlindClimb || fp.rescueHeadingHold) {
        autopilotHeadingRecovery(false, 0.0f);
    }
    fp.rescueBlindClimb = false;
    fp.rescueClimbed = false;
    fp.rescueHeadingHold = false;
}
#endif

#ifndef USE_WING
// A rescue's first leg with no heading, and a return still to fly: nothing can hold the craft or
// brake it, so it climbs level where it drifts and completes on altitude alone, and the pitch-forward
// that teaches the IMU its heading waits until it is up clear of whatever it started near and the
// drift has died away.
static bool isRescueBlindClimb(void)
{
#if ENABLE_RESCUE_PLAN
    return isRescueStop() && activePlanCount() > 1 && headingUnknown();
#else
    return false;
#endif
}
#endif // !USE_WING

static void legDispatched(uint16_t holdDurationDs)
{
    fp.state = FP_NAV_TARGETING;
    fp.holdDurationDs = holdDurationDs;
    fp.bestDistanceToTargetM = FLT_MAX;
    fp.lastProgressUs = micros();
    fp.hdgFaultTimeS = 0.0f;
}

#ifdef USE_WING
static void dispatchWingLeg(const waypoint_t *wp, const vector3_t *targetEnuM)
{
    fpWingLeg_t leg = {
        .type = wp->type,
        .targetEnuM = *targetEnuM,
        .vertRateMps = legVertRateMps(wp),
        .startAltM = commandedAltitudeM(positionEstimatorGetEstimate()),
        .climbOut = isRescueStop() && wp->type == WAYPOINT_TYPE_HOLD,
        .injectedLanding = fp.injectedCount > 0,
    };
    const waypoint_t *nextWp = nextPositionalWaypoint(fp.currentIndex + 1);
    vector3_t nextEnuM;
    if (nextWp != NULL && computeTargetEnuM(nextWp, &nextEnuM)) {
        leg.hasNext = true;
        leg.nextType = nextWp->type;
        leg.nextEnuM = (vector2_t){{ nextEnuM.v[ENU_E], nextEnuM.v[ENU_N] }};
    }
    flightPlanWingDispatchLeg(&leg);
    fp.legTargetEnuM = *targetEnuM;
    fp.legVertRateMps = leg.vertRateMps;
    legDispatched(wp->duration);
}
#endif

static bool dispatchWaypoint(void)
{
    const waypoint_t *wp = drainModifiers();
    if (wp == NULL) {
        completePlan();
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

#ifdef USE_WING
    dispatchWingLeg(&effective, &targetEnuM);
    return true;
#else

    // TAKEOFF climbs in place: the waypoint's lat/lon are advisory (MAVLink
    // marks them optional) — the target is the current position at the
    // waypoint's altitude.
    if (effective.type == WAYPOINT_TYPE_TAKEOFF) {
        const positionEstimate3d_t *est = positionEstimatorGetEstimate();
        targetEnuM.v[ENU_E] = est->position.v[ENU_E] * 0.01f;
        targetEnuM.v[ENU_N] = est->position.v[ENU_N] * 0.01f;
    }
    const bool blindClimb = isRescueBlindClimb();
    const bool rescueStop = isRescueStop() && !blindClimb;
    if (rescueStop) {
        const vector2_t stillMps = { .x = 0.0f, .y = 0.0f };
        const vector2_t holdM = restPointM(positionEstimatorGetEstimate(), &stillMps, rescueStopMaxAngleDeg());
        targetEnuM.v[ENU_E] = holdM.x;
        targetEnuM.v[ENU_N] = holdM.y;
    }

    const autopilotConfig_t *cfg = autopilotConfig();
    const bool isStationKeeping = isStationKeepingType(effective.type);
    float arrivalRadiusM = isStationKeeping
        ? cfg->waypointHoldRadius * 0.01f
        : fminf(cfg->waypointArrivalRadius * 0.01f, FP_PASS_MAX_M);
#if ENABLE_RESCUE_PLAN && !defined(USE_WING)
    // Legacy rescue begins its descent gps_rescue_descent_dist from home and comes down as it
    // closes the last stretch, rather than arriving overhead and then sinking. The plan gets the
    // same shape by arriving early: the return leg hands over at that distance and the landing leg
    // is already inside its own radius when it is dispatched.
    if (fp.isRescuePlan && !rescueStop
        && (effective.type == WAYPOINT_TYPE_FLYOVER || effective.type == WAYPOINT_TYPE_LAND)) {
        arrivalRadiusM = fmaxf(arrivalRadiusM, (float)gpsRescueConfig()->descentDistanceM);
    }
#endif
    fp.legArriveRadiusM = arrivalRadiusM;
    // Rescue brakes all the way home through the descent; mission legs keep their own trapezoid.
    fp.legApproachDecelMps2 = 0.0f;
#if ENABLE_RESCUE_PLAN
    if (fp.isRescuePlan && !rescueStop
        && (effective.type == WAYPOINT_TYPE_FLYOVER || effective.type == WAYPOINT_TYPE_LAND)) {
        fp.legApproachDecelMps2 = FP_RESCUE_APPROACH_DECEL_MPS2;
    }
#endif

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

    const bool faceTarget = (effective.yawBehaviour == WAYPOINT_YAW_FACE_TARGET);
    const bool passGate = flownAsCarrotLeg(&effective, fp.currentIndex);
    const bool turnIn = fp.turnInRequested && passGate;
    fp.turnInRequested = false;
    fp.legTurnIn = turnIn;

    fp.patternPending = false;
    fp.patternActive = false;
    if (!passGate) {
        fp.carrotValid = false;   // a carrot after a point leg starts afresh, not from a stale gate
        fp.legAnchorValid = false;
    }
    fp.legIsPassGate = passGate;
    fp.legValid = false;               // re-anchor the leg line on the next update
    fp.legTargetEnuM = targetEnuM;
    fp.legCruiseMps = cruiseMps;
    fp.legVertRateMps = legVertRateMps(&effective);
    fp.legYawBehaviour = effective.yawBehaviour;
    fp.legYawHoldDeg = attitude.values.yaw * 0.1f;
    fp.inPreTurn = false;

    const positionEstimate3d_t *dispatchEst = positionEstimatorGetEstimate();
    const float startAltM = commandedAltitudeM(dispatchEst);
    const bool handingOver = positionNavHasActiveTarget() && !fp.dispatchAfresh;

    // Gate state belongs to the waypoint, not to the dispatch: a leg re-issued mid-flight (a
    // position-control re-init, the delay-expiry cruise restore, or the swap from the hold below to
    // the real target) keeps the wait it has already served.
    if (fp.legYawIndex != fp.currentIndex) {
        fp.legYawIndex = fp.currentIndex;
        fp.legYawGateStartUs = micros();
        fp.legYawTimedOut = false;
        fp.legYawBraked = handingOver && passGate;
    }
    fp.legYawGated = faceTarget && !turnIn && !fp.legYawTimedOut && !legNoseOnTarget(dispatchEst, &targetEnuM);
    fp.legYawHolding = fp.legYawGated && !passGate;
    if (!passGate && effective.yawBehaviour == WAYPOINT_YAW_DEFAULT) {
        autopilotSetNavHeadingOverride(false, 0.0f);   // precise legs use the configured yaw mode
    }

    // Where the craft is now, at the leg's altitude: what the face-the-target hold below starts from,
    // so it hands the position controller no leg to fly before the nose has turned. The altitude is
    // the leg's from the outset - only translation waits.
    const vector3_t craftAtLegAltM = {.v = {
        [ENU_E] = dispatchEst->position.v[ENU_E] * 0.01f,
        [ENU_N] = dispatchEst->position.v[ENU_N] * 0.01f,
        [ENU_U] = targetEnuM.v[ENU_U],
    }};

    if (passGate) {
        // positionNav flies the carrot's velocity, and the carrot is the position the
        // controller holds the craft to: a negative acceptance radius means it never
        // self-completes (the craft sitting on a carrot frozen at its own position
        // would otherwise trip "reached", drop ap.navActive, and kill the yaw rotation
        // on a gate-closed leg start - a deadlock). The executor owns advancement, so
        // there is no callback. A carrot already flying carries on across the corner
        // as it is, where it is; only engage/retry and a point leg start one afresh.
        if (!fp.carrotValid) {
            // Taking over from a leg still flying, it starts where that leg held the craft to and at
            // the velocity it commanded, so neither P nor the commanded velocity steps, and turns
            // onto this leg from there. A turn-in starts on the craft at its own velocity and comes round
            // from there. Otherwise it sets off at the speed the craft is making along the leg, never
            // across or backwards along it, from where the craft comes to rest on it.
            const vector2_t craftM = { .x = craftAtLegAltM.v[ENU_E], .y = craftAtLegAltM.v[ENU_N] };
            if (turnIn) {
                fp.carrotVelMps.x = dispatchEst->velocity.v[ENU_E] * 0.01f;
                fp.carrotVelMps.y = dispatchEst->velocity.v[ENU_N] * 0.01f;
                fp.carrotEnuM = craftM;
            } else if (handingOver) {
                const vector3_t commandedCmS = positionNavGetTargetVelocityCmS();
                fp.carrotVelMps.x = commandedCmS.v[ENU_E] * 0.01f;
                fp.carrotVelMps.y = commandedCmS.v[ENU_N] * 0.01f;
                vector2_t heldM = autopilotGetPositionErrorCm();
                vector2Scale(&heldM, &heldM, 0.01f);
                vector2Add(&heldM, &heldM, &craftM);
                placeCarrot(dispatchEst, &heldM);
            } else {
                const vector2_t legVecM = { .x = targetEnuM.v[ENU_E] - craftM.x, .y = targetEnuM.v[ENU_N] - craftM.y };
                const float legLenM = vector2Norm(&legVecM);
                vector2_t legDir = { .x = 0.0f, .y = 0.0f };
                if (legLenM > 1.0f) {
                    vector2Scale(&legDir, &legVecM, 1.0f / legLenM);
                }
                const float alongMps = (dispatchEst->velocity.v[ENU_E] * legDir.x + dispatchEst->velocity.v[ENU_N] * legDir.y) * 0.01f;
                vector2Scale(&fp.carrotVelMps, &legDir, fp.legYawGated ? 0.0f : constrainf(alongMps, 0.0f, cruiseMps));
                fp.carrotEnuM = restPointM(dispatchEst, &fp.carrotVelMps, autopilotConfig()->maxAngle);
            }
            fp.carrotSpeedMps = vector2Norm(&fp.carrotVelMps);
            fp.carrotValid = true;
            fp.legAnchorValid = false;
        } else {
            readBackCarrot();
        }
        const vector3_t carrotM = {.v = {
            [ENU_E] = fp.carrotEnuM.x,
            [ENU_N] = fp.carrotEnuM.y,
            [ENU_U] = targetEnuM.v[ENU_U],
        }};
        positionNavSetTargetEf(&carrotM, cruiseMps, FP_CARROT_NO_ARRIVAL_M,
                               FP_COMPLETION_ANY_MPS, true, NULL, NULL);
        fp.legTurnCapped = turnIn;
        if (turnIn) {
            positionNavSetMaxAngle(rescueStopMaxAngleDeg());
        }
        positionNavSetAccelLimits(0.0f, 0.0f);
        positionNavSetAltitudeArrivalRequired(false);
        positionNavSetVelocityFeedforward(&fp.carrotVelMps);
    } else if (fp.legYawHolding) {
        // Station-keeping face-the-target leg: hold where we are while the nose comes round. The
        // real target, with its arrival radius and callback, is issued by the update loop the
        // moment the gate opens - issuing it now would translate a misaligned craft, and a frozen
        // target with an arrival radius would trip the arrival test on the spot.
        positionNavSetTargetEf(&craftAtLegAltM, cruiseMps, FP_CARROT_NO_ARRIVAL_M,
                               FP_COMPLETION_ANY_MPS, true, NULL, NULL);
        positionNavSetAccelLimits(0.0f, FP_APPROACH_DECEL_MPS2);
        positionNavSetAltitudeArrivalRequired(false);
    } else {
        positionNavSetTargetEf(&targetEnuM, cruiseMps, arrivalRadiusM,
                               rescueStop ? FP_RESCUE_STOP_STILL_MPS : FP_COMPLETION_ANY_MPS, true,
                               onWaypointReached, NULL);
        // An approach brake replaces the braking curve.
        positionNavSetAccelLimits(pointLegAccelMps2(), (fp.legApproachDecelMps2 > 0.0f) ? 0.0f : FP_APPROACH_DECEL_MPS2);
        positionNavSetApproachBrake(fp.legApproachDecelMps2, FP_RESCUE_LAND_STILL_RADIUS_M);
        // En-route waypoints advance on horizontal arrival; a vehicle that cannot
        // reach the commanded altitude must not orbit forever. HOLD, LAND and
        // TAKEOFF are station-keeping targets and keep the altitude gate, bar the
        // rescue's LAND, which starts down at the descent distance wherever the
        // return left it.
        bool altitudeGated = isStationKeeping;
#if ENABLE_RESCUE_PLAN
        if (fp.isRescuePlan && effective.type == WAYPOINT_TYPE_LAND) {
            altitudeGated = false;
        }
#endif
        positionNavSetAltitudeArrivalRequired(altitudeGated);
        if (rescueStop) {
            const vector2_t stillMps = { .x = 0.0f, .y = 0.0f };
            positionNavSetVelocityFeedforward(&stillMps);
            positionNavSetSettleTimeout(FP_RESCUE_STOP_SETTLE_S);
            positionNavSetMaxAngle(rescueStopMaxAngleDeg());
        }
    }
    if (fp.dispatchAfresh) {
        positionNavStartAfresh();
        fp.dispatchAfresh = false;
    }
#if ENABLE_RESCUE_PLAN
    if (blindClimb) {
        fp.rescueBlindClimb = true;
        fp.rescueClimbed = false;
        autopilotHeadingRecovery(true, 0.0f);
    }
#endif

    // Altitude walks to the waypoint at the leg's rate from the altitude already commanded, so the
    // altitude controller never sees a step.
    positionNavSetVerticalProfile(fp.legVertRateMps, startAltM);

    // Command the nose from the dispatch itself, so a leg that states where to point never spends
    // a cycle with the previous leg's nose command, or none at all.
    updateLegYaw(dispatchEst);

    legDispatched(effective.duration);
    return true;
#endif
}

static void abortMission(flightPlanAbortReason_e reason)
{
#if ENABLE_RESCUE_PLAN
    endHeadingRecovery();
#endif
    fp.state = FP_NAV_ABORTED;
    fp.abortReason = reason;
    fp.patternPending = false;
    fp.patternActive = false;
    fp.inPreTurn = false;
    autopilotSetNavHeadingOverride(false, 0.0f);
#ifdef USE_WING
    resetWingPlan();
#endif
    // Dropping the nav target makes positionControl() fall back to holding
    // position at the current location; the pilot exits via the mode switch.
    positionNavClearTarget();
}

static void navTargetDeltaEnuM(const positionEstimate3d_t *est, vector3_t *deltaM)
{
    const positionNavCommand_t *cmd = positionNavGetActiveCommand();
    deltaM->v[ENU_E] = cmd->targetPosEfM.v[ENU_E] - est->position.v[ENU_E] * 0.01f;
    deltaM->v[ENU_N] = cmd->targetPosEfM.v[ENU_N] - est->position.v[ENU_N] * 0.01f;
    deltaM->v[ENU_U] = cmd->includeAltitude ? cmd->targetPosEfM.v[ENU_U] - est->position.v[ENU_U] * 0.01f : 0.0f;
}

#ifndef USE_WING
static float distanceToNavTargetM(const positionEstimate3d_t *est)
{
    vector3_t deltaM;
    navTargetDeltaEnuM(est, &deltaM);
    return vector3Norm(&deltaM);
}
#endif // !USE_WING

// Delta to the actual waypoint. On a carrot leg the nav target is the carrot,
// which rides with the craft, so use the leg's true waypoint instead.
static void navWaypointDeltaEnuM(const positionEstimate3d_t *est, vector3_t *deltaM)
{
    if (fp.legIsPassGate) {
        deltaM->v[ENU_E] = fp.legTargetEnuM.v[ENU_E] - est->position.v[ENU_E] * 0.01f;
        deltaM->v[ENU_N] = fp.legTargetEnuM.v[ENU_N] - est->position.v[ENU_N] * 0.01f;
        deltaM->v[ENU_U] = fp.legTargetEnuM.v[ENU_U] - est->position.v[ENU_U] * 0.01f;
    } else {
        navTargetDeltaEnuM(est, deltaM);
    }
}

#ifndef USE_WING
// How far the craft travels before the speed it is carrying can be turned around: the braking
// distance at the deceleration ap_max_angle buys, plus the speed carried through the reversal. A
// leg dispatched at speed - a rescue triggered mid-dash above all - spends this going the wrong
// way, and that is physics rather than a flyaway.
static float brakingDistanceM(const positionEstimate3d_t *est)
{
    const float speedMps = sqrtf(sq(est->velocity.v[ENU_E]) + sq(est->velocity.v[ENU_N])) * 0.01f;
    if (fp.legTurnIn) {
        // Braking at the rescue's lean, then a turn's width further out as it comes round.
        const float turnWidthM = 2.0f * FP_TURN_IN_MPS / DEGREES_TO_RADIANS(FP_TURN_IN_RATE_DPS);
        return sq(speedMps) / (2.0f * brakingDecelMps2(rescueStopMaxAngleDeg())) + speedMps * FP_BRAKE_REVERSAL_S + turnWidthM;
    }
    return sq(speedMps) / (2.0f * brakingDecelMps2(autopilotConfig()->maxAngle)) + speedMps * FP_BRAKE_REVERSAL_S;
}
#endif

// Stall/flyaway sanity against a distance-to-goal. The carrot path passes the
// distance to the waypoint (not the carrot, which rides with the craft);
// the point path passes the distance to the nav target.
static void updateProgressTracking(float distanceM, timeUs_t currentTimeUs)
{
    if (fp.bestDistanceToTargetM == FLT_MAX) {
#ifdef USE_WING
        const float overshootM = fmaxf(distanceM * FP_FLYAWAY_LEG_FRACTION,
                                       flightPlanWingOvershootM(positionEstimatorGetEstimate()));
#else
        const float overshootM = fmaxf(distanceM * FP_FLYAWAY_LEG_FRACTION,
                                       brakingDistanceM(positionEstimatorGetEstimate()));
#endif
        fp.flyawayMarginM = constrainf(overshootM, FP_FLYAWAY_MARGIN_MIN_M, FP_FLYAWAY_MARGIN_MAX_M);
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

#ifndef USE_WING
static void checkLegProgress(timeUs_t currentTimeUs, const positionEstimate3d_t *est)
{
    if (!positionNavHasActiveTarget()) {
        return;
    }
    updateProgressTracking(distanceToNavTargetM(est), currentTimeUs);
}

// Integrate the heading-fault detector for one cycle. Returns true when a
// course-over-ground vs fused-heading disagreement has been sustained long
// enough to declare the heading estimate faulty. Only judged while the nose is
// on command and the craft is moving, so wind and manoeuvres cannot trip it.
static bool checkHeadingFault(float dtS, const positionEstimate3d_t *est)
{
    // Only meaningful while the autopilot is actively steering the nose toward a
    // heading; in FIXED/DAMPENER the nose is uncommanded, so a course-vs-heading
    // gap is expected and must not be read as a fault.
    const uint8_t yawMode = autopilotConfig()->yawMode;
    if (yawMode == YAW_MODE_FIXED || yawMode == YAW_MODE_DAMPENER) {
        fp.hdgFaultTimeS = 0.0f;
        return false;
    }

    // A carrot rides with the craft, so the bearing to it means nothing: judge against the waypoint.
    vector3_t deltaM;
    navWaypointDeltaEnuM(est, &deltaM);
    const float bearingDeg = RADIANS_TO_DEGREES(atan2_approx(deltaM.v[ENU_E], deltaM.v[ENU_N]));
    const float headingDeg = attitude.values.yaw * 0.1f;
    const float yawErrDeg = wrapDeg180f(headingDeg - bearingDeg);
    const float cogVsHeadingDeg = fabsf(wrapDeg180f(headingDeg - gpsSol.groundCourse * 0.1f));

    const bool eligible = gpsSol.groundSpeed > FP_HDG_FAULT_MIN_SPEED_CMS
                       && fabsf(yawErrDeg) < FP_HDG_FAULT_ALIGNED_DEG
                       && !fp.inPreTurn;   // the pre-turn deliberately swings the nose off-course
    fp.hdgFaultTimeS = (eligible && cogVsHeadingDeg > FP_HDG_FAULT_ERR_DEG)
                     ? fp.hdgFaultTimeS + dtS
                     : fmaxf(fp.hdgFaultTimeS - 2.0f * dtS, 0.0f);
    return fp.hdgFaultTimeS > FP_HDG_FAULT_TIME_S;
}

// Bearing from the craft to a point, in the same +-180 frame the carrot's nose command uses.
// False when the point is too close to mean anything: chasing a bearing around a point the craft is
// sitting on would spin it.
static bool bearingToPointDeg(const positionEstimate3d_t *est, const vector3_t *pointEnuM, float *bearingDeg)
{
    const float deltaEM = pointEnuM->v[ENU_E] - est->position.v[ENU_E] * 0.01f;
    const float deltaNM = pointEnuM->v[ENU_N] - est->position.v[ENU_N] * 0.01f;
    if (sq(deltaEM) + sq(deltaNM) < sq(FP_YAW_BEARING_MIN_M)) {
        return false;
    }
    *bearingDeg = RADIANS_TO_DEGREES(atan2_approx(deltaEM, deltaNM));
    return true;
}

// Is the nose on the leg, within the angle a face-the-target leg is willing to set off at? True
// when there is no meaningful bearing to be on, so the gate cannot latch on a target underfoot.
static bool legNoseOnTarget(const positionEstimate3d_t *est, const vector3_t *targetEnuM)
{
    float bearingDeg;
    if (!bearingToPointDeg(est, targetEnuM, &bearingDeg)) {
        return true;
    }
    return fabsf(wrapDeg180f(attitude.values.yaw * 0.1f - bearingDeg)) < FP_YAW_ALIGN_DEG;
}

// Nose command for a leg that states its own yaw behaviour, refreshed every cycle because the
// bearing moves as the craft does. WAYPOINT_YAW_DEFAULT legs are left to the carrot's pre-turn
// logic and the configured yaw mode. Also opens the translate gate: a FACE_TARGET leg holds station
// until the nose has come round onto it, and once open the gate stays open for the leg, so a gust
// swinging the nose mid-leg cannot park the craft in mid-air. A leg turning in from the craft's own
// motion is not held: the nose follows its carrot round.
static void updateLegYaw(const positionEstimate3d_t *est)
{
    if (fp.legTurnIn && vector2Norm(&fp.carrotVelMps) > 1.0f) {
        autopilotSetNavHeadingOverride(true, RADIANS_TO_DEGREES(atan2_approx(fp.carrotVelMps.x, fp.carrotVelMps.y)));
        return;
    }
    switch (fp.legYawBehaviour) {
    case WAYPOINT_YAW_HOLD:
        autopilotSetNavHeadingOverride(true, fp.legYawHoldDeg);
        return;
    case WAYPOINT_YAW_FACE_TARGET:
    case WAYPOINT_YAW_FACE_NEXT:
        break;
    default:
        return;
    }

    // Every path from here either commands a heading or hands the nose back: leaving the previous
    // leg's override in force would steer to a bearing this leg never asked for.
    vector3_t faceEnuM = fp.legTargetEnuM;
    if (fp.legYawBehaviour == WAYPOINT_YAW_FACE_NEXT) {
        const waypoint_t *nextWp = nextPositionalWaypoint(fp.currentIndex + 1);
        if (nextWp == NULL || !computeTargetEnuM(nextWp, &faceEnuM)) {
            autopilotSetNavHeadingOverride(false, 0.0f);
            return;
        }
    }

    float bearingDeg;
    if (!bearingToPointDeg(est, &faceEnuM, &bearingDeg)) {
        autopilotSetNavHeadingOverride(false, 0.0f);
        fp.legYawGated = false;
        return;
    }

    const bool waitedOut = cmpTimeUs(micros(), fp.legYawGateStartUs) >= (timeDelta_t)FP_YAW_ALIGN_TIMEOUT_US;
    const float speedMps = sqrtf(sq(est->velocity.v[ENU_E]) + sq(est->velocity.v[ENU_N])) * 0.01f;
    fp.legYawBraked = fp.legYawBraked || waitedOut || speedMps <= FP_YAW_SWING_MAX_SPEED_MPS;
    if ((fp.legYawGated || isRescueStop()) && !fp.legYawBraked) {
        autopilotSetNavHeadingOverride(true, fp.legYawHoldDeg);
        return;
    }
    autopilotSetNavHeadingOverride(true, bearingDeg);

    if (fp.legYawGated) {
        if (fabsf(wrapDeg180f(attitude.values.yaw * 0.1f - bearingDeg)) < FP_YAW_ALIGN_DEG) {
            fp.legYawGated = false;
        } else if (waitedOut) {
            fp.legYawGated = false;
            fp.legYawTimedOut = true;
        }
    }
}

// One cycle of leg-line carrot tracking for an en-route pass-through leg: march a
// carrot along the leg line toward the waypoint and hand it to positionNav, carve
// the corner into the next leg at a turn-angle-scaled speed, and own gate
// detection/advancement. Ported from the tracker in PR #15442.
static void updateLegCarrot(float dtS, timeUs_t currentTimeUs, const positionEstimate3d_t *est)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    const float cornerSpeedFloorMps = cfg->navCornerSpeed * 0.01f;
    const float cornerDeltaVMps     = cfg->navCornerDeltaV * 0.01f;
    const float decelMps2           = cfg->navDecel * 0.01f;
    const float accelMps2           = cfg->navAccel * 0.01f;
    const float leadTimeS           = cfg->navCarrotLeadTime * 0.1f;
    const float leadMaxM            = cfg->navCarrotLeadMax * 0.01f;

    const vector2_t craft = { .x = est->position.v[ENU_E] * 0.01f, .y = est->position.v[ENU_N] * 0.01f };
    const vector2_t wp    = { .x = fp.legTargetEnuM.v[ENU_E], .y = fp.legTargetEnuM.v[ENU_N] };
    const vector2_t toWp  = { .x = wp.x - craft.x, .y = wp.y - craft.y };
    const float distM = vector2Norm(&toWp);

    readBackCarrot();

    // Corner geometry uses the next positional waypoint, skipping any modifier
    // records between legs (their coordinates would otherwise steer the corner).
    vector2_t next = wp;
    bool haveNext = false;
    const uint8_t nextIndex = nextPositionalIndex(fp.currentIndex + 1);
    const waypoint_t *nextWp = activePlanWaypoint(nextIndex);
    if (nextWp != NULL) {
        vector3_t nextEnuM;
        waypoint_t effNext = *nextWp;
        if (computeTargetEnuM(&effNext, &nextEnuM)) {
            next.x = nextEnuM.v[ENU_E];
            next.y = nextEnuM.v[ENU_N];
            haveNext = true;
        }
    }

    // Corner speed on a constant delta-v budget: the velocity change a corner
    // demands is 2*v*sin(turn/2), so v = deltaV / (2 sin(turn/2)) sheds the same
    // delta-v at every gate — a shallow bend at cruise runs no wider than a
    // hairpin at the floor. The gate radius scales with it for the same reason.
    float cornerSpeedMps = fp.legCruiseMps;
    bool turnsAtGate = false;
    const vector2_t inFrom = fp.legValid ? fp.legStartEnuM : craft;
    if (haveNext) {
        const vector2_t inVec  = { .x = wp.x - inFrom.x, .y = wp.y - inFrom.y };
        const vector2_t outVec = { .x = next.x - wp.x,   .y = next.y - wp.y };
        const float inLen  = vector2Norm(&inVec);
        const float outLen = vector2Norm(&outVec);
        if (inLen > 1.0f && outLen > 1.0f) {
            const float cosTurn = (inVec.x * outVec.x + inVec.y * outVec.y) / (inLen * outLen);
            const float shed = sqrtf(fmaxf(2.0f - 2.0f * cosTurn, 0.0f)); // = 2 sin(turn/2)
            turnsAtGate = shed > 0.05f;
            cornerSpeedMps = turnsAtGate
                ? constrainf(cornerDeltaVMps / shed, cornerSpeedFloorMps, fp.legCruiseMps)
                : fp.legCruiseMps;   // straight through
        }
    }
    const float arriveRadiusM = fp.legArriveRadiusM;
    // FLYBY cuts the corner: the gate scales up with the corner speed. FLYOVER
    // keeps its fly-over-the-point meaning - the carrot tracking and corner-speed
    // profile still apply, but the gate stays at the arrival radius so the craft
    // passes tight over the point instead of carving the corner wide.
    // The resolved radius can exceed FP_PASS_MAX_M (a rescue hands over at its
    // descent distance), which would invert the corner-gate bounds; the corner
    // gate keeps its own ceiling and the fly-over gate honours the radius.
    const waypoint_t *thisWp = currentWaypoint();
    const bool flyby = (thisWp == NULL) || (thisWp->type == WAYPOINT_TYPE_FLYBY);
    const float arriveM = flyby
        ? constrainf(cornerSpeedMps * FP_GATE_RADIUS_SCALE,
                     fminf(arriveRadiusM, FP_PASS_MAX_M), FP_PASS_MAX_M)
        : arriveRadiusM;

    // A point leg next picks up at no more than its braking speed from the gate: brake into that, or
    // it is left to shed the rest at the carrot acceleration, well past the point. The rescue's
    // carries this leg's approach on instead.
    if (haveNext && fp.legApproachDecelMps2 <= 0.0f && !flownAsCarrotLeg(nextWp, nextIndex)) {
        const vector2_t outVec = { .x = next.x - wp.x, .y = next.y - wp.y };
        const float handOverM = sqrtf(sq(vector2Norm(&outVec)) + sq(arriveM));
        cornerSpeedMps = fminf(cornerSpeedMps, sqrtf(2.0f * FP_APPROACH_DECEL_MPS2 * handOverM));
    }

    // Overrun fallback: a fast crossing that misses the arrive bubble by a hair
    // still counts once the craft is past the waypoint along-track inside a sane
    // lateral corridor — without it the craft overruns the carrot and yaws back.
    bool overran = false;
    if (fp.legValid) {
        const vector2_t inVec = { .x = wp.x - fp.legStartEnuM.x, .y = wp.y - fp.legStartEnuM.y };
        const float inLen = vector2Norm(&inVec);
        if (inLen > 1.0f) {
            const float ux = inVec.x / inLen, uy = inVec.y / inLen;
            const float along   = (craft.x - fp.legStartEnuM.x) * ux + (craft.y - fp.legStartEnuM.y) * uy;
            const float lateral = fabsf((craft.y - fp.legStartEnuM.y) * ux - (craft.x - fp.legStartEnuM.x) * uy);
            overran = along > inLen && lateral < FP_OVERRUN_LAT_M;
        }
    }

    // Stall/flyaway sanity is measured against the waypoint, not the carrot, which rides with the
    // craft.
    updateProgressTracking(distM, currentTimeUs);
    if (fp.state != FP_NAV_TARGETING) {
        return;   // the sanity check aborted the mission
    }

    // A face-the-target leg dispatched already inside the gate must still turn before it counts as
    // arrived, or the nose gate is skipped entirely on a waypoint that happens to be close. Bounded
    // by the same alignment timeout that releases the march gate below.
    // Where the leg turns, the gate is also crossed when the carrot reaches it: the carrot is the path
    // the craft is held to, and has to turn at the gate, not wherever a craft pushed off the line
    // happens to meet it.
    vector2_t carrotToWp;
    vector2Sub(&carrotToWp, &wp, &fp.carrotEnuM);
    const float gateDistM = turnsAtGate ? fminf(distM, vector2Norm(&carrotToWp)) : distM;
    if (!fp.legYawGated && !fp.legTurnIn && !fp.legTurnCapped && (gateDistM < arriveM || overran)) {
        // Gate crossed: the next leg's line starts on this waypoint, so the flown line is the drawn
        // wp->wp line; the carrot carries on from where it is and turns onto it.
        fp.legAnchorEnuM = wp;
        fp.legAnchorValid = true;
        onWaypointReached(NULL);   // fires the reached listener and advances/dispatches
        return;
    }

    if (!fp.legValid) {
        fp.legStartEnuM = fp.legAnchorValid ? fp.legAnchorEnuM : fp.carrotEnuM;
        fp.legAnchorValid = false;
        fp.legValid = true;
        fp.measFiltValid = false;   // new leg, new along-track frame: re-seed the filters
    }

    const vector2_t legVec = { .x = wp.x - fp.legStartEnuM.x, .y = wp.y - fp.legStartEnuM.y };
    const float legLenM = vector2Norm(&legVec);
    vector2_t legDir = { .x = 1.0f, .y = 0.0f };
    if (legLenM > 1.0f) {
        legDir.x = legVec.x / legLenM;
        legDir.y = legVec.y / legLenM;
    }
    const float craftAlongM = (craft.x - fp.legStartEnuM.x) * legDir.x + (craft.y - fp.legStartEnuM.y) * legDir.y;

    if (!fp.measFiltValid || dtS <= 0.0f) {
        fp.alongFiltM = craftAlongM;
        fp.measFiltValid = true;
    } else {
        const float alpha = dtS / (FP_MEAS_FILTER_S + dtS);
        fp.alongFiltM += alpha * (craftAlongM - fp.alongFiltM);
    }

    // Trapezoidal profile keyed on the craft's distance to the gate: cruise, then
    // decelerate to cross at the corner speed. The carrot speed slews (no
    // full-cruise leap at leg starts) and is deliberately not reset at leg
    // switches, so speed carries smoothly through corners.
    // Pre-turn timing keeps the raw along-track (the nose must start swinging on
    // time); the trapezoid uses the filtered copy.
    const float remainingRawM  = fmaxf(legLenM - craftAlongM - arriveM, 0.0f);
    const float remainingFiltM = fmaxf(legLenM - fp.alongFiltM - arriveM, 0.0f);

    // Lag compensation: the craft answers the carrot's braking FP_CARROT_BRAKE_LAG_S
    // later, so a profile keyed on the filtered remaining distance is crossed hot
    // by about lag * decel. Start the brake one lag-distance early and the craft —
    // not just the carrot — arrives at corner speed, riding the last stretch AT
    // corner speed instead of still braking through the gate.
    const float brakeLagM = fp.carrotSpeedMps * FP_CARROT_BRAKE_LAG_S;
    const float remainingBrakeM = fmaxf(remainingFiltM - brakeLagM, 0.0f);
    float desiredMps = fminf(fp.legCruiseMps, sqrtf(sq(cornerSpeedMps) + 2.0f * decelMps2 * remainingBrakeM));

    const float headingDeg = attitude.values.yaw * 0.1f;
    const float legBearingDeg = RADIANS_TO_DEGREES(atan2_approx(legDir.x, legDir.y));
    const float legErrDeg = wrapDeg180f(headingDeg - legBearingDeg);

    // Pre-turn: through the approach zone (ending a few metres before the gate, so
    // the yaw law has closed the lag by the crossing) blend the commanded nose
    // heading from this leg onto the next. Computed BEFORE the march gate so the
    // gate can exempt it: the pre-turn deliberately swings the nose past the leg
    // on sharp corners, and must not stall forward progress while doing so.
    fp.inPreTurn = false;
    float noseBearingDeg = legBearingDeg;
    if (haveNext) {
        const float preturnM = cfg->navPreturnDist * 0.01f;
        const float t = 1.0f - constrainf((remainingRawM - 5.0f) / fmaxf(preturnM - 5.0f, 1.0f), 0.0f, 1.0f);
        const vector2_t nextLeg = { .x = next.x - wp.x, .y = next.y - wp.y };
        if (t > 0.0f && vector2Norm(&nextLeg) > 1.0f) {
            const float nextBearingDeg = RADIANS_TO_DEGREES(atan2_approx(nextLeg.x, nextLeg.y));
            noseBearingDeg = legBearingDeg + t * wrapDeg180f(nextBearingDeg - legBearingDeg);
            fp.inPreTurn = true;
        }
    }

    // March gating keys on alignment with the leg direction so forward progress
    // never accumulates while rotating in place at a leg start or a reversal; the
    // pre-turn swing is exempt so corner speed carries through the gate.
    // A leg whose nose never came round is flown anyway (see FP_YAW_ALIGN_TIMEOUT_US): the march
    // gate must not hold it back a second time. A heading that is actually wrong, rather than
    // merely stuck, is what checkHeadingFault() is for.
    const bool aligned = fabsf(legErrDeg) < 90.0f || fp.inPreTurn || fp.legYawTimedOut;
    if (!aligned || fp.legYawGated || legLenM <= 1.0f) {
        desiredMps = 0.0f;
    }

    // The carrot steers onto the leg line toward a point one lead ahead of it (closing at no more
    // than 45 degrees), at the profile speed, its velocity changing at no more than the carrot
    // acceleration. The acceleration goes to turning first: speeding up out of a corner before it
    // has come round only carries it wide.
    const vector2_t fromStart = { .x = fp.carrotEnuM.x - fp.legStartEnuM.x, .y = fp.carrotEnuM.y - fp.legStartEnuM.y };
    const float carrotAlongM = fromStart.x * legDir.x + fromStart.y * legDir.y;
    const float carrotCrossM = fromStart.x * legDir.y - fromStart.y * legDir.x;
    const float leadM = constrainf(fp.carrotSpeedMps * leadTimeS, FP_CARROT_LEAD_MIN_M, leadMaxM);
    const float aheadM = fmaxf(leadM, fabsf(carrotCrossM));
    vector2_t aimDir = {
        .x = fp.legStartEnuM.x + legDir.x * (carrotAlongM + aheadM) - fp.carrotEnuM.x,
        .y = fp.legStartEnuM.y + legDir.y * (carrotAlongM + aheadM) - fp.carrotEnuM.y,
    };
    const float budgetMps = accelMps2 * dtS;
    const float flyingMps = vector2Norm(&fp.carrotVelMps);
    float turnBudgetMps = budgetMps;
    float slowBudgetMps = budgetMps;
    bool turnInBraked = true;
    if (fp.legTurnIn) {
        // Straight at the waypoint (the line is drawn from wherever the turn ends), at the turn-in
        // speed or the approach brake where that is slower. The speed is shed at the rescue's lean
        // before the turn starts, and the turn comes round at the turn-in rate within that lean.
        vector2Sub(&aimDir, &wp, &fp.carrotEnuM);
        desiredMps = fminf(fp.legCruiseMps, FP_TURN_IN_MPS);
        if (fp.legApproachDecelMps2 > 0.0f) {
            desiredMps = fminf(desiredMps, positionNavApproachSpeedMps(fp.legCruiseMps, fp.legApproachDecelMps2, FP_RESCUE_LAND_STILL_RADIUS_M, distM));
        }
        // Slow enough to turn inside the distance left, or it circles the waypoint.
        desiredMps = fminf(desiredMps, 0.5f * DEGREES_TO_RADIANS(FP_TURN_IN_RATE_DPS) * vector2Norm(&carrotToWp));
        const float leanMps2 = brakingDecelMps2(rescueStopMaxAngleDeg());
        slowBudgetMps = leanMps2 * dtS;
        turnInBraked = fp.carrotSpeedMps <= desiredMps + slowBudgetMps;
        turnBudgetMps = turnInBraked ? fminf(DEGREES_TO_RADIANS(FP_TURN_IN_RATE_DPS) * flyingMps, leanMps2) * dtS : 0.0f;
    }
    vector2Normalize(&aimDir, &aimDir);
    vector2_t carrotDir = aimDir;
    float turningMps = 0.0f;
    float turnRad = 0.0f;
    if (flyingMps > 0.01f) {
        vector2Scale(&carrotDir, &fp.carrotVelMps, 1.0f / flyingMps);
        turnRad = atan2_approx(vector2Cross(&carrotDir, &aimDir), vector2Dot(&carrotDir, &aimDir));
        turningMps = fabsf(turnRad) * flyingMps;
        if (turningMps <= turnBudgetMps) {
            carrotDir = aimDir;
        } else {
            const float stepRad = ((turnRad > 0.0f) ? turnBudgetMps : -turnBudgetMps) / flyingMps;
            vector2Rotate(&carrotDir, &carrotDir, stepRad);
            turningMps = turnBudgetMps;
        }
    }
    const float speedUpMps = sqrtf(fmaxf(sq(budgetMps) - sq(turningMps), 0.0f));
    fp.carrotSpeedMps = constrainf(desiredMps, fp.carrotSpeedMps - slowBudgetMps, fp.carrotSpeedMps + speedUpMps);
    if (fp.legTurnIn && turnInBraked && fabsf(turnRad) < DEGREES_TO_RADIANS(FP_TURN_IN_DONE_DEG)) {
        fp.legTurnIn = false;
        fp.legValid = false;
    }
    if (fp.legTurnCapped && !fp.legTurnIn) {
        const vector2_t craftVelMps = { .x = est->velocity.v[ENU_E] * 0.01f, .y = est->velocity.v[ENU_N] * 0.01f };
        const float courseOffRad = atan2_approx(vector2Cross(&craftVelMps, &toWp), vector2Dot(&craftVelMps, &toWp));
        if (vector2Norm(&craftVelMps) < 1.0f || fabsf(courseOffRad) < DEGREES_TO_RADIANS(FP_TURN_IN_DONE_DEG)) {
            fp.legTurnCapped = false;
            positionNavSetMaxAngle(0.0f);
        }
    }
    if (fp.legApproachDecelMps2 > 0.0f && !fp.legTurnIn) {
        fp.carrotSpeedMps = fminf(fp.carrotSpeedMps,
                                  positionNavApproachSpeedMps(fp.legCruiseMps, fp.legApproachDecelMps2, FP_RESCUE_LAND_STILL_RADIUS_M, distM));
    }
    vector2Scale(&fp.carrotVelMps, &carrotDir, fp.carrotSpeedMps);
    placeCarrot(est, &fp.carrotEnuM);

    // Nose command. On a pass-through leg the executor owns yaw, which is what
    // makes the march gate safe: point the nose along the leg (blending onto the
    // next leg through the pre-turn) whenever it is not already aligned. This
    // rotates the nose onto the leg at an engage or reversal, so a nose-backwards
    // engage cannot deadlock the frozen carrot. It is also held on the leg while
    // the carrot is still coming round onto it, or a course-steered nose swings
    // back after the turning course and out again. Once both are aligned, the
    // configured yaw mode takes over.
    // A leg that states its own yaw behaviour has already had its nose commanded by updateLegYaw().
    const float carrotBearingDeg = RADIANS_TO_DEGREES(atan2_approx(fp.carrotVelMps.x, fp.carrotVelMps.y));
    const bool carrotComingRound = fp.carrotSpeedMps > FP_MIN_CRUISE_MPS
                                && fabsf(wrapDeg180f(carrotBearingDeg - legBearingDeg)) > FP_YAW_ALIGN_DEG;
    if (fp.legYawBehaviour == WAYPOINT_YAW_DEFAULT) {
        if (fp.inPreTurn || !aligned || carrotComingRound) {
            autopilotSetNavHeadingOverride(true, noseBearingDeg);
        } else {
            autopilotSetNavHeadingOverride(false, 0.0f);
        }
    }

    const vector3_t carrot = {.v = {
        [ENU_E] = fp.carrotEnuM.x,
        [ENU_N] = fp.carrotEnuM.y,
        [ENU_U] = fp.legTargetEnuM.v[ENU_U],   // altitude tracks the waypoint, as the point path does
    }};
    positionNavMoveTargetEf(&carrot);
    positionNavSetVelocityFeedforward(&fp.carrotVelMps);
}

// The descent is flown at the rate the caller states — the LAND leg's own rate, resolved at
// dispatch. The below-ground target only keeps vertical arrival from ever triggering; it is the
// rate, not the depth, that decides how fast the craft comes down. A landing at the end of an
// approach brake closes the rest of it on the way down; any other creeps to its point at the
// descent rate.
static void startLanding(timeUs_t currentTimeUs, float targetEastM, float targetNorthM, float descentRateMps)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const vector3_t targetM = {.v = {
        [ENU_E] = targetEastM,
        [ENU_N] = targetNorthM,
        [ENU_U] = est->position.v[ENU_U] * 0.01f - FP_LANDING_TARGET_DEPTH_M,
    }};

    const float startAltM = commandedAltitudeM(est);
    const float descentMps = MAX(FP_LANDING_MIN_RATE_MPS, descentRateMps);
    const bool continueApproach = fp.legApproachDecelMps2 > 0.0f;
    const float cruiseMps = continueApproach ? fp.legCruiseMps : descentMps;
    positionNavSetTargetEf(&targetM, cruiseMps, 1.0f, 0.1f, true, NULL, NULL);
    positionNavSetAccelLimits(pointLegAccelMps2(), continueApproach ? 0.0f : FP_APPROACH_DECEL_MPS2);
    positionNavSetApproachBrake(fp.legApproachDecelMps2, FP_RESCUE_LAND_STILL_RADIUS_M);
    positionNavSetVerticalProfile(descentMps, startAltM);
    fp.landingRateMps = descentMps;

    fp.state = FP_NAV_LANDING;
    fp.landingStartUs = currentTimeUs;
    fp.touchdownQuietStartUs = 0;
    fp.landingDescentEstablished = false;
}

static void updateLanding(timeUs_t currentTimeUs)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float verticalVelocityCmS = est->velocity.v[ENU_U];
    const float commandedDescentCmS = fp.landingRateMps * 100.0f;

    if (fp.state == FP_NAV_LANDING) {
        positionNavLowerTargetAltitude(est->position.v[ENU_U] * 0.01f - FP_LANDING_TARGET_DEPTH_M);
    }

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
            completePlan();
            disarm(DISARM_REASON_LANDING);
        }
    } else {
        fp.touchdownQuietStartUs = 0;
    }
}
#endif // !USE_WING

// A re-target rather than a continuation of the leg being flown: the craft brakes onto where it is.
static void startLandingInPlace(timeUs_t currentTimeUs)
{
#ifdef USE_WING
    UNUSED(currentTimeUs);
    const waypoint_t here = {
        .latitude = gpsSol.llh.lat,
        .longitude = gpsSol.llh.lon,
        .altitude = gpsSol.llh.altCm,
        .type = WAYPOINT_TYPE_LAND,
    };
    flightPlanNavInjectPlan(&here, 1);
#else
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    startLanding(currentTimeUs, est->position.v[ENU_E] * 0.01f, est->position.v[ENU_N] * 0.01f,
                 autopilotConfig()->landingDescentRate * 0.01f);
    positionNavStartAfresh();
#endif
}

#ifndef USE_WING
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
    startLanding(currentTimeUs, cmd->targetPosEfM.v[ENU_E], cmd->targetPosEfM.v[ENU_N], fp.legVertRateMps);
}
#endif // !USE_WING

// Geofence RTH response: [fly home at return altitude, land at home]. Return
// altitude never commands an en-route descent (rescue's ALT_MODE_MAX
// behaviour). Falls back to landing in place if the plan can't be injected.
static void injectReturnHomePlan(timeUs_t currentTimeUs)
{
#if defined(USE_GPS_RESCUE)
    const int32_t returnAltCm = MAX(GPS_home_llh.altCm + (int32_t)gpsRescueConfig()->returnAltitudeM * 100, gpsSol.llh.altCm);
#else
    const int32_t returnAltCm = gpsSol.llh.altCm;
#endif
#if defined(USE_GPS_RESCUE) && !defined(USE_WING)
    const uint16_t speedCmS = gpsRescueConfig()->groundSpeedCmS;
#else
    const uint16_t speedCmS = 0;    // waypoint default: autopilot maxVelocity
#endif
    const waypoint_t plan[] = {
        {
            .latitude = GPS_home_llh.lat,
            .longitude = GPS_home_llh.lon,
            .altitude = returnAltCm,
            .speed = speedCmS,
            .type = WAYPOINT_TYPE_FLYOVER,
            .yawBehaviour = WAYPOINT_YAW_FACE_TARGET,
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
        startLandingInPlace(currentTimeUs);
    }
}

#if ENABLE_RESCUE_PLAN
#ifdef USE_WING
// A wing can stop neither to climb nor to land. Its rescue climbs in a loiter where it is, unless
// the climb is too small to bother with, and sets off once at height and heading for home, where it
// loiters down and lands: [climbing HOLD -> FLYOVER home -> LAND home]. Near home there is no return
// worth flying: [LAND home]. It needs no heading. It returns no lower than the landing's approach is
// flown, and lands on the arming point's ground.
static uint8_t buildRescuePlan(waypoint_t out[FP_INJECTED_PLAN_MAX])
{
    if (!STATE(GPS_FIX_HOME) || !STATE(GPS_FIX)) {
        return 0;
    }

    const gpsRescueConfig_t *cfg = gpsRescueConfig();
    const waypoint_t landHome = {
        .latitude = GPS_home_llh.lat,
        .longitude = GPS_home_llh.lon,
        .altitude = GPS_home_llh.altCm,
        .vertRate = cfg->descendRate,
        .type = WAYPOINT_TYPE_LAND,
    };
    if (GPS_distanceToHome < cfg->minStartDistM) {
        out[0] = landHome;
        return 1;
    }

    const int32_t currentAltCm = gpsSol.llh.altCm;
    int32_t returnAltCm;
    switch (cfg->altitudeMode) {
    case GPS_RESCUE_ALT_MODE_FIXED:
        returnAltCm = GPS_home_llh.altCm + cfg->returnAltitudeM * 100;
        break;
    case GPS_RESCUE_ALT_MODE_CURRENT:
        returnAltCm = currentAltCm + cfg->initialClimbM * 100;
        break;
    case GPS_RESCUE_ALT_MODE_MAX:
    default:
        // the maximum altitude is kept above the arming point
        returnAltCm = GPS_home_llh.altCm + (int32_t)gpsRescueGetMaxAltitudeCm() + cfg->initialClimbM * 100;
        break;
    }
    returnAltCm = MAX(returnAltCm, currentAltCm); // never command an en-route descent
    returnAltCm = MAX(returnAltCm, GPS_home_llh.altCm + autopilotWingConfig()->landApproachAlt * 100);

    const uint16_t climbRateCmS = MIN(cfg->ascendRate, autopilotWingConfig()->maxClimbRate * 10);
    uint8_t count = 0;
    if (returnAltCm - currentAltCm >= FP_WING_RESCUE_MIN_CLIMB_CM) {
        out[count++] = (waypoint_t){
            .latitude = gpsSol.llh.lat,
            .longitude = gpsSol.llh.lon,
            .altitude = returnAltCm,
            .vertRate = climbRateCmS,
            .type = WAYPOINT_TYPE_HOLD,
        };
    }
    out[count++] = (waypoint_t){
        .latitude = GPS_home_llh.lat,
        .longitude = GPS_home_llh.lon,
        .altitude = returnAltCm,
        .vertRate = climbRateCmS,
        .type = WAYPOINT_TYPE_FLYOVER,
    };
    out[count++] = landHome;
    return count;
}
#else
// Whether the craft comes to a stop inside gps_rescue_min_start_dist, braking at the rescue's own
// lean: one passing close to home at speed stops well clear of it, and one closing fast from
// further out can stop beside it.
static bool rescueStopsNearHome(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const vector2_t stillMps = { .x = 0.0f, .y = 0.0f };
    const vector2_t restM = restPointM(est, &stillMps, rescueStopMaxAngleDeg());
    vector2_t fromHomeCm;
    GPS_distance2d(&GPS_home_llh, &gpsSol.llh, &fromHomeCm);
    const float eastM = fromHomeCm.v[EF_EAST] * 0.01f + restM.x - est->position.v[ENU_E] * 0.01f;
    const float northM = fromHomeCm.v[EF_NORTH] * 0.01f + restM.y - est->position.v[ENU_N] * 0.01f;
    return sq(eastM) + sq(northM) < sq((float)gpsRescueConfig()->minStartDistM);
}

// Failsafe rescue mission: [climb HOLD where the craft comes to a stop ->
// FLYOVER home -> LAND home]. The climb completes at altitude with the craft
// still, so the return leg sets off from rest - legacy ATTAIN_ALT semantics -
// bar the one after a heading recovery, which turns in from the pitch-forward.
// A craft that stops near home has no return worth flying, and a climb only
// lifts it over whoever is standing there: [LAND where the craft comes to a
// stop]. Without a heading it cannot hold that position, so it gets no plan and
// the caller's altitude-only descent lands it where it is.
static uint8_t buildRescuePlan(waypoint_t out[FP_INJECTED_PLAN_MAX])
{
    if (!STATE(GPS_FIX_HOME) || !STATE(GPS_FIX)) {
        return 0;
    }

    const int32_t homeAltCm = GPS_home_llh.altCm;
    const int32_t currentAltCm = gpsSol.llh.altCm;
    const int32_t climbCm = (int32_t)gpsRescueConfig()->initialClimbM * 100;
    const int32_t fixedCm = (int32_t)gpsRescueConfig()->returnAltitudeM * 100;
    const uint16_t speedCmS = gpsRescueConfig()->groundSpeedCmS;

    if (rescueStopsNearHome()) {
        if (headingUnknown()) {
            return 0;
        }
        out[0] = (waypoint_t){
            .latitude = gpsSol.llh.lat,
            .longitude = gpsSol.llh.lon,
            .altitude = currentAltCm,
            .speed = speedCmS,
            .vertRate = gpsRescueConfig()->descendRate,
            .type = WAYPOINT_TYPE_LAND,
            .yawBehaviour = WAYPOINT_YAW_HOLD,
        };
        return 1;
    }

    int32_t returnAltCm;
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
    returnAltCm = MAX(returnAltCm, currentAltCm); // never command an en-route descent

    out[0] = (waypoint_t){
        .latitude = gpsSol.llh.lat,
        .longitude = gpsSol.llh.lon,
        .altitude = returnAltCm,
        .vertRate = gpsRescueConfig()->ascendRate,
        .type = WAYPOINT_TYPE_HOLD,
        .yawBehaviour = WAYPOINT_YAW_FACE_NEXT,     // turn toward home while climbing
    };
    out[1] = (waypoint_t){
        .latitude = GPS_home_llh.lat,
        .longitude = GPS_home_llh.lon,
        .altitude = returnAltCm,
        .speed = speedCmS,
        .vertRate = gpsRescueConfig()->ascendRate,
        .type = WAYPOINT_TYPE_FLYOVER,
        .yawBehaviour = WAYPOINT_YAW_FACE_TARGET,   // and do not set off until it is pointing there
    };
    out[2] = (waypoint_t){
        .latitude = GPS_home_llh.lat,
        .longitude = GPS_home_llh.lon,
        .altitude = returnAltCm,
        .speed = speedCmS,
        .vertRate = gpsRescueConfig()->descendRate,
        .type = WAYPOINT_TYPE_LAND,
    };
    return 3;
}
#endif // USE_WING

bool flightPlanNavStageRescuePlan(void)
{
    waypoint_t plan[FP_INJECTED_PLAN_MAX];
    const uint8_t count = buildRescuePlan(plan);
    if (count == 0) {
        return false;
    }

    if (fp.active) {
        // Already flying (rx-loss during a mission): replace it immediately.
        captureAltitudeFrame();
        memcpy(fp.injected, plan, count * sizeof(plan[0]));
        fp.injectedCount = count;
        fp.currentIndex = 0;
        fp.abortReason = FP_ABORT_NONE;
        fp.isRescuePlan = true;
        endHeadingRecovery();
        fp.stagedCount = 0;
        fp.dispatchAfresh = true;
        resetLegState();
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

// Altitude-only fallback descent. Runs independently of the executor (fp.active):
// the switch-rescue staging-failure case has no GPS fix at all, so the position
// controller cannot run. alt-hold owns the throttle (its landing branch, gated on
// this being active); updateLanding() reuses the one landing touchdown detector
// and disarms on touchdown.
void flightPlanNavRescueDescent(bool request, timeUs_t currentTimeUs)
{
    if (!request) {
        if (fp.rescueDescentActive) {
            fp.rescueDescentActive = false;
            altHoldSetEmergencyDescent(false, 0.0f);
        }
        return;
    }
    if (!fp.rescueDescentActive) {
        fp.rescueDescentActive = true;
        fp.landingRateMps = MAX(FP_LANDING_MIN_RATE_MPS, gpsRescueConfig()->descendRate * 0.01f);
        fp.landingStartUs = currentTimeUs;
        fp.touchdownQuietStartUs = 0;
        fp.landingDescentEstablished = false;
        altHoldSetEmergencyDescent(true, (float)gpsRescueConfig()->descendRate);
    }
    // A wing's sink can stop in the air, which this would take for a touchdown.
#ifndef USE_WING
    updateLanding(currentTimeUs);
#endif
}

bool flightPlanNavIsRescueDescentActive(void)
{
    return fp.rescueDescentActive;
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
        startLandingInPlace(currentTimeUs);
    }
}

#ifndef USE_WING
static void patternCarrot(float phaseRad, float radiusM, vector3_t *out)
{
    out->v[ENU_U] = fp.patternCentreM.v[ENU_U];
    if (fp.patternType == WAYPOINT_PATTERN_FIGURE8) {
        // Lemniscate of Gerono — crosses the centre twice per cycle
        out->v[ENU_E] = fp.patternCentreM.v[ENU_E] + radiusM * sin_approx(phaseRad);
        out->v[ENU_N] = fp.patternCentreM.v[ENU_N] + radiusM * sin_approx(phaseRad) * cos_approx(phaseRad);
    } else {
        out->v[ENU_E] = fp.patternCentreM.v[ENU_E] + radiusM * cos_approx(phaseRad);
        out->v[ENU_N] = fp.patternCentreM.v[ENU_N] + radiusM * sin_approx(phaseRad);
    }
}

// The carrot command must never complete: completion would refire
// onWaypointReached (restarting the hold timer) and zero the velocity target.
// completionSpeed 0 keeps it permanently unreachable. Accel limits are lifted
// because the approach-decel braking term would cap chase speed to a crawl at
// carrot-lead distances; the next leg's dispatch restores them.
static void issuePatternCommand(float radiusM)
{
    vector3_t carrot;
    patternCarrot(fp.patternPhaseRad, radiusM, &carrot);
    const float startAltM = commandedAltitudeM(positionEstimatorGetEstimate());
    positionNavSetTargetEf(&carrot, fp.patternCruiseMps, radiusM, 0.0f, true, NULL, NULL);
    positionNavMoveTargetEf(&carrot);   // flown as a moving target from its first cycle
    positionNavSetAccelLimits(0.0f, 0.0f);
    positionNavSetVerticalProfile(fp.legVertRateMps, startAltM);
}

static void startHoldPattern(const waypoint_t *wp)
{
    const positionNavCommand_t *cmd = positionNavGetActiveCommand();
    const float radiusM = autopilotConfig()->waypointHoldRadius * 0.01f;

    fp.patternType = wp->pattern;
    fp.patternCentreM = cmd->targetPosEfM;
    fp.patternCruiseMps = cmd->cruiseSpeedMps;
    fp.patternSpeedMps = MIN(cmd->cruiseSpeedMps, radiusM * FP_PATTERN_MAX_RATE_RADS);

    if (wp->pattern == WAYPOINT_PATTERN_FIGURE8) {
        // The curve passes through the centre, which is where the vehicle is
        fp.patternPhaseRad = 0.0f;
    } else {
        // Start the carrot at the vehicle's azimuth from the centre so the
        // first move is outward onto the ring, not a dash across it
        const positionEstimate3d_t *est = positionEstimatorGetEstimate();
        fp.patternPhaseRad = atan2_approx(est->position.v[ENU_N] * 0.01f - fp.patternCentreM.v[ENU_N],
                                          est->position.v[ENU_E] * 0.01f - fp.patternCentreM.v[ENU_E]);
    }
    fp.patternLastUpdateUs = micros();
    fp.patternActive = true;
    issuePatternCommand(radiusM);
}

static void updateHoldPattern(timeUs_t currentTimeUs)
{
    const timeDelta_t sinceUs = cmpTimeUs(currentTimeUs, fp.patternLastUpdateUs);
    if (sinceUs < (timeDelta_t)FP_PATTERN_UPDATE_US) {
        return;
    }
    fp.patternLastUpdateUs = currentTimeUs;

    const float radiusM = autopilotConfig()->waypointHoldRadius * 0.01f;
    fp.patternPhaseRad += (fp.patternSpeedMps / radiusM) * MIN(sinceUs * 1e-6f, 1.0f);
    if (fp.patternPhaseRad > M_PIf) {
        fp.patternPhaseRad -= 2.0f * M_PIf;
    }

    if (positionNavHasActiveTarget()) {
        vector3_t carrot;
        patternCarrot(fp.patternPhaseRad, radiusM, &carrot);
        positionNavMoveTargetEf(&carrot);
    } else {
        // A position-control re-init wiped the carrot command; re-issue it
        issuePatternCommand(radiusM);
    }
}
#endif // !USE_WING

// Orbit period at the configured pattern radius for a leg flown at speedCmS
// (0 = autopilot max velocity — the same cruise derivation dispatch uses).
// MAVLink converts LOITER_TURNS turn counts to and from hold durations here.
uint16_t flightPlanNavOrbitPeriodDs(uint16_t speedCmS)
{
#ifdef USE_WING
    UNUSED(speedCmS);
    return flightPlanWingOrbitPeriodDs();
#else
    const float radiusM = autopilotConfig()->waypointHoldRadius * 0.01f;
    float cruiseMps = (speedCmS > 0) ? speedCmS * 0.01f : autopilotConfig()->maxVelocity * 0.01f;
    if (cruiseMps < FP_MIN_CRUISE_MPS) {
        cruiseMps = FP_MIN_CRUISE_MPS;
    }
    const float rateRads = MIN(cruiseMps, radiusM * FP_PATTERN_MAX_RATE_RADS) / radiusM;
    const float periodDs = 10.0f * 2.0f * M_PIf / rateRads;
    return (periodDs >= (float)UINT16_MAX) ? UINT16_MAX : (uint16_t)lrintf(periodDs);
#endif
}

static void advanceToNext(void)
{
    // The leg that's ending consumed any staged altitude override; clear it
    // before the next drainModifiers() pass so a new override (or none) is
    // staged cleanly for the upcoming leg.
    fp.altOverridePending = false;
    fp.patternPending = false;
    fp.patternActive = false;

    if (fp.currentIndex + 1 >= activePlanCount()) {
        completePlan();
        return;
    }
    fp.currentIndex++;
    // If dispatch fails (e.g. origin lost), state falls back to IDLE and the
    // update loop will retry; index has already advanced so we won't replay
    // the previous waypoint.
    dispatchWaypoint();
}

#if ENABLE_RESCUE_PLAN && !defined(USE_WING)
// Rescue climb complete but the IMU heading is untrusted: hold here and pitch forward so GPS
// course-over-ground can teach the estimator its heading before the return leg (legacy
// PITCH_FORWARD semantics).
static void startRescuePitchForward(void)
{
    fp.rescueBlindClimb = false;
    fp.rescueClimbed = false;
    fp.rescueHeadingHold = true;
    fp.rescueHeadingStartUs = micros();
    fp.rescueHeadingTrusted = false;
    autopilotHeadingRecovery(true, FP_RESCUE_PITCH_FORWARD_DEG);
}

static bool rescueDrifting(const positionEstimate3d_t *est)
{
    return sq(est->velocity.v[ENU_E]) + sq(est->velocity.v[ENU_N]) >= sq(FP_RESCUE_DRIFT_STILL_CMS);
}
#endif

#ifdef USE_WING
static void startWingLanding(void)
{
    fp.state = FP_NAV_LANDING;
    flightPlanWingStartLanding(micros());
}

// A wing cannot stop on a waypoint, so wherever a multirotor would wait on one it loiters: a HOLD,
// TAKEOFF or LAND for its duration, and any waypoint reached before a DELAY says it may be for the
// rest of the delay. A LAND then lands.
static void wingWaypointReached(const waypoint_t *wp)
{
    uint32_t holdDs = isStationKeepingType(wp->type) ? fp.holdDurationDs : 0;
    if (fp.delayActive) {
        const timeDelta_t earlyUs = cmpTimeUs(fp.delayEndUs, micros());
        holdDs += (earlyUs > 0) ? earlyUs / 100000 : 0;
    }
    if (holdDs == 0) {
        if (wp->type == WAYPOINT_TYPE_LAND) {
            startWingLanding();
        } else {
            advanceToNext();
        }
        return;
    }
    fp.state = FP_NAV_HOLDING;
    fp.holdStartUs = micros();
    fp.holdLastUpdateUs = fp.holdStartUs;
    fp.holdElapsedUs = 0;
    fp.holdDurationDs = MIN(holdDs, (uint32_t)UINT16_MAX);
    flightPlanWingHold();
}

typedef enum {
    WING_POSITION_VALID,
    WING_POSITION_RIDING_OUT,
    WING_POSITION_LOST,
} wingPosition_e;

// Without a position a wing cannot stop and wait: it circles where it is at the commanded height,
// with the leg's clocks stopped, and carries on with the leg if the position returns in time.
static wingPosition_e wingPosition(timeUs_t currentTimeUs)
{
    if (positionEstimatorIsValidXY()) {
        fp.positionLostUs = 0;
        return WING_POSITION_VALID;
    }
    if (fp.positionLostUs == 0) {
        fp.positionLostUs = currentTimeUs;
    }
    fp.holdLastUpdateUs = currentTimeUs;
    fp.lastProgressUs = currentTimeUs;
    return (cmpTimeUs(currentTimeUs, fp.positionLostUs) < (timeDelta_t)FP_WING_POSITION_LOSS_TIMEOUT_US)
        ? WING_POSITION_RIDING_OUT : WING_POSITION_LOST;
}

static void updateWingPlan(timeUs_t currentTimeUs)
{
    if (fp.state == FP_NAV_TARGETING) {
        float progressM;
        if (flightPlanWingUpdateLeg(positionEstimatorGetEstimate(), &progressM) == FPW_REACHED) {
            onWaypointReached(NULL);
        } else {
            updateProgressTracking(progressM, currentTimeUs);
        }
    } else if (fp.state == FP_NAV_HOLDING) {
        fp.holdElapsedUs += (timeUs_t)(currentTimeUs - fp.holdLastUpdateUs);
        fp.holdLastUpdateUs = currentTimeUs;
        if (fp.holdElapsedUs >= (uint64_t)fp.holdDurationDs * 100000u) {
            const waypoint_t *wp = currentWaypoint();
            if (wp != NULL && wp->type == WAYPOINT_TYPE_LAND) {
                startWingLanding();
            } else {
                advanceToNext();
            }
        }
    }
}

// The landing flies itself through a position loss, for as long as a leg would wait for it.
static void updateWingLanding(timeUs_t currentTimeUs)
{
    if (wingPosition(currentTimeUs) == WING_POSITION_LOST) {
        abortMission(FP_ABORT_ESTIMATOR);
    } else if (landingWingUpdate(currentTimeUs)) {
        completePlan();
        disarm(DISARM_REASON_LANDING);
    }
}
#endif

static void onWaypointReached(void *userData)
{
    UNUSED(userData);
    if (!fp.active) {
        return;
    }

    const waypoint_t *wp = currentWaypoint();
    if (wp == NULL) {
        completePlan();
        return;
    }

    // Listener indices refer to the PG mission; injected-plan progress is
    // meaningless to a MAVLink partner tracking the uploaded plan.
    if (reachedListener && fp.injectedCount == 0) {
        reachedListener(fp.currentIndex);
    }

#ifdef USE_WING
    wingWaypointReached(wp);
#else
#if ENABLE_RESCUE_PLAN
    if (isRescueStop() && activePlanCount() > 1 && !imuIsHeadingValid()) {
        if (fp.rescueBlindClimb && rescueDrifting(positionEstimatorGetEstimate())) {
            fp.rescueClimbed = true;
            fp.rescueClimbedUs = micros();
        } else {
            startRescuePitchForward();
        }
        return;
    }
    endHeadingRecovery();
#endif

    // A LAND duration is a pre-descent loiter and a TAKEOFF duration a
    // post-climb loiter; the hold-expiry path starts the descent (LAND) or
    // advances (HOLD, TAKEOFF).
    const bool holdsOnArrival = isStationKeepingType(wp->type);
    if (holdsOnArrival && fp.holdDurationDs > 0) {
        fp.state = FP_NAV_HOLDING;
        fp.holdStartUs = micros();
        fp.holdLastUpdateUs = fp.holdStartUs;
        fp.holdElapsedUs = 0;
        if (wp->type == WAYPOINT_TYPE_HOLD && wp->pattern != WAYPOINT_PATTERN_NONE) {
            // Deferred: the update loop starts the pattern once the arrival
            // braking has settled (see FP_PATTERN_START_SPEED_MPS).
            fp.patternPending = true;
        }
        return;
    }

    if (wp->type == WAYPOINT_TYPE_LAND) {
        // Touchdown completes the plan, so a mid-plan LAND is terminal.
        startLandingAtNavTarget(micros());
        return;
    }

    advanceToNext();
#endif
}

// The gate state is scoped to a waypoint index, so a new plan starting at the same index must not
// inherit the wait the old one had served.
static void clearLegYawState(void)
{
    fp.turnInRequested = false;
    fp.legTurnIn = false;
    fp.legTurnCapped = false;
    fp.legYawBehaviour = WAYPOINT_YAW_DEFAULT;
    fp.legYawGated = false;
    fp.legYawHolding = false;
    fp.legYawTimedOut = false;
    fp.legYawIndex = UINT8_MAX;
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
    fp.pendingStartIndex = 0;
    fp.active = false;
    fp.zBiasM = 0.0f;
    fp.abortReason = FP_ABORT_NONE;
    fp.injectedCount = 0;
    fp.patternPending = false;
    fp.patternActive = false;
    fp.hdgFaultTimeS = 0.0f;
    fp.lastUpdateUs = 0;
    fp.legIsPassGate = false;
    fp.legValid = false;
    fp.carrotSpeedMps = 0.0f;
    fp.carrotValid = false;
    fp.dispatchAfresh = false;
    fp.inPreTurn = false;
    fp.measFiltValid = false;
#if ENABLE_RESCUE_PLAN
    fp.stagedCount = 0;
    fp.isRescuePlan = false;
    fp.rescueBlindClimb = false;
    fp.rescueClimbed = false;
    fp.rescueHeadingHold = false;
    fp.rescueDescentActive = false;
    altHoldSetEmergencyDescent(false, 0.0f);
#endif
    clearModifierState();
    clearLegYawState();
}

void flightPlanNavEngage(void)
{
    fp.currentIndex = 0;
    fp.state = FP_NAV_IDLE;
    fp.abortReason = FP_ABORT_NONE;
    // Engagement always starts the PG mission; an injected plan does not
    // survive a switch cycle (no resume).
    fp.injectedCount = 0;
    fp.patternPending = false;
    fp.patternActive = false;
    fp.hdgFaultTimeS = 0.0f;
    fp.lastUpdateUs = 0;
    fp.legIsPassGate = false;
    fp.legValid = false;
    fp.carrotSpeedMps = 0.0f;
    fp.carrotValid = false;
    fp.dispatchAfresh = false;
    fp.inPreTurn = false;
    fp.measFiltValid = false;
    autopilotForceLevelPark(false);   // a fresh engage clears any latched heading-fault park
    autopilotSetNavHeadingOverride(false, 0.0f);
    resetLegState();

    captureAltitudeFrame();

    // HOLD waypoints keep the position target live for the hold duration; a
    // new target replaces the previous on advance anyway, so this module
    // never wants positionNav to auto-clear the target on reach.
    positionNavSetAutoClearOnReach(false);

#if ENABLE_RESCUE_PLAN
    fp.isRescuePlan = false;
    endHeadingRecovery();
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
        completePlan();
        fp.active = true;
        return;
    }

    // A MISSION_SET_CURRENT received while idle chooses the starting leg.
    fp.currentIndex = (fp.pendingStartIndex < flightPlanConfig()->waypointCount) ? fp.pendingStartIndex : 0;
    fp.pendingStartIndex = 0;
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
    fp.patternPending = false;
    fp.patternActive = false;
    fp.hdgFaultTimeS = 0.0f;
    fp.lastUpdateUs = 0;
    fp.legIsPassGate = false;
    fp.legValid = false;
    fp.carrotSpeedMps = 0.0f;
    fp.carrotValid = false;
    fp.dispatchAfresh = false;
    fp.inPreTurn = false;
    fp.measFiltValid = false;
    autopilotForceLevelPark(false);
    autopilotSetNavHeadingOverride(false, 0.0f);
#if ENABLE_RESCUE_PLAN
    fp.stagedCount = 0;
    fp.isRescuePlan = false;
    endHeadingRecovery();
#endif
    resetLegState();
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
    fp.carrotValid = false;
    fp.dispatchAfresh = true;
    fp.carrotSpeedMps = 0.0f;
    fp.measFiltValid = false;
#if ENABLE_RESCUE_PLAN
    fp.isRescuePlan = false;
#endif
    resetLegState();
    // If dispatch fails (GPS origin lost) the state falls back to IDLE and
    // the update loop retries against the injected plan.
    dispatchWaypoint();
    return true;
}

bool flightPlanNavIsInjectedPlanActive(void)
{
    return fp.active && fp.injectedCount > 0;
}

#if ENABLE_RESCUE_PLAN
static bool rescueHeadingSettled(timeUs_t currentTimeUs)
{
    if (!fp.rescueHeadingTrusted) {
        fp.rescueHeadingTrusted = true;
        fp.rescueHeadingTrustedUs = currentTimeUs;
        fp.rescueHeadingOffCourseUs = currentTimeUs;
    }
    const float offCourseDeg = fabsf(wrapDeg180f((attitude.values.yaw - gpsSol.groundCourse) * 0.1f));
    if (gpsSol.groundSpeed < FP_RESCUE_HEADING_SETTLE_CMS || offCourseDeg > FP_RESCUE_HEADING_SETTLE_DEG) {
        fp.rescueHeadingOffCourseUs = currentTimeUs;
    }
    return cmpTimeUs(currentTimeUs, fp.rescueHeadingOffCourseUs) >= (timeDelta_t)FP_RESCUE_HEADING_SETTLE_US
        || cmpTimeUs(currentTimeUs, fp.rescueHeadingTrustedUs) >= (timeDelta_t)FP_RESCUE_HEADING_SETTLE_MAX_US;
}
#endif

void flightPlanNavUpdate(timeUs_t currentTimeUs)
{
    if (!fp.active) {
        return;
    }

    DEBUG_SET(DEBUG_FLIGHT_PLAN, 0, fp.state);         //!< Executor State [enum:flightPlanNavState_e]
    DEBUG_SET(DEBUG_FLIGHT_PLAN, 1, fp.abortReason);   //!< Abort Reason [enum:flightPlanAbortReason_e]
    DEBUG_SET(DEBUG_FLIGHT_PLAN, 2, fp.currentIndex);  //!< Waypoint Index

    const float dtS = (fp.lastUpdateUs != 0)
        ? constrainf(cmpTimeUs(currentTimeUs, fp.lastUpdateUs) * 1e-6f, 0.0f, 0.25f)
        : 0.0f;
    fp.lastUpdateUs = currentTimeUs;

#if ENABLE_RESCUE_PLAN
    if (fp.rescueHeadingHold) {
        // Leg progress/stall checks are meaningless while deliberately
        // pitching forward away from the climb waypoint.
        if (imuIsHeadingValid()) {
            if (rescueHeadingSettled(currentTimeUs)) {
                endHeadingRecovery();
                fp.dispatchAfresh = true;
                fp.turnInRequested = true;
                advanceToNext();
            }
        } else if (cmpTimeUs(currentTimeUs, fp.rescueHeadingStartUs) >= (timeDelta_t)FP_RESCUE_HEADING_TIMEOUT_US) {
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
        // Position control re-initialised and wiped the target: a carrot leg
        // starts afresh from what the craft is doing, not from a stale carrot.
        fp.carrotValid = false;
        fp.carrotSpeedMps = 0.0f;
        fp.measFiltValid = false;
        dispatchWaypoint();
        return;
    }

    if (fp.state == FP_NAV_LANDING) {
#ifdef USE_WING
        updateWingLanding(currentTimeUs);
#else
        updateLanding(currentTimeUs);
#endif
        return;
    }

    if (fp.state == FP_NAV_TARGETING || fp.state == FP_NAV_HOLDING) {
#ifdef USE_WING
        if (wingPosition(currentTimeUs) == WING_POSITION_RIDING_OUT) {
            return;
        }
#endif
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

#ifdef USE_WING
    UNUSED(dtS);
    updateWingPlan(currentTimeUs);
#else
#if ENABLE_RESCUE_PLAN
    if (fp.state == FP_NAV_TARGETING && fp.rescueBlindClimb) {
        if (!headingUnknown()) {
            // A heading after all (a compass back): fly the stop and climb with it, nose held until braked.
            endHeadingRecovery();
            fp.dispatchAfresh = true;
            fp.legYawIndex = UINT8_MAX;
            dispatchWaypoint();
            return;
        }
        // Only the climb means anything: the target rides with the craft, so the progress and
        // flyaway checks judge the altitude, and the arrival test the altitude alone.
        const positionEstimate3d_t *est = positionEstimatorGetEstimate();
        const vector3_t atCraftM = {.v = {
            [ENU_E] = est->position.v[ENU_E] * 0.01f,
            [ENU_N] = est->position.v[ENU_N] * 0.01f,
            [ENU_U] = fp.legTargetEnuM.v[ENU_U],
        }};
        positionNavMoveTargetEf(&atCraftM);
        if (!fp.rescueClimbed) {
            checkLegProgress(currentTimeUs, est);
        } else if (!rescueDrifting(est)
                   || cmpTimeUs(currentTimeUs, fp.rescueClimbedUs) >= (timeDelta_t)FP_RESCUE_DRIFT_TIMEOUT_US) {
            startRescuePitchForward();
        }
        return;
    }
#endif

    if (fp.state == FP_NAV_TARGETING) {
        const positionEstimate3d_t *est = positionEstimatorGetEstimate();
        updateLegYaw(est);
        if (fp.legYawHolding && !fp.legYawGated) {
            dispatchWaypoint();   // nose is on the leg: swap the hold for the real target
            return;
        }
        if (fp.legIsPassGate) {
            updateLegCarrot(dtS, currentTimeUs, est);   // marches the carrot, owns gate advance + sanity
        } else {
            checkLegProgress(currentTimeUs, est);
        }
        if (fp.state == FP_NAV_TARGETING && positionNavHasActiveTarget()
            && checkHeadingFault(dtS, est)) {
            autopilotForceLevelPark(true);
            abortMission(FP_ABORT_MAG_FAULT);
            return;
        }
    }

    if (fp.state == FP_NAV_HOLDING) {
        // Accumulate per-update deltas so durations may exceed one full turn
        // of the 32-bit micros() clock.  Unsigned subtraction also preserves
        // the delta for an individual update that crosses the wrap point.
        fp.holdElapsedUs += (timeUs_t)(currentTimeUs - fp.holdLastUpdateUs);
        fp.holdLastUpdateUs = currentTimeUs;
        updateLegYaw(positionEstimatorGetEstimate());
        if (fp.patternPending) {
            const waypoint_t *wp = currentWaypoint();
            if (!positionNavHasActiveTarget() || wp == NULL) {
                // The hold command was wiped while settling; degrade to a
                // plain hold rather than orbit an unknown centre.
                fp.patternPending = false;
            } else {
                const positionEstimate3d_t *est = positionEstimatorGetEstimate();
                const float hSpeedMps = sqrtf(sq(est->velocity.v[ENU_E]) + sq(est->velocity.v[ENU_N])) * 0.01f;
                if (hSpeedMps < FP_PATTERN_START_SPEED_MPS
                    || cmpTimeUs(currentTimeUs, fp.holdStartUs) >= (timeDelta_t)FP_PATTERN_START_TIMEOUT_US) {
                    fp.patternPending = false;
                    startHoldPattern(wp);
                }
            }
        } else if (fp.patternActive) {
            updateHoldPattern(currentTimeUs);
        }
        const uint64_t holdUs = (uint64_t)fp.holdDurationDs * 100000u;
        if (fp.holdElapsedUs >= holdUs) {
            const waypoint_t *wp = currentWaypoint();
            if (wp != NULL && wp->type == WAYPOINT_TYPE_LAND) {
                startLandingAtNavTarget(currentTimeUs);
            } else {
                advanceToNext();
            }
        }
    }
#endif
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

float flightPlanNavGetDistanceToWaypointM(void)
{
    if (!fp.active || !positionNavHasActiveTarget()) {
        return -1.0f;
    }
    vector3_t deltaM;
    navWaypointDeltaEnuM(positionEstimatorGetEstimate(), &deltaM);
    return vector3Norm(&deltaM);
}

int32_t flightPlanNavGetBearingToWaypointDeciDeg(void)
{
    if (!fp.active || !positionNavHasActiveTarget()) {
        return -1;
    }
    vector3_t deltaM;
    navWaypointDeltaEnuM(positionEstimatorGetEstimate(), &deltaM);
    // Compass bearing: 0 = north, growing clockwise. ENU east/north maps to
    // atan2(E, N); wrap into [0, 3600) deci-degrees.
    int32_t deciDeg = lrintf(atan2_approx(deltaM.v[ENU_E], deltaM.v[ENU_N]) * (1800.0f / M_PIf));
    deciDeg %= 3600;
    if (deciDeg < 0) {
        deciDeg += 3600;
    }
    return deciDeg;
}

uint16_t flightPlanNavGetEtaSeconds(void)
{
    if (!fp.active || !positionNavHasActiveTarget()) {
        return 0;
    }
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float speedMps = sqrtf(sq(est->velocity.v[ENU_E]) + sq(est->velocity.v[ENU_N])) * 0.01f;
    if (speedMps < 0.5f) {
        return 0;
    }
    // Horizontal groundspeed only reaches the waypoint's horizontal offset, so
    // divide by the 2D distance; the 3D slant would overstate ETA on climbs.
    vector3_t deltaM;
    navWaypointDeltaEnuM(est, &deltaM);
    const float distance2dM = sqrtf(sq(deltaM.v[ENU_E]) + sq(deltaM.v[ENU_N]));
    const float etaS = distance2dM / speedMps;
    return (etaS >= (float)UINT16_MAX) ? UINT16_MAX : (uint16_t)lrintf(etaS);
}

bool flightPlanNavSetCurrentIndex(uint8_t index)
{
    // SET_CURRENT addresses the uploaded PG mission; an injected runtime plan
    // (geofence RTH / failsafe rescue) owns its own sequencing.
    if (fp.injectedCount > 0 || index >= flightPlanConfig()->waypointCount) {
        return false;
    }

    if (fp.active) {
        fp.currentIndex = index;
        fp.patternPending = false;
        fp.patternActive = false;
        fp.abortReason = FP_ABORT_NONE;
        fp.carrotValid = false;
        fp.dispatchAfresh = true;
        fp.carrotSpeedMps = 0.0f;
        fp.measFiltValid = false;
        resetLegState();
        fp.state = FP_NAV_TARGETING;
        dispatchWaypoint();
    } else {
        fp.pendingStartIndex = index;
    }
    return true;
}

void flightPlanNavSetReachedListener(flightPlanWaypointReachedFn fn)
{
    reachedListener = fn;
}

#endif // ENABLE_FLIGHT_PLAN

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

// Waypoint mission sequencer. Waypoints are dropped in flight with the USER2
// switch (see nav_hud.c); USER3 alone flies them, in order, nose-first:
//
//  - the craft first yaws to face the waypoint (GPS Rescue's yaw controller,
//    gps_rescue_yaw_p), holding position while it turns
//  - once roughly aligned, a carrot target marches from the craft toward the
//    waypoint at gps_rescue_ground_speed; the position-hold autopilot chases
//    it, so the same tuning that makes rescue fly well makes missions fly well
//  - within 5 m of a waypoint the next leg begins; after the last waypoint
//    the craft parks on it and holds until the switch is released
//
// Engaging USER3 self-enables the position and altitude hold engines - no
// separate hold switches needed - and captures the current altitude. The
// pilot always wins: a deliberate stick deflection aborts, as do failsafe,
// GPS Rescue, and losing the fix. Every abort latches until a switch cycle.

#include "platform.h"

#ifdef USE_NAV_MISSION

#include "build/debug.h"

#include <math.h>

#include "common/axis.h"
#include "common/maths.h"
#include "common/vector.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/nav_hud.h"
#include "flight/pos_hold.h"
#include "flight/position.h"

#include "io/gps.h"

#include "pg/gps_rescue.h"
#include "pg/nav_hud.h"

#include "nav_mission.h"

#define NAV_MISSION_ARRIVE_CM          500.0f    // final/descend waypoints: reached inside this radius
#define NAV_MISSION_PASS_CM            1200.0f   // intermediate waypoints: pass-through gate, corner is carved
#define NAV_MISSION_CORNER_SPEED_CMS   220.0f    // ~5 mph floor across a waypoint gate
#define NAV_MISSION_CORNER_DV_CMS      440.0f    // constant velocity-change budget per corner: every gate
                                                 // sheds the SAME delta-v, so worst-case wide excursion is
                                                 // identical for a gentle bend at cruise and a hairpin at
                                                 // crawl (hairpin works out to the 220 floor exactly)
#define NAV_MISSION_DECEL_CMSS         100.0f    // constant-deceleration approach profile. Deliberately
                                                 // HALF what the craft could pull in isolation: the craft
                                                 // brakes through chase dynamics (gap collapse + drag), and
                                                 // a profile it can actually track is what guarantees gates
                                                 // are crossed at their corner speed, not just commanded
#define NAV_MISSION_ACCEL_CMSS         250.0f    // carrot speed slew: no full-cruise leaps at leg starts
#define NAV_MISSION_MAX_HOME_CM        200000.0f // refuse any waypoint beyond 2 km from home
#define NAV_MISSION_CARROT_LEAD_CM     2500.0f   // carrot never runs further ahead of the craft
#define NAV_MISSION_SANITY_CM          (NAV_MISSION_CARROT_LEAD_CM + 2500.0f)
#define NAV_MISSION_RESUME_CM          10000.0f  // re-engage within this of where the mission was interrupted = resume, else restart at wp1
#define NAV_MISSION_STICK_ABORT        0.15f     // deliberate deflection, well above trim noise
#define NAV_MISSION_MAX_YAW_RATE_DPS   180.0f    // same cap GPS Rescue uses
#define NAV_MISSION_MIN_ENGAGE_ALT_CM  300       // refuse to engage on the ground
#define NAV_MISSION_ALT_ARRIVE_CM      250      // vertical arrival tolerance
#define NAV_MISSION_PRETURN_CM         1500.0f   // blend the nose onto the next leg through this approach zone
#define NAV_MISSION_DECK_MARGIN_CM     200.0f    // cruise target sits this far above the deck so sag stays above it
#define NAV_MISSION_DIP_MAX_SPEED_CMS  100       // dips descend only once horizontal motion is arrested
#define NAV_MISSION_ALT_ACCEL_CMSS     250.0f    // climb-rate slew: continuous feed-forward into altitudeControl
#define NAV_MISSION_ALT_TAPER_HZ       1.0f      // proportional climb-rate taper approaching the target altitude
#define NAV_MISSION_HDG_FAULT_ERR_DEG  70.0f     // COG vs heading disagreement that means the compass is lying
#define NAV_MISSION_HDG_FAULT_TIME_S   2.0f      // sustained this long before tripping
#define NAV_MISSION_HDG_FAULT_MIN_SPEED_CMS 300  // COG is only meaningful at speed
#define NAV_MISSION_HDG_FAULT_ALIGNED_DEG   30.0f // only judge while the nose is where we commanded it
#define NAV_MISSION_OVERSPEED_CMS      150.0f    // measured-speed governor margin over the profile speed
#define NAV_MISSION_OVERRUN_LAT_CM     800.0f    // fast miss of an arrive bubble still counts within this lateral corridor

static navMissionPhase_e phase = NAV_MISSION_IDLE;
static unsigned missionIndex = 0;
static bool switchWas = false;
static vector2_t legStartCm;       // anchor of the current leg: the flown line is legStart -> waypoint
static float legProgressCm = 0.0f; // how far along the leg the carrot has marched
static bool legValid = false;
static float yawRateDps = 0.0f;
static timeUs_t lastTimeUs = 0;
static float carrotSpeedCmS = 0.0f;   // slewed carrot speed: the acceleration limit lives here
static vector2_t carrotPrevCm;        // last commanded carrot: the next leg anchors on it
static bool carrotPrevValid = false;
static timeUs_t retryTimeUs = 0;      // time of the single position-control re-engage
static bool retryPending = false;     // failure noticed; re-anchor once a fresh fix has arrived
#ifdef USE_POSITION_HOLD
static uint16_t retryGpsStamp = 0;    // tracks fix arrival while a retry is pending
#endif
static bool missionFailed = false;    // latched: position control died twice; pilot must take over
static float hdgFaultTimeS = 0.0f;    // heading-fault detector integrator
static vector2_t resumeAnchorCm;      // craft position while flying: where a re-engage may resume from
static bool resumeAnchorValid = false;
// 3D (stadium) profile: cruise at max(waypoint altitude, rescue return alt),
// descend only directly overhead the waypoint, climb back before the next leg
static bool altActive = false;
static float altTargetCm = 0.0f;
static float altRateCmS = 0.0f;
static bool descending = false;

extern float GPS_cosLat;   // gps.c: longitude shrink factor at the current latitude

#ifdef UNIT_TEST
bool testMissionSwitch = false;
static bool missionSwitchActive(void) { return testMissionSwitch; }
#else
static bool missionSwitchActive(void) { return IS_RC_MODE_ACTIVE(BOXUSER3); }
#endif

static bool sticksDeflected(void)
{
    return getRcDeflectionAbs(FD_ROLL) > NAV_MISSION_STICK_ABORT
        || getRcDeflectionAbs(FD_PITCH) > NAV_MISSION_STICK_ABORT;
}

static bool missionBlocked(void)
{
    return failsafeIsActive() || FLIGHT_MODE(GPS_RESCUE_MODE) || !STATE(GPS_FIX);
}

static bool waypointsWithinFence(void)
{
    for (unsigned i = 0; i < navHudWaypointCount(); i++) {
        vector2_t wp = *navHudWaypointAt(i);
        if (vector2Norm(&wp) > NAV_MISSION_MAX_HOME_CM) {
            return false;
        }
    }
    return true;
}

// exact inverse of GPS_distance2d(): local ENU cm relative to home -> lat/lon
static void enuToLocation(const vector2_t *enuCm, gpsLocation_t *loc)
{
    *loc = GPS_home_llh;
    loc->lat = GPS_home_llh.lat + lrintf(enuCm->y / EARTH_ANGLE_TO_CM);
    loc->lon = GPS_home_llh.lon + lrintf(enuCm->x / (EARTH_ANGLE_TO_CM * fmaxf(GPS_cosLat, 0.01f)));
}

static float missionFloorCm(void)
{
    return gpsRescueConfig()->returnAltitudeM * 100.0f;
}

static float legCruiseAltCm(void)
{
    // the margin keeps the ACTUAL altitude above the deck: the target alone
    // at deck height would sag below it under horizontal acceleration
    return fmaxf((float)navHudWaypointAltCm(missionIndex), missionFloorCm() + NAV_MISSION_DECK_MARGIN_CM);
}

static float wrapDeg180f(float deg)
{
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg - 180.0f;
}

// GPS Rescue's yaw law verbatim: errorAngle = heading MINUS bearing, a
// positive yaw-rate setpoint reduces a positive error; same P gain, same cap,
// same yaw_control_reversed handling (gps_rescue_multirotor.c)
static float missionYawRateDps(float errorDeg)
{
    const float rate = constrainf(errorDeg * gpsRescueConfig()->yawP / 10.0f,
                                  -NAV_MISSION_MAX_YAW_RATE_DPS, NAV_MISSION_MAX_YAW_RATE_DPS);
    return rate * GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
}

// slew the altitude target toward the goal at rescue's climb/descend rates.
// The rate itself ramps at NAV_MISSION_ALT_ACCEL_CMSS and tapers near the
// goal, so the feed-forward step into altitudeControl is always continuous
// (the old +/-rate-or-zero version stepped the throttle at every transition)
static void rampAltTarget(float desiredCm, float dtS)
{
    const float deltaCm = desiredCm - altTargetCm;
    // proportional taper near the goal, bounded by a no-overshoot approach
    // curve, bounded by rescue's configured climb/descend rates
    float mag = fminf(fabsf(deltaCm) * NAV_MISSION_ALT_TAPER_HZ,
                      sqrtf(2.0f * NAV_MISSION_ALT_ACCEL_CMSS * fabsf(deltaCm)));
    mag = fminf(mag, (deltaCm > 0.0f) ? (float)gpsRescueConfig()->ascendRate
                                      : (float)gpsRescueConfig()->descendRate);
    const float desiredRate = copysignf(mag, deltaCm);
    const float maxStep = NAV_MISSION_ALT_ACCEL_CMSS * dtS;
    altRateCmS = constrainf(desiredRate, altRateCmS - maxStep, altRateCmS + maxStep);
    altTargetCm += altRateCmS * dtS;
    if (fabsf(desiredCm - altTargetCm) < 2.0f && fabsf(altRateCmS) < 2.0f) {
        altTargetCm = desiredCm;
        altRateCmS = 0.0f;
    }
}

static void holdHere(void)
{
    posControlSetMissionTarget(&gpsSol.llh, 0.0f);
}

static void abortMission(void)
{
    yawRateDps = 0.0f;
    altActive = false;   // altitude hold reverts to stock behaviour instantly
    descending = false;
    phase = NAV_MISSION_ABORTED;
}

// blackbox substate for debug[0] = phase*10 + substate. The v29 log froze all
// debug on every early-return path, making dips, retries, DONE and FAILED
// indistinguishable; this is written every cycle regardless of path.
// 0 cruise/none, 2 dip-arrest, 3 dip-descend, 4 retry, 5 climb-gate, 6 failed
static uint8_t navSubState = 0;

void navMissionUpdate(timeUs_t currentTimeUs)
{
    float dtS = (lastTimeUs != 0) ? (cmpTimeUs(currentTimeUs, lastTimeUs) * 1e-6f) : 0.0f;
    dtS = constrainf(dtS, 0.0f, 0.25f);
    lastTimeUs = currentTimeUs;

    DEBUG_SET(DEBUG_NAV_MISSION, 0, (int16_t)(phase * 10 + navSubState));
    DEBUG_SET(DEBUG_NAV_MISSION, 1, (int16_t)missionIndex);

    const bool sw = missionSwitchActive();

    if (!sw) {
        if (phase == NAV_MISSION_FLYING) {
            holdHere();   // stop cleanly where we are, not at the carrot
        }
        phase = NAV_MISSION_IDLE;
        navSubState = 0;
        yawRateDps = 0.0f;
        altActive = false;
        descending = false;
        missionFailed = false;   // the switch cycle acknowledges the failure
        switchWas = false;
        return;
    }

    if (!switchWas) {
        // rising edge: engage (or refuse). A fresh engage restarts the route at
        // waypoint 1, EXCEPT a genuine in-air resume: if the mission was
        // interrupted mid-route (cancelled to watch, or aborted) and the craft
        // is still near where it left off, continue from the current waypoint -
        // so "cancel after wp3, then flip back on to continue to wp4" works.
        // A completed run (missionIndex has reached the count) or a craft flown
        // elsewhere restarts at wp1 - that is what stops a relocated craft from
        // flying straight to a stale late waypoint (the field bug: moved the
        // craft, re-engaged, and it flew to the 6th of 6 instead of the 1st).
        bool resume = false;
        if (missionIndex > 0 && missionIndex < navHudWaypointCount()
            && resumeAnchorValid && STATE(GPS_FIX_HOME)) {
            vector2_t craftNowCm;
            GPS_distance2d(&GPS_home_llh, &gpsSol.llh, &craftNowCm);
            const vector2_t d = { .x = craftNowCm.x - resumeAnchorCm.x,
                                  .y = craftNowCm.y - resumeAnchorCm.y };
            resume = vector2Norm(&d) < NAV_MISSION_RESUME_CM;
        }
        if (!resume) {
            missionIndex = 0;
        }
        legValid = false;
        carrotPrevValid = false;
        carrotSpeedCmS = 0.0f;
        retryTimeUs = 0;
        retryPending = false;
        missionFailed = false;
        hdgFaultTimeS = 0.0f;
        navSubState = 0;
        if (navHudWaypointCount() == 0 || !waypointsWithinFence() || missionBlocked()
            || !ARMING_FLAG(ARMED) || !STATE(GPS_FIX_HOME)
            || getEstimatedAltitudeCm() < NAV_MISSION_MIN_ENGAGE_ALT_CM) {
            phase = NAV_MISSION_ABORTED;
        } else {
            phase = NAV_MISSION_FLYING;
            descending = false;
            altActive = navHudConfig()->mission3D;
            if (altActive) {
                altTargetCm = getEstimatedAltitudeCm();
                altRateCmS = 0.0f;
            }
#ifdef USE_POSITION_HOLD
            // a fresh anchor at the current location: if the pilot's own POS
            // HOLD switch was already on there is no enable edge, and the
            // mission would inherit a stale hold target and sanity radius
            posHoldRetrigger();
#endif
        }
    }
    switchWas = sw;

    if (phase != NAV_MISSION_FLYING && phase != NAV_MISSION_DONE) {
        return;   // ABORTED leaves the pilot in charge until a switch cycle
    }

#ifdef USE_POSITION_HOLD
    // the position engine underneath us latched a sanity/sensor failure, or
    // the heading estimate is provably wrong (mag fault): re-engage ONCE at
    // the current location; any second trip this engagement fails the mission
    // to wings-level (positionControlRelax decays the frozen lean angle away)
    // and only a switch cycle re-arms it
    if (posHoldControlLost() || hdgFaultTimeS > NAV_MISSION_HDG_FAULT_TIME_S) {
        if (retryTimeUs != 0) {
            // failed twice: give up on the route but PARK the craft. core.c
            // forces ANGLE + ALT hold on navMissionFailed() (NOT pos hold - a
            // failure is often a heading/GPS fault and position hold leans on
            // heading, which would fly it sideways), so it self-levels and
            // holds altitude instead of coasting on at its last bank angle. A
            // switch cycle hands full control back.
            missionFailed = true;
            navSubState = 6;
            abortMission();
            return;
        }
        navSubState = 4;
        yawRateDps = 0.0f;
        // re-anchor only on a FRESH fix: the fix that tripped the failure may
        // be an outlier, and anchoring on it would guarantee a second trip
        if (!retryPending) {
            retryPending = true;
            gpsHasNewData(&retryGpsStamp);   // consume the current (suspect) fix
            return;
        }
        if (!gpsHasNewData(&retryGpsStamp)) {
            return;   // hold quietly until new GPS data arrives
        }
        retryPending = false;
        retryTimeUs = currentTimeUs;
        posHoldRetrigger();
        legValid = false;
        carrotPrevValid = false;
        carrotSpeedCmS = 0.0f;
        hdgFaultTimeS = 0.0f;
        return;   // one clean cycle while the engine re-anchors
    }
    retryPending = false;
#endif

    if (phase == NAV_MISSION_DONE) {
        // parked: keep the altitude ramp running so the climb-rate
        // feed-forward settles to zero and the target reaches the final
        // waypoint's exact recorded altitude
        if (altActive && navHudWaypointCount() > 0) {
            rampAltTarget((float)navHudWaypointAltCm(navHudWaypointCount() - 1), dtS);
        }
        return;
    }

    if (missionBlocked() || !ARMING_FLAG(ARMED)) {
        holdHere();
        abortMission();
        return;
    }
    if (sticksDeflected()) {
        abortMission();
        return;
    }

    vector2_t craftCm;
    GPS_distance2d(&GPS_home_llh, &gpsSol.llh, &craftCm);
    // remember where the mission is actively flying: a later re-engage resumes
    // from the current waypoint only if the craft is still near this point
    resumeAnchorCm = craftCm;
    resumeAnchorValid = true;

    // waypoint targeting. Intermediate waypoints are pass-through gates so
    // corners are carved at speed; the final waypoint (and any 3D descend
    // waypoint) demands a precise arrival. Advancing past a gate loops so the
    // new leg and its yaw are computed in this SAME cycle - no stale cycle.
    const vector2_t *wp;
    vector2_t toWp;
    float distCm;
    bool lastWaypoint;
    bool passGate;
    float arriveCm;
    float cornerSpeedCmS;
    for (;;) {
        wp = navHudWaypointAt(missionIndex);
        toWp.x = wp->x - craftCm.x;
        toWp.y = wp->y - craftCm.y;
        distCm = vector2Norm(&toWp);

        lastWaypoint = (missionIndex + 1 >= navHudWaypointCount());
        const float wpAltNowCm = (float)navHudWaypointAltCm(missionIndex);
        // waypoints below the hard deck are deliberate low passes: cruise at
        // the deck, stop overhead, dip vertically, then climb back out
        const bool belowDeck = altActive && (wpAltNowCm < missionFloorCm() - NAV_MISSION_ALT_ARRIVE_CM);
        passGate = !(lastWaypoint || belowDeck);
        cornerSpeedCmS = NAV_MISSION_CORNER_SPEED_CMS;
        arriveCm = NAV_MISSION_ARRIVE_CM;
        if (passGate) {
            // turn-angle-scaled corner speed on a constant delta-v budget:
            // the velocity change a corner demands is 2*v*sin(turn/2), so
            // v_corner = DV / (2*sin(turn/2)) sheds the same delta-v at every
            // gate - a shallow bend carried at cruise runs no wider than a
            // hairpin taken at the floor speed. The gate radius scales with
            // the corner speed for the same reason: sharp corners switch legs
            // close in (small corner cut - canyon walls), gentle bends flow
            // through wide.
            const vector2_t *next = navHudWaypointAt(missionIndex + 1);
            const vector2_t inFrom = legValid ? legStartCm : craftCm;
            vector2_t inVec = { .x = wp->x - inFrom.x, .y = wp->y - inFrom.y };
            vector2_t outVec = { .x = next->x - wp->x, .y = next->y - wp->y };
            const float inLen = vector2Norm(&inVec);
            const float outLen = vector2Norm(&outVec);
            if (inLen > 1.0f && outLen > 1.0f) {
                const float cosTurn = (inVec.x * outVec.x + inVec.y * outVec.y) / (inLen * outLen);
                const float shedFactor = sqrtf(fmaxf(2.0f - 2.0f * cosTurn, 0.0f)); // = 2*sin(turn/2)
                cornerSpeedCmS = (shedFactor > 0.05f)
                    ? constrainf(NAV_MISSION_CORNER_DV_CMS / shedFactor,
                                 NAV_MISSION_CORNER_SPEED_CMS,
                                 (float)gpsRescueConfig()->groundSpeedCmS)
                    : (float)gpsRescueConfig()->groundSpeedCmS;   // straight-through
            }
            arriveCm = constrainf(cornerSpeedCmS * 1.5f, NAV_MISSION_ARRIVE_CM, NAV_MISSION_PASS_CM);
        }

        // overrun fallback: a fast crossing that misses the arrive bubble by
        // a hair (wp6 in the v29 log: 6.1 m abeam of a 5 m bubble at 11 m/s)
        // must still count once the craft is past the waypoint along-track,
        // inside a sane lateral corridor. Without it the craft overruns the
        // pinned carrot, stalls, and yaws back - the pilot-abort signature.
        bool overranWp = false;
        if (legValid) {
            vector2_t inVec = { .x = wp->x - legStartCm.x, .y = wp->y - legStartCm.y };
            const float inLen = vector2Norm(&inVec);
            if (inLen > 1.0f) {
                const float ux = inVec.x / inLen;
                const float uy = inVec.y / inLen;
                const float alongIn = (craftCm.x - legStartCm.x) * ux + (craftCm.y - legStartCm.y) * uy;
                const float lateralIn = fabsf((craftCm.y - legStartCm.y) * ux - (craftCm.x - legStartCm.x) * uy);
                overranWp = alongIn > inLen && lateralIn < NAV_MISSION_OVERRUN_LAT_CM;
            }
        }

        if (distCm >= arriveCm && !overranWp) {
            break;   // still flying toward this waypoint
        }

        // horizontally overhead the waypoint. Below-deck waypoints (and the
        // final waypoint) are moved to vertically: dip to the recorded
        // altitude, count the waypoint, then the next leg climbs back out
        if (altActive && (belowDeck || lastWaypoint)) {
            const float wpAltCm = wpAltNowCm;
            // the waypoint only counts once the target has fully ramped down
            // AND the craft is actually at the recorded altitude: a dip is a
            // real touch, not a near miss 5 m above it
            if (altTargetCm > wpAltCm + 2.0f
                || fabsf((float)getEstimatedAltitudeCm() - wpAltCm) > NAV_MISSION_ALT_ARRIVE_CM) {
                gpsLocation_t overheadLoc;
                enuToLocation(wp, &overheadLoc);
                posControlSetMissionTarget(&overheadLoc, NAV_MISSION_SANITY_CM);
                carrotPrevCm = *wp;   // the next leg starts at the dip point
                carrotPrevValid = true;
                // the craft stops here; bleed the carrot speed off smoothly so
                // the climb-out re-accelerates from rest without a target step
                carrotSpeedCmS = fmaxf(carrotSpeedCmS - NAV_MISSION_ACCEL_CMSS * dtS, 0.0f);
                if (!lastWaypoint) {
                    // rotate toward the NEXT waypoint while dipping, so the
                    // climb-out departs nose-first without a pause
                    const vector2_t *next = navHudWaypointAt(missionIndex + 1);
                    vector2_t toNext = { .x = next->x - craftCm.x, .y = next->y - craftCm.y };
                    const float nextBearingDeg = RADIANS_TO_DEGREES(atan2_approx(toNext.x, toNext.y));
                    yawRateDps = missionYawRateDps(wrapDeg180f(attitude.values.yaw / 10.0f - nextBearingDeg));
                } else {
                    yawRateDps = 0.0f;   // final waypoint: hold heading and park
                }
                if (gpsSol.groundSpeed > NAV_MISSION_DIP_MAX_SPEED_CMS) {
                    // arrest horizontal motion at cruise altitude first:
                    // dips are flown straight down, never on a diagonal
                    descending = false;
                    rampAltTarget(legCruiseAltCm(), dtS);
                } else {
                    descending = true;
                    rampAltTarget(wpAltCm, dtS);
                }
                navSubState = descending ? 3 : 2;
                return;
            }
        }
        descending = false;
        missionIndex++;
        legValid = false;
        // a fresh leg gets a clean heading-fault window: just after a corner the
        // nose is already on the new leg (the pre-turn blend put it there) but
        // the velocity vector still carries the old leg's momentum, so COG can
        // briefly disagree with heading without any real compass fault. Only a
        // fault that persists on a single leg for the full 2 s should trip.
        hdgFaultTimeS = 0.0f;
        if (missionIndex >= navHudWaypointCount()) {
            // mission complete: park exactly on the final waypoint
            gpsLocation_t finalLoc;
            enuToLocation(wp, &finalLoc);
            posControlSetMissionTarget(&finalLoc, NAV_MISSION_SANITY_CM);
            yawRateDps = 0.0f;
            phase = NAV_MISSION_DONE;
            return;
        }
        // loop: compute the freshly-activated leg in this same cycle
    }
    descending = false;

    // 3D: climb toward the leg's cruise altitude; horizontal march is gated
    // on the bare hard deck - no slack below it - so dips are exited straight
    // up and low engagements climb before translating. Above the deck,
    // altitude changes fly as slanted legs at the rescue climb/descend rates.
    bool altitudeReadyToCruise = true;
    if (altActive) {
        rampAltTarget(legCruiseAltCm(), dtS);
        altitudeReadyToCruise = (float)getEstimatedAltitudeCm() >= missionFloorCm();
    }
    navSubState = altitudeReadyToCruise ? 0 : 5;

    // each leg is a fixed LINE to the waypoint; the carrot rides that line,
    // so wind pushes the craft off the line and the position controller pulls
    // it back on - legs fly arrow-straight, exactly as drawn on the map.
    // A new leg anchors on the LAST COMMANDED CARROT (normally pinned on the
    // waypoint just passed): the drawn wp->wp line, and the position target
    // stays continuous through the corner instead of snapping back to the
    // craft. Engage/retry clear carrotPrevValid to anchor on the craft.
    if (!legValid) {
        legStartCm = carrotPrevValid ? carrotPrevCm : craftCm;
        legProgressCm = 0.0f;
        legValid = true;
    }
    vector2_t legVec = { .x = wp->x - legStartCm.x, .y = wp->y - legStartCm.y };
    const float legLenCm = vector2Norm(&legVec);
    vector2_t legDir = { .x = 1.0f, .y = 0.0f };
    if (legLenCm > 1.0f) {
        legDir.x = legVec.x / legLenCm;
        legDir.y = legVec.y / legLenCm;
    }
    // craft's along-track position on the leg
    const float craftAlongCm = (craftCm.x - legStartCm.x) * legDir.x + (craftCm.y - legStartCm.y) * legDir.y;

    vector2_t carrotCm = {
        .x = legStartCm.x + legDir.x * legProgressCm,
        .y = legStartCm.y + legDir.y * legProgressCm,
    };

    // steering: at the carrot once it is meaningfully ahead, else at the
    // waypoint itself. Approaching a pass gate the target bearing blends
    // toward the NEXT leg through the deceleration zone, so the nose crosses
    // the gate already facing the next waypoint.
    // AHEAD is load-bearing: a craft that overruns the carrot (post-dip
    // catch-up, tailwind) must never be steered at a carrot BEHIND it - that
    // commanded a 180 deg nose flip at speed and tripped the heading-fault
    // detector on the spin it caused (both v29 field failures). On overrun,
    // fall back to the waypoint bearing, which is always forward.
    vector2_t craftToCarrot = { .x = carrotCm.x - craftCm.x, .y = carrotCm.y - craftCm.y };
    const bool carrotUsable = vector2Norm(&craftToCarrot) > 200.0f
        && (craftToCarrot.x * legDir.x + craftToCarrot.y * legDir.y) > 0.0f;
    float targetBearingDeg = RADIANS_TO_DEGREES(carrotUsable
        ? atan2_approx(craftToCarrot.x, craftToCarrot.y)
        : atan2_approx(toWp.x, toWp.y));

    const float gateOffsetCm = passGate ? arriveCm : 0.0f;
    const float remainingCm = fmaxf(legLenCm - craftAlongCm - gateOffsetCm, 0.0f);

    bool preTurn = false;
    if (passGate && missionIndex + 1 < navHudWaypointCount()) {
        // the blend completes 5 m BEFORE the gate: the yaw law needs that
        // last stretch to close the tracking lag, so the nose is actually ON
        // the next leg as the gate is crossed, not still chasing it
        const float t = 1.0f - constrainf((remainingCm - 500.0f) / (NAV_MISSION_PRETURN_CM - 500.0f), 0.0f, 1.0f);
        if (t > 0.0f) {
            preTurn = true;
            const vector2_t *next = navHudWaypointAt(missionIndex + 1);
            vector2_t nextLeg = { .x = next->x - wp->x, .y = next->y - wp->y };
            const float nextBearingDeg = RADIANS_TO_DEGREES(atan2_approx(nextLeg.x, nextLeg.y));
            targetBearingDeg += t * wrapDeg180f(nextBearingDeg - targetBearingDeg);
        }
    }

    const float headingDeg = attitude.values.yaw / 10.0f;
    const float yawErrorDeg = wrapDeg180f(headingDeg - targetBearingDeg);
    yawRateDps = missionYawRateDps(yawErrorDeg);

    // heading-fault detector: nose-on-command cruise where COG disagrees
    // wildly with the compass means the heading estimate is broken - the
    // sideways-flyaway precursor. Catch it before the sanity radius does.
    // A real crab angle past 70 degrees cannot hold the leg line at over
    // 3 m/s ground speed, so wind cannot trip this.
    const float cogVsHeadingDeg = fabsf(wrapDeg180f(headingDeg - gpsSol.groundCourse * 0.1f));
    const bool hdgCheckEligible = altitudeReadyToCruise && !preTurn
        && gpsSol.groundSpeed > NAV_MISSION_HDG_FAULT_MIN_SPEED_CMS
        && fabsf(yawErrorDeg) < NAV_MISSION_HDG_FAULT_ALIGNED_DEG
        // overrun/brake transients are self-inflicted heading-vs-COG splits,
        // not compass faults: while the craft is at or past the carrot the
        // commanded bearing is not the travel direction, so don't integrate
        && craftAlongCm < legProgressCm + 200.0f;
    hdgFaultTimeS = (hdgCheckEligible && cogVsHeadingDeg > NAV_MISSION_HDG_FAULT_ERR_DEG)
        ? hdgFaultTimeS + dtS
        : fmaxf(hdgFaultTimeS - 2.0f * dtS, 0.0f);

    // march gating keys on alignment with the LEG direction, so the pre-turn
    // (which deliberately swings the nose off the current leg) cannot stall
    // forward progress; leg starts and true reversals still rotate in place.
    // Binary on purpose: corner slowing belongs to the trapezoid alone - the
    // old cos carve double-braked once the carrot stopped being craft-pushed
    if (altitudeReadyToCruise) {
        const float legBearingDeg = RADIANS_TO_DEGREES(atan2_approx(legDir.x, legDir.y));
        const float legErrDeg = wrapDeg180f(headingDeg - legBearingDeg);
        const float marchScale = (fabsf(legErrDeg) < 90.0f || preTurn) ? 1.0f : 0.0f;

        // trapezoidal speed profile keyed on the CRAFT's distance to the
        // gate: cruise at gps_rescue_ground_speed, decelerate to cross the
        // gate at its turn-angle-scaled corner speed. marchScale shapes the
        // DESIRED speed before the slew, so speed never accumulates while
        // rotating in place; the slew limit removes the full-cruise leap at
        // leg starts and is deliberately not reset at leg switches, so speed
        // carries smoothly through corners.
        float desiredSpeedCmS = sqrtf(sq(cornerSpeedCmS)
                                      + 2.0f * NAV_MISSION_DECEL_CMSS * remainingCm);
        desiredSpeedCmS = fminf(desiredSpeedCmS, (float)gpsRescueConfig()->groundSpeedCmS);
        // governor on MEASURED speed: the trapezoid plans braking from the
        // carrot's speed, but the craft can carry more (post-dip catch-up,
        // tailwind - 14.5 m/s seen against a 9 m/s carrot). A carrot that
        // keeps marching ahead of an overspeeding craft surrenders all its
        // braking authority; hold it instead until the craft is back on
        // profile, and the slew bleeds carrot speed smoothly, not in a step.
        if ((float)gpsSol.groundSpeed > desiredSpeedCmS + NAV_MISSION_OVERSPEED_CMS) {
            desiredSpeedCmS = 0.0f;
        }
        desiredSpeedCmS *= marchScale;
        carrotSpeedCmS = constrainf(desiredSpeedCmS,
                                    carrotSpeedCmS - NAV_MISSION_ACCEL_CMSS * dtS,
                                    carrotSpeedCmS + NAV_MISSION_ACCEL_CMSS * dtS);
        legProgressCm += carrotSpeedCmS * dtS;
    }
    // pure-pursuit style lead: two seconds of travel at the current carrot
    // speed, so the chase equilibrium slows WITH the trapezoid. The carrot is
    // deliberately allowed to sit BEHIND a craft that has overrun the speed
    // profile - a behind-target is the only thing this position controller
    // brakes for - but only by a bounded gap, so braking is firm, not a slam.
    // Clamping the carrot forward to the craft (v17..v22) turned every corner
    // approach into a bulldozer: the craft pushed the target through the gate
    // at cruise speed and drag was the only deceleration.
    const float alongFloorCm = fmaxf(craftAlongCm, 0.0f);
    // 1.2 s lead (was 2.0 s): the 2 s/25 m gap saturated the position P-term
    // into a 10.5-14.5 m/s chase against a 9 m/s carrot on every long leg;
    // the shorter lead keeps the chase equilibrium near the commanded speed.
    // brakeGap likewise tightened so an overrun bites sooner and shallower.
    const float leadCm = constrainf(carrotSpeedCmS * 1.2f, 600.0f, NAV_MISSION_CARROT_LEAD_CM);
    const float brakeGapCm = 0.25f * carrotSpeedCmS + 150.0f;
    legProgressCm = constrainf(legProgressCm, alongFloorCm - brakeGapCm, alongFloorCm + leadCm);
    legProgressCm = constrainf(legProgressCm, 0.0f, legLenCm);
    carrotCm.x = legStartCm.x + legDir.x * legProgressCm;
    carrotCm.y = legStartCm.y + legDir.y * legProgressCm;
    carrotPrevCm = carrotCm;
    carrotPrevValid = true;

    gpsLocation_t targetLoc;
    enuToLocation(&carrotCm, &targetLoc);
    posControlSetMissionTarget(&targetLoc, NAV_MISSION_SANITY_CM);

    DEBUG_SET(DEBUG_NAV_MISSION, 0, (int16_t)(phase * 10 + navSubState));
    DEBUG_SET(DEBUG_NAV_MISSION, 1, (int16_t)missionIndex);
    DEBUG_SET(DEBUG_NAV_MISSION, 2, (int16_t)constrain(lrintf(distCm / 10.0f), 0, INT16_MAX));   // dm to waypoint
    DEBUG_SET(DEBUG_NAV_MISSION, 3, (int16_t)lrintf(yawErrorDeg));                               // heading error
    DEBUG_SET(DEBUG_NAV_MISSION, 4, (int16_t)lrintf(yawRateDps));
    DEBUG_SET(DEBUG_NAV_MISSION, 5, (int16_t)constrain(lrintf(legProgressCm / 10.0f), 0, INT16_MAX)); // carrot along leg, dm
    DEBUG_SET(DEBUG_NAV_MISSION, 6, (int16_t)constrain(lrintf(craftAlongCm / 10.0f), INT16_MIN, INT16_MAX));
    DEBUG_SET(DEBUG_NAV_MISSION, 7, (int16_t)constrain(lrintf(carrotSpeedCmS), 0, INT16_MAX));
}

bool navMissionIsControlling(void)
{
    return phase == NAV_MISSION_FLYING || phase == NAV_MISSION_DONE;
}

float navMissionGetYawRateDps(void)
{
    return (phase == NAV_MISSION_FLYING) ? yawRateDps : 0.0f;
}

bool navMissionIsDescending(void)
{
    return phase == NAV_MISSION_FLYING && descending;
}

bool navMissionAltitudeActive(void)
{
    return altActive && (phase == NAV_MISSION_FLYING || phase == NAV_MISSION_DONE);
}

float navMissionGetTargetAltitudeCm(void)
{
    return altTargetCm;
}

float navMissionGetTargetClimbRateCmS(void)
{
    return altRateCmS;
}

navMissionPhase_e navMissionGetPhase(void)
{
    return phase;
}

unsigned navMissionActiveIndex(void)
{
    return missionIndex;
}

bool navMissionFailed(void)
{
    return missionFailed;
}

#ifdef UNIT_TEST
float navMissionCarrotSpeedForTest(void)
{
    return carrotSpeedCmS;
}

bool navMissionRetriedForTest(void)
{
    return retryTimeUs != 0;
}

void navMissionResetForTest(void)
{
    phase = NAV_MISSION_IDLE;
    missionIndex = 0;
    switchWas = false;
    legValid = false;
    legProgressCm = 0.0f;
    yawRateDps = 0.0f;
    altActive = false;
    altTargetCm = 0.0f;
    altRateCmS = 0.0f;
    descending = false;
    lastTimeUs = 0;
    carrotSpeedCmS = 0.0f;
    carrotPrevValid = false;
    retryTimeUs = 0;
    retryPending = false;
    missionFailed = false;
    hdgFaultTimeS = 0.0f;
    resumeAnchorValid = false;
    testMissionSwitch = false;
}
#endif

#endif // USE_NAV_MISSION

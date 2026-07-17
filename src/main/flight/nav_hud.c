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

// Navigation HUD core: maintains a decimated breadcrumb trail of flown GPS
// positions and assembles a navigation state snapshot for the OSD renderer.
// All navigation calculations live here so the renderer only rasterises.

#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OSD_NAV_HUD

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"

#ifdef USE_MAG
#include "sensors/compass.h"
#include "sensors/sensors.h"
#endif

#if defined(USE_GPS_RESCUE)
#include "flight/gps_rescue.h"
#if !defined(USE_WING)
#include "flight/autopilot.h"
#endif
#endif

#include "io/gps.h"

#include "pg/nav_hud.h"

#include "nav_hud.h"

#define NAV_HUD_STALE_US            2500000     // no accepted fix for this long -> stale
#define NAV_HUD_UPDATE_INTERVAL_US  50000       // state snapshot refresh cap (20 Hz)
#define NAV_HUD_MAX_SPEED_CMS       10000.0f    // 100 m/s plausibility limit for fix-to-fix movement
#define NAV_HUD_JUMP_MARGIN_CM      3000.0f     // fixed allowance on top of the speed window
#define NAV_HUD_COG_MIN_SPEED_CMS   150         // ground course unusable below this speed
#define NAV_HUD_MIN_ROUTE_CM        1500.0f     // rescue route shorter than this is not drawn
#define NAV_HUD_ETA_MIN_CLOSING_CMS 100.0f      // require 1 m/s closing speed for an ETA
#define NAV_HUD_TREND_BAND_CMS      50.0f
#define NAV_HUD_MAX_ETA_S           5999
#define NAV_HUD_MAX_SPACING_M       5000
#define NAV_HUD_BEARING_MIN_DIST_CM 300
#define NAV_HUD_MAG_CHECK_MIN_CMS   300         // judge heading vs course only above this speed
#define NAV_HUD_MAG_CHECK_BAD_DEG   60
#define NAV_HUD_MAG_CHECK_GOOD_DEG  30
#define NAV_HUD_MAG_CHECK_TICKS     40          // ~2 s at the 20 Hz snapshot rate

static navHudState_t navHudState;

static navHudTrailPoint_t trail[NAV_HUD_TRAIL_CAPACITY];
static unsigned trailCount = 0;
static uint16_t trailSpacingM = 0;

static vector2_t waypoints[NAV_HUD_WAYPOINT_MAX];
static int32_t waypointAltsCm[NAV_HUD_WAYPOINT_MAX];
static int32_t waypointCandidateAltCm = 0;
static unsigned waypointCount = 0;
static bool waypointSwitchWas = false;
static vector2_t waypointCandidateCm;      // captured at switch press, committed on release
static timeUs_t waypointPressUs = 0;
static bool waypointPressPending = false;
static bool waypointUndoDone = false;
static bool waypointHintShown = false;
static timeUs_t waypointHintUntilUs = 0;
static bool waypointHintActive = false;
static timeUs_t waypointDeleteMsgUntilUs = 0;   // brief "WPn DELETED" confirmation window
static uint8_t waypointDeleteMsgNumber = 0;     // which waypoint number was just deleted

#define NAV_HUD_WAYPOINT_UNDO_US 1500000   // hold the switch this long to delete the last waypoint
#define NAV_HUD_WAYPOINT_HINT_US 3500000   // how long the one-time first-drop hint shows
#define NAV_HUD_WAYPOINT_DELMSG_US 2500000 // how long the delete confirmation shows
#define NAV_HUD_HOME_MOVE_CM  2500.0f      // home shift past this = a genuinely new site; drop the route

#ifdef UNIT_TEST
bool testWaypointSwitch = false;
static bool waypointSwitchActive(void) { return testWaypointSwitch; }
#else
static bool waypointSwitchActive(void) { return IS_RC_MODE_ACTIVE(BOXUSER2); }
#endif

static uint16_t headingBadTicks = 0;
static uint16_t headingGoodTicks = 0;
static bool headingSuspectLatch = false;

static bool hasAcceptedFix = false;
static vector2_t acceptedPosCm;
static timeUs_t lastFixUs = 0;
static bool suspectValid = false;
static vector2_t suspectPosCm;
static bool glitchActive = false;

static float closingSpeedCmS = 0.0f;
static float previousDistanceCm = 0.0f;

static bool wasArmed = false;
static bool wasRescueActive = false;
static bool rescueStartValid = false;
static vector2_t rescueStartCm;

static gpsLocation_t homeRef;
static bool homeRefValid = false;

static timeUs_t lastUpdateUs = 0;

int16_t navHudWrapDeg180(int32_t deg)
{
    deg = (deg + 180) % 360;
    if (deg < 0) {
        deg += 360;
    }
    return deg - 180;
}

uint16_t navHudWrapDeg360(int32_t deg)
{
    deg %= 360;
    if (deg < 0) {
        deg += 360;
    }
    return deg;
}

// compass bearing (0 = North, clockwise) from one local ENU point to another
int16_t navHudBearingDeg(const vector2_t *fromCm, const vector2_t *toCm)
{
    const float east = toCm->x - fromCm->x;
    const float north = toCm->y - fromCm->y;
    return navHudWrapDeg360(lrintf(RADIANS_TO_DEGREES(atan2_approx(east, north))));
}

// signed distance from the craft to the line running lineStart -> home (origin);
// positive when the craft is right of the track direction
int32_t navHudCrossTrackCm(const vector2_t *lineStartCm, const vector2_t *craftCm)
{
    const float lineLength = vector2Norm(lineStartCm);
    if (lineLength < 1.0f) {
        return 0;
    }
    // track direction is from the start point towards home (the origin)
    const float dirEast = -lineStartCm->x / lineLength;
    const float dirNorth = -lineStartCm->y / lineLength;
    const float relEast = craftCm->x - lineStartCm->x;
    const float relNorth = craftCm->y - lineStartCm->y;
    // 2D cross product dir x rel; in ENU a negative value is right of track
    return lrintf(-(dirEast * relNorth - dirNorth * relEast));
}

uint32_t navHudSelectAutoScaleM(uint32_t requiredWidthM)
{
    static const uint16_t scaleStepsM[] = { 20, 30, 50, 75, 100, 150, 200, 300, 400, 600, 800, 1200,
                                            1600, 2400, 3200, 4800, 6400, 9600, 12800, 19200, 25600, 38400, 51200 };
    for (unsigned i = 0; i < ARRAYLEN(scaleStepsM); i++) {
        if (requiredWidthM <= scaleStepsM[i]) {
            return scaleStepsM[i];
        }
    }
    return scaleStepsM[ARRAYLEN(scaleStepsM) - 1];
}

void navHudResetTrail(void)
{
    trailCount = 0;
    trailSpacingM = MAX(navHudConfig()->breadcrumbSpacingM, 1);
    hasAcceptedFix = false;
    suspectValid = false;
    glitchActive = false;
    closingSpeedCmS = 0.0f;
    previousDistanceCm = 0.0f;
    rescueStartValid = false;
    headingBadTicks = 0;
    headingGoodTicks = 0;
    headingSuspectLatch = false;
    // NOTE: the dropped waypoints deliberately survive a trail reset. The trail
    // is per-flight (reset on the arm edge), but the mission route persists
    // across disarm/arm and is cleared only on power-up (navHudInit) or a
    // genuine home relocation (checkResetConditions). Only the momentary-switch
    // gesture state is reset here so a press can't carry across an arm.
    waypointSwitchWas = false;
    waypointPressPending = false;
    waypointUndoDone = false;
    waypointHintShown = false;
    waypointHintUntilUs = 0;
    waypointHintActive = false;
    waypointDeleteMsgUntilUs = 0;
}

void navHudInit(void)
{
    memset(&navHudState, 0, sizeof(navHudState));
    navHudResetTrail();
    waypointCount = 0;   // the route lives only in RAM: a power cycle clears it
    homeRefValid = false;
    wasArmed = false;
    wasRescueActive = false;
    lastFixUs = 0;
    lastUpdateUs = 0;
}

unsigned navHudTrailCount(void)
{
    return trailCount;
}

unsigned navHudWaypointCount(void)
{
    return waypointCount;
}

const vector2_t *navHudWaypointAt(unsigned index)
{
    if (index >= waypointCount) {
        return NULL;
    }
    return &waypoints[index];
}

int32_t navHudWaypointAltCm(unsigned index)
{
    if (index >= waypointCount) {
        return 0;
    }
    return waypointAltsCm[index];
}

bool navHudWaypointHintActive(void)
{
    return waypointHintActive;
}

// the number of the waypoint just deleted while the confirmation is showing,
// or 0 when no delete confirmation is active
uint8_t navHudWaypointDeleteMsg(void)
{
    return (waypointDeleteMsgUntilUs != 0) ? waypointDeleteMsgNumber : 0;
}

const navHudTrailPoint_t *navHudTrailPointAt(unsigned index)
{
    if (index >= trailCount) {
        return NULL;
    }
    return &trail[index];
}

static bool navHudRescueFlying(void)
{
#if defined(USE_GPS_RESCUE)
    return FLIGHT_MODE(GPS_RESCUE_MODE);
#else
    return false;
#endif
}

static void checkResetConditions(void)
{
    const bool armed = ARMING_FLAG(ARMED);
    if (armed && !wasArmed) {
        navHudResetTrail();
    }
    wasArmed = armed;

    if (STATE(GPS_FIX_HOME)) {
        if (!homeRefValid) {
            homeRef = GPS_home_llh;
            homeRefValid = true;
            navHudResetTrail();
        } else {
            // A small home shift is just GPS jitter when re-arming on the same
            // spot (home is re-set at each arm unless gps_set_home_point_once):
            // the route must survive that so a mission can be re-flown across a
            // battery swap. Only a genuine relocation past NAV_HUD_HOME_MOVE_CM
            // invalidates the trail frame AND the waypoints (they are stored in
            // ENU relative to home, so at a new site they point to the wrong
            // places).
            vector2_t homeShiftCm;
            GPS_distance2d(&homeRef, &GPS_home_llh, &homeShiftCm);
            if (vector2Norm(&homeShiftCm) > NAV_HUD_HOME_MOVE_CM) {
                homeRef = GPS_home_llh;
                navHudResetTrail();
                waypointCount = 0;
            }
        }
    } else {
        homeRefValid = false;
    }
}

static void updateRescueTracking(void)
{
    const bool active = navHudRescueFlying();
    if (active && !wasRescueActive) {
        if (hasAcceptedFix) {
            rescueStartCm = acceptedPosCm;
            rescueStartValid = true;
        } else {
            rescueStartValid = false;
        }
    }
    wasRescueActive = active;
}

static void pushTrailPoint(int32_t eastM, int32_t northM, uint8_t flags)
{
    unsigned activeCapacity = MIN((unsigned)navHudConfig()->breadcrumbCount, (unsigned)NAV_HUD_TRAIL_CAPACITY);
    activeCapacity = MAX(activeCapacity, 2u);

    // Adaptive decimation: when full, drop every second point and double the
    // spacing so the whole flight stays represented at reduced resolution.
    while (trailCount >= activeCapacity) {
        for (unsigned i = 0; i < trailCount / 2; i++) {
            trail[i] = trail[i * 2 + 1];
        }
        trailCount /= 2;
        if (trailSpacingM < NAV_HUD_MAX_SPACING_M) {
            trailSpacingM = MIN(trailSpacingM * 2, NAV_HUD_MAX_SPACING_M);
        }
    }

    trail[trailCount].eastM = constrain(eastM, INT16_MIN, INT16_MAX);
    trail[trailCount].northM = constrain(northM, INT16_MIN, INT16_MAX);
    trail[trailCount].flags = flags;
    trailCount++;
}

static void recordBreadcrumb(const vector2_t *posCm, bool rescueFlying)
{
    if (!navHudConfig()->breadcrumbs) {
        return;
    }

    const int32_t eastM = lrintf(posCm->x / 100.0f);
    const int32_t northM = lrintf(posCm->y / 100.0f);
    const uint8_t flags = rescueFlying ? NAV_HUD_TRAIL_FLAG_RESCUE : 0;

    if (trailCount == 0) {
        pushTrailPoint(eastM, northM, flags);
        return;
    }

    const navHudTrailPoint_t *last = &trail[trailCount - 1];
    const int64_t dx = eastM - last->eastM;
    const int64_t dy = northM - last->northM;
    const int64_t spacing = trailSpacingM;
    if (dx * dx + dy * dy >= spacing * spacing) {
        pushTrailPoint(eastM, northM, flags);
    }
}

// Feed one GPS fix (local ENU cm relative to home) into the trail/plausibility
// pipeline. Split out from navHudOnGpsNewData so unit tests can drive it.
STATIC_UNIT_TESTED void navHudIngestFix(const vector2_t *posCm, float intervalS, bool rescueFlying, timeUs_t nowUs)
{
    intervalS = constrainf(intervalS, 0.05f, 2.5f);

    bool accepted = false;
    if (!hasAcceptedFix) {
        accepted = true;
    } else {
        const float windowCm = NAV_HUD_MAX_SPEED_CMS * intervalS + NAV_HUD_JUMP_MARGIN_CM;
        vector2_t delta = { .x = posCm->x - acceptedPosCm.x, .y = posCm->y - acceptedPosCm.y };
        if (vector2Norm(&delta) <= windowCm) {
            accepted = true;
        } else if (suspectValid) {
            // a second sample consistent with the first "implausible" one means
            // the position really did change (e.g. reacquisition after GPS loss)
            vector2_t suspectDelta = { .x = posCm->x - suspectPosCm.x, .y = posCm->y - suspectPosCm.y };
            if (vector2Norm(&suspectDelta) <= windowCm) {
                accepted = true;
            }
        }
    }

    if (!accepted) {
        suspectPosCm = *posCm;
        suspectValid = true;
        glitchActive = true;
        return;
    }

    if (hasAcceptedFix) {
        const float newDistanceCm = vector2Norm(posCm);
        const float closing = (previousDistanceCm - newDistanceCm) / intervalS;
        closingSpeedCmS += 0.25f * (closing - closingSpeedCmS);
        previousDistanceCm = newDistanceCm;
    } else {
        previousDistanceCm = vector2Norm(posCm);
        closingSpeedCmS = 0.0f;
    }

    acceptedPosCm = *posCm;
    hasAcceptedFix = true;
    suspectValid = false;
    glitchActive = false;
    lastFixUs = nowUs;

    if (ARMING_FLAG(ARMED)) {
        recordBreadcrumb(posCm, rescueFlying);
    }
}

void navHudOnGpsNewData(void)
{
    if (navHudConfig()->mode == NAV_HUD_MODE_OFF) {
        return;
    }

    checkResetConditions();

    if (!STATE(GPS_FIX) || !STATE(GPS_FIX_HOME)) {
        return;
    }

    updateRescueTracking();

    vector2_t posCm;
    GPS_distance2d(&GPS_home_llh, &gpsSol.llh, &posCm);
    navHudIngestFix(&posCm, gpsSol.navIntervalMs * 0.001f, navHudRescueFlying(), micros());
}

#if defined(USE_GPS_RESCUE) && !defined(USE_WING)
static navHudPhase_e mapRescuePhase(void)
{
#if defined(USE_GPS_RESCUE) && !defined(USE_WING)
    switch (gpsRescueGetPhase()) {
    case RESCUE_INITIALIZE:
        return NAV_HUD_PHASE_INIT;
    case RESCUE_ATTAIN_ALT:
        return NAV_HUD_PHASE_CLIMB;
    case RESCUE_ROTATE:
        return NAV_HUD_PHASE_TURN;
    case RESCUE_FLY_HOME:
        return NAV_HUD_PHASE_RETURN;
    case RESCUE_DESCENT:
        return NAV_HUD_PHASE_DESCEND;
    case RESCUE_LANDING:
        return NAV_HUD_PHASE_LAND;
    case RESCUE_ABORT:
        return NAV_HUD_PHASE_ABORTED;
    case RESCUE_DO_NOTHING:
        return NAV_HUD_PHASE_HOLD;
    case RESCUE_IDLE:
    default:
        return NAV_HUD_PHASE_INIT;
    }
#else
    // wing rescue (and rescue-less builds) expose no phase detail
    return NAV_HUD_PHASE_RETURN;
#endif
}
#endif

void navHudUpdate(timeUs_t currentTimeUs)
{
    if (navHudConfig()->mode == NAV_HUD_MODE_OFF) {
        memset(&navHudState, 0, sizeof(navHudState));
        return;
    }

    if (lastUpdateUs != 0 && cmpTimeUs(currentTimeUs, lastUpdateUs) < NAV_HUD_UPDATE_INTERVAL_US) {
        return;
    }
    lastUpdateUs = currentTimeUs;

    checkResetConditions();
    updateRescueTracking();

    navHudState_t *s = &navHudState;

    s->gpsValid = STATE(GPS_FIX) && gpsIsHealthy();
    s->homeValid = STATE(GPS_FIX_HOME);
    s->stale = !hasAcceptedFix || (lastFixUs != 0 && cmpTimeUs(currentTimeUs, lastFixUs) > NAV_HUD_STALE_US);
    s->positionValid = s->homeValid && hasAcceptedFix && !s->stale;
    s->gpsGlitch = glitchActive;

    s->craftPosCm = hasAcceptedFix ? acceptedPosCm : (vector2_t){ .x = 0.0f, .y = 0.0f };
    s->distanceToHomeCm = hasAcceptedFix ? lrintf(vector2Norm(&acceptedPosCm)) : 0;

    s->headingDeg = navHudWrapDeg360(DECIDEGREES_TO_DEGREES((int32_t)attitude.values.yaw));
#ifdef USE_MAG
    // same gate the IMU uses to fuse the compass: a healthy mag means the
    // heading is trustworthy even at hover, unlike GPS-course-derived heading
    s->headingFromMag = sensors(SENSOR_MAG) && compassIsHealthy();
#else
    s->headingFromMag = false;
#endif
    s->headingValid = s->headingFromMag || canUseGPSHeading;
    s->cogDeg = navHudWrapDeg360(gpsSol.groundCourse / 10);
    s->cogValid = s->gpsValid && gpsSol.groundSpeed >= NAV_HUD_COG_MIN_SPEED_CMS;
    s->groundSpeedCmS = s->gpsValid ? gpsSol.groundSpeed : 0;

    // a heading that fights the GPS course while moving fast means the compass
    // is misaligned or interfered with; latch suspicion so the renderer can
    // rotate by course instead of drawing a convincingly wrong map
    if (s->cogValid && s->groundSpeedCmS >= NAV_HUD_MAG_CHECK_MIN_CMS) {
        const int courseErrDeg = ABS(navHudWrapDeg180(s->headingDeg - s->cogDeg));
        if (courseErrDeg > NAV_HUD_MAG_CHECK_BAD_DEG) {
            headingGoodTicks = 0;
            if (headingBadTicks < UINT16_MAX) {
                headingBadTicks++;
            }
        } else if (courseErrDeg < NAV_HUD_MAG_CHECK_GOOD_DEG) {
            headingBadTicks = 0;
            if (headingGoodTicks < UINT16_MAX) {
                headingGoodTicks++;
            }
        }
        if (headingBadTicks >= NAV_HUD_MAG_CHECK_TICKS) {
            headingSuspectLatch = true;
        } else if (headingGoodTicks >= NAV_HUD_MAG_CHECK_TICKS) {
            headingSuspectLatch = false;
        }
    }
    s->headingSuspect = headingSuspectLatch;

    if (s->positionValid && s->distanceToHomeCm > NAV_HUD_BEARING_MIN_DIST_CM) {
        const vector2_t origin = { .x = 0.0f, .y = 0.0f };
        s->bearingToHomeDeg = navHudBearingDeg(&s->craftPosCm, &origin);
    } else {
        s->bearingToHomeDeg = s->headingDeg;
    }

    // waypoint switch: a TAP drops a waypoint (position captured at the press
    // so speed doesn't smear it, committed on release); HOLDING the switch
    // ~1.5 s deletes the last waypoint instead - quick undo for a mis-drop.
    // The press starts the gesture whenever the craft has a valid fix; whether
    // it becomes a drop or a delete is decided later, so the delete gesture
    // works even with a full route (the "can I add?" room check is only on the
    // drop). A delete flashes a brief on-screen confirmation.
    {
        const bool sw = waypointSwitchActive();
        if (sw && !waypointSwitchWas) {
            if (ARMING_FLAG(ARMED) && s->positionValid) {
                waypointCandidateCm = s->craftPosCm;
                waypointCandidateAltCm = getEstimatedAltitudeCm();
                waypointPressUs = currentTimeUs;
                waypointPressPending = true;
                waypointUndoDone = false;
            }
        } else if (sw && waypointPressPending && !waypointUndoDone
                   && cmpTimeUs(currentTimeUs, waypointPressUs) >= NAV_HUD_WAYPOINT_UNDO_US) {
            // held past the threshold: delete the last waypoint (if any)
            waypointPressPending = false;
            waypointUndoDone = true;
            if (waypointCount > 0) {
                waypointDeleteMsgNumber = waypointCount;   // the number just removed
                waypointCount--;
                waypointDeleteMsgUntilUs = currentTimeUs + NAV_HUD_WAYPOINT_DELMSG_US;
            }
        } else if (!sw && waypointSwitchWas && waypointPressPending) {
            // released before the threshold: commit the drop, if there is room
            waypointPressPending = false;
            if (waypointCount < NAV_HUD_WAYPOINT_MAX) {
                waypointAltsCm[waypointCount] = waypointCandidateAltCm;
                waypoints[waypointCount++] = waypointCandidateCm;
                if (!waypointHintShown) {
                    // one teachable moment on the very first drop, then silence
                    waypointHintShown = true;
                    waypointHintUntilUs = currentTimeUs + NAV_HUD_WAYPOINT_HINT_US;
                }
            }
        }
        waypointSwitchWas = sw;
        waypointHintActive = (waypointHintUntilUs != 0)
                             && (cmpTimeUs(waypointHintUntilUs, currentTimeUs) > 0);
        if (waypointDeleteMsgUntilUs != 0 && cmpTimeUs(waypointDeleteMsgUntilUs, currentTimeUs) <= 0) {
            waypointDeleteMsgUntilUs = 0;   // confirmation window elapsed
        }
    }

    s->altitudeCm = getEstimatedAltitudeCm();  // same source as the standard OSD altitude element
    s->verticalVelCmS = constrain(lrintf(getAltitudeDerivative()), INT16_MIN, INT16_MAX);

    s->satellites = gpsSol.numSat;
    s->hdop = gpsSol.dop.hdop;

    // hold modes (often bound to a single switch; reported separately)
    s->posHoldActive = FLIGHT_MODE(POS_HOLD_MODE);
    s->altHoldActive = FLIGHT_MODE(ALT_HOLD_MODE);

    // rescue status
    s->rescueActive = navHudRescueFlying();
#if defined(USE_GPS_RESCUE) && !defined(USE_WING)
    s->rescueFailure = (uint8_t)gpsRescueGetFailure();
#else
    s->rescueFailure = 0;
#endif
#if defined(USE_GPS_RESCUE) && !defined(USE_WING)
    s->rescueHealthy = (s->rescueFailure == (uint8_t)RESCUE_HEALTHY);
    if (s->rescueActive) {
        s->phase = mapRescuePhase();
    } else if (gpsRescueIsConfigured() && !gpsRescueIsAvailable()) {
        s->phase = NAV_HUD_PHASE_UNAVAILABLE;
    } else {
        s->phase = NAV_HUD_PHASE_IDLE;
    }
#else
    s->rescueHealthy = true;
    s->phase = s->rescueActive ? NAV_HUD_PHASE_RETURN : NAV_HUD_PHASE_IDLE;
#endif

#if defined(USE_GPS_RESCUE) && !defined(USE_WING)
    if (s->rescueActive) {
        s->targetAltitudeCm = lrintf(gpsRescueGetTargetAltitudeCm());
        s->targetSpeedCmS = constrain(lrintf(gpsRescueGetTargetVelocityCmS()), 0, UINT16_MAX);
        s->descentDistanceCm = lrintf(gpsRescueGetDescentDistanceCm());
    } else {
        s->targetAltitudeCm = 0;
        s->targetSpeedCmS = 0;
        s->descentDistanceCm = 0;
    }
    s->targetPosValid = autopilotGetTargetPositionEfCm(&s->targetPosCm);
#else
    s->targetAltitudeCm = 0;
    s->targetSpeedCmS = 0;
    s->descentDistanceCm = 0;
    s->targetPosValid = false;
#endif

    s->rescueStartCm = rescueStartCm;
    s->routeValid = s->rescueActive && rescueStartValid && vector2Norm(&rescueStartCm) > NAV_HUD_MIN_ROUTE_CM;

    s->desiredHeadingDeg = s->bearingToHomeDeg;
    s->headingErrorDeg = navHudWrapDeg180(s->desiredHeadingDeg - s->headingDeg);

    if (s->routeValid && (s->phase == NAV_HUD_PHASE_RETURN || s->phase == NAV_HUD_PHASE_DESCEND || s->phase == NAV_HUD_PHASE_LAND)) {
        s->crossTrackCm = navHudCrossTrackCm(&rescueStartCm, &s->craftPosCm);
    } else {
        s->crossTrackCm = 0;
    }

    // progress towards home
    if (s->positionValid && closingSpeedCmS > NAV_HUD_ETA_MIN_CLOSING_CMS) {
        s->etaSeconds = MIN((uint32_t)(s->distanceToHomeCm / closingSpeedCmS), (uint32_t)NAV_HUD_MAX_ETA_S);
        s->etaValid = true;
    } else {
        s->etaSeconds = 0;
        s->etaValid = false;
    }
    if (closingSpeedCmS > NAV_HUD_TREND_BAND_CMS) {
        s->trackTrend = 1;
    } else if (closingSpeedCmS < -NAV_HUD_TREND_BAND_CMS) {
        s->trackTrend = -1;
    } else {
        s->trackTrend = 0;
    }
}

const navHudState_t *navHudGetState(void)
{
    return &navHudState;
}

#endif // USE_OSD_NAV_HUD

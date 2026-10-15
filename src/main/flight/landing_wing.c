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

#ifdef USE_WING

#include "build/debug.h"

#include "common/maths.h"
#include "common/vector.h"

#include "drivers/rangefinder/rangefinder.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/flight_plan_nav_wing.h"
#include "flight/imu.h"
#include "flight/landing_wing.h"
#include "flight/launch_wing.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "flight/position_nav.h"

#include "pg/autopilot.h"
#include "pg/autopilot_wing.h"
#include "pg/gps_rescue.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#define LANDING_WING_STILL_SINK_CMS           30.0f
#define LANDING_WING_STILL_SPEED_CMS          300.0f
#define LANDING_WING_STILL_RATE_DPS           15.0f
#define LANDING_WING_STILL_RATE_TAU_S         0.2f
#define LANDING_WING_STILL_ACC_G              0.15f
#define LANDING_WING_HARD_CONTACT_G           2.0f
#define LANDING_WING_HARD_CONTACT_STILL_US    1000000
#define LANDING_WING_GROUND_MARGIN_M          2.0f
#define LANDING_WING_FLAT_DECIDEG             75      // well under the gentlest bank a loiter flies without a position
#define LANDING_WING_FLAT_STILL_US            3000000
#define LANDING_WING_LEVEL_FLARES             3.0f
#define LANDING_WING_FLARE_PITCH_DPS          4.0f
#define LANDING_WING_FLARE_PITCH_GAIN         20.0f   // deg/s of nose-up for each m/s it sinks faster than the flare's sink
#define LANDING_WING_FLARE_GUARD_US           3000000
#define LANDING_WING_FLARE_BANK_DEG           3.0f    // enough to hold final's line as a crosswind takes a bigger crab the slower it floats
#define LANDING_WING_UNKNOWN_GROUND_BANK_DEG  15.0f   // circling down, a wing tip clears ground met unawares
#define LANDING_WING_UNKNOWN_GROUND_LOW_BANK_DEG 10.0f   // and near home's ground, wide enough to touch down on

#define LANDING_WING_DEPARTURE_SPEED_CMS      500.0f
#define LANDING_WING_DEPARTURE_CEILING_CM     3000.0f
#define LANDING_WING_DEPARTURE_STRAIGHT_DEG   10.0f
#define LANDING_WING_DEPARTURE_TIME_US        3000000

#define LANDING_WING_COURSE_MIN_SPEED_CMS     300.0f
#define LANDING_WING_APPROACH_ALT_WINDOW_M    3.0f
#define LANDING_WING_ENTRY_BEARING_DEG        60.0f
#define LANDING_WING_FINAL_ESTABLISHED_DEG    30.0f
#define LANDING_WING_FINAL_SINK_MARGIN        1.5f
#define LANDING_WING_FLOAT_MAX_FINAL          0.5f    // of the final's length
#define LANDING_WING_PATTERN_WIDTH_FINAL      0.5f
#define LANDING_WING_PATTERN_WIDTH_TURNS      2.5f    // tightest turns across, so base is a leg of its own
#define LANDING_WING_PATTERN_BEYOND_FINAL     (1.0f / 3.0f)
#define LANDING_WING_LOW_FLARES               2.0f
#define LANDING_WING_LINE_AHEAD_M             2000.0f
#define LANDING_WING_MIN_LEG_M                0.1f
#define LANDING_WING_SLOPE_CHECK_M            30.0f
#define LANDING_WING_OVERSHOOT_MIN_M          30.0f
#define LANDING_WING_OVERSHOOT_TIME_S         2.0f
#define LANDING_WING_CROSS_TRACK_MAX_M        15.0f
#define LANDING_WING_CROSS_TRACK_HEIGHT_M     15.0f
#define LANDING_WING_PULL_UP_US               3000000
#define LANDING_WING_PULL_UP_CLEARANCE_M      5.0f
#define LANDING_WING_GO_AROUND_ALT_WINDOW_M   5.0f
#define LANDING_WING_FLARE_GIVE_UP_US         30000000
#define LANDING_WING_FLARE_GIVE_UP_SPEED_CMS  500.0f
#define LANDING_WING_STICK_GO_AROUND          0.3f
#define LANDING_WING_DESCENT_DEPTH_M          1000.0f // deeper than any ground it could be coming down onto
#define LANDING_WING_BRAKING_G                0.15f   // slowing harder than flight slows it: the ground is braking it
#define LANDING_WING_BRAKING_US               200000
#define LANDING_WING_FLYING_US                2000000
#define LANDING_WING_GROUND_ACC_G             0.3f    // shaken this hard, it is on the ground
#define LANDING_WING_FLARE_CONFIRM_US         200000
#define LANDING_WING_RECKON_TAU_S             30.0f   // far longer than the estimator takes to follow a step in the altitude

static float courseDeg(const positionEstimate3d_t *est)
{
    return RADIANS_TO_DEGREES(atan2_approx(est->velocity.v[ENU_E], est->velocity.v[ENU_N]));
}

static float flareHeightM(void)
{
    return autopilotWingConfig()->landFlareHeight * 0.01f;
}

float landingWingHomeGroundM(void)
{
    return launchWingThrown() ? -autopilotWingConfig()->landLaunchHeight * 0.01f : 0.0f;
}

float landingWingHomeGroundRadiusM(void)
{
#ifdef USE_GPS_RESCUE
    return gpsRescueConfig()->minStartDistM;
#else
    return 0.0f;
#endif
}

// Coming down to land, the wings are held level below this height.
static float levelHeightM(void)
{
    return LANDING_WING_LEVEL_FLARES * flareHeightM();
}

static bool rangefinderHeightM(float *heightM)
{
#ifdef USE_RANGEFINDER
    if (rangefinderIsHealthy()) {
        const int32_t rangeCm = rangefinderGetLatestAltitude();
        if (rangeCm > RANGEFINDER_OUT_OF_RANGE) {
            *heightM = rangeCm * 0.01f;
            return true;
        }
    }
#else
    UNUSED(heightM);
#endif
    return false;
}

float landingWingHeightM(float groundM)
{
    float heightM;
    if (rangefinderHeightM(&heightM)) {
        return heightM;
    }
    return getAltitudeCmControl() * 0.01f - groundM;
}

// Whether the height over home's ground is the height over the ground under the aircraft.
static bool groundUnderKnown(void)
{
    float heightM;
    if (rangefinderHeightM(&heightM)) {
        return true;
    }
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const vector2_t fromHomeM = positionEstimateHorizontalM(est);
    return est->isValidXY && vector2Norm(&fromHomeM) <= landingWingHomeGroundRadiusM();
}

void landingWingTouchdownReset(landingWingTouchdown_t *touchdown)
{
    touchdown->contact = false;
    touchdown->braking = false;
    touchdown->still = false;
    touchdown->hardContact = false;
    touchdown->rateUs = 0;
    touchdown->low = false;
    touchdown->flareSpent = false;
    touchdown->flare.active = false;
}

// Slowing along the nose, g.
static float brakingG(void)
{
    return rMat.m[NWU_U][X] - acc.accADC.v[X] * acc.dev.acc_1G_rec;
}

// An impact, or else over ground of known height the sink stopping low down, and over ground of
// unknown height the ground braking it.
STATIC_UNIT_TESTED bool landingWingContact(landingWingTouchdown_t *touchdown, timeUs_t nowUs, float heightM, bool heightKnown, float sinkDemandCmS)
{
    if (brakingG() <= LANDING_WING_BRAKING_G) {
        touchdown->braking = false;
    } else if (!touchdown->braking) {
        touchdown->braking = true;
        touchdown->brakingSinceUs = nowUs;
    }
    if (heightKnown && heightM >= levelHeightM()) {
        return false;
    }
    if (acc.accMagnitude > LANDING_WING_HARD_CONTACT_G) {
        return true;
    }
    if (heightKnown) {
        return sinkDemandCmS > 0.0f && fabsf(positionEstimatorGetEstimate()->velocity.v[ENU_U]) < LANDING_WING_STILL_SINK_CMS;
    }
    return touchdown->braking && cmpTimeUs(nowUs, touchdown->brakingSinceUs) >= LANDING_WING_BRAKING_US;
}

// Latched, so that the motor stays stopped through a bounce and the ground run. Still sinking as only
// flight does, neither shaken nor braked as the ground would, it was a wrong guess in the air.
static void cutOnContact(landingWingTouchdown_t *touchdown, timeUs_t nowUs, float heightM, bool heightKnown, float sinkDemandCmS,
                         autopilotWingLimits_t *limits)
{
    if (!touchdown->contact) {
        touchdown->contact = landingWingContact(touchdown, nowUs, heightM, heightKnown, sinkDemandCmS);
        touchdown->flyingSinceUs = nowUs;
    } else if (positionEstimatorGetEstimate()->velocity.v[ENU_U] > -LANDING_WING_FLYING_SINK_CMS
               || fabsf(acc.accMagnitude - 1.0f) > LANDING_WING_GROUND_ACC_G || brakingG() > LANDING_WING_BRAKING_G) {
        touchdown->flyingSinceUs = nowUs;
    } else if (cmpTimeUs(nowUs, touchdown->flyingSinceUs) > LANDING_WING_FLYING_US) {
        touchdown->contact = false;
    }
    if (touchdown->contact) {
        limits->throttleMin = 0.0f;
        limits->throttleMax = 0.0f;
        limits->motorStop = true;
    }
}

// The body rates smoothed: an idling motor shakes the gyro, but it does not turn the aircraft.
static bool rotating(landingWingTouchdown_t *touchdown, timeUs_t nowUs)
{
    const float k = (touchdown->rateUs == 0) ? 1.0f
        : constrainf(cmpTimeUs(nowUs, touchdown->rateUs) * 1e-6f / LANDING_WING_STILL_RATE_TAU_S, 0.0f, 1.0f);
    touchdown->rateUs = nowUs;
    bool turning = false;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        touchdown->rateDps[axis] += k * (gyro.gyroADCf[axis] - touchdown->rateDps[axis]);
        turning = turning || fabsf(touchdown->rateDps[axis]) >= LANDING_WING_STILL_RATE_DPS;
    }
    return turning;
}

// Wings flat on the ground, either way up: never so in the air for long, where a loiter banks.
static bool wingsFlat(void)
{
    const int32_t rollDeci = ABS(attitude.values.roll);
    return rollDeci < LANDING_WING_FLAT_DECIDEG || rollDeci > 1800 - LANDING_WING_FLAT_DECIDEG;
}

bool landingWingTouchdownUpdate(landingWingTouchdown_t *touchdown, timeUs_t nowUs, float heightM, float sinkDemandCmS)
{
    const bool turning = rotating(touchdown, nowUs);
    const bool low = heightM < flareHeightM() + LANDING_WING_GROUND_MARGIN_M;
    if (!low) {
        touchdown->hardContact = false;
    } else if (acc.accMagnitude > LANDING_WING_HARD_CONTACT_G) {
        touchdown->hardContact = true;
    }

    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const bool still = sinkDemandCmS > 0.0f
        && est->velocity.v[ENU_U] > -LANDING_WING_STILL_SINK_CMS
        && (!est->isValidXY || positionEstimateGroundspeedCmS(est) < LANDING_WING_STILL_SPEED_CMS)
        && fabsf(acc.accMagnitude - 1.0f) < LANDING_WING_STILL_ACC_G
        && !turning
        && (low || wingsFlat());
    if (!still) {
        touchdown->still = false;
        return false;
    }
    if (!touchdown->still) {
        touchdown->still = true;
        touchdown->stillSinceUs = nowUs;
    }
    timeDelta_t requiredUs = autopilotConfig()->landingDetectionTime * 100000;
    if (!low) {
        requiredUs = LANDING_WING_FLAT_STILL_US;
    } else if (touchdown->hardContact) {
        requiredUs = LANDING_WING_HARD_CONTACT_STILL_US;
    }
    return cmpTimeUs(nowUs, touchdown->stillSinceUs) >= requiredUs;
}

static float glideSlopeRad(void)
{
    return DEGREES_TO_RADIANS(autopilotWingConfig()->landGlideAngle);
}

static float flareSinkCmS(void)
{
    return autopilotWingConfig()->landFlareSink * 10.0f;
}

// The sink the glide slope comes down at the cruise speed, which the flare eases off from its height.
static float cruiseGlideSinkCmS(void)
{
    return fmaxf(autopilotWingConfig()->cruiseSpeed * 10.0f * tan_approx(glideSlopeRad()), flareSinkCmS());
}

// From the flare height to the ground: the sink eases off with the height to the flare's, then
// holds it for the rest of the way.
static float flareTimeS(void)
{
    const float easeS = flareHeightM() * 100.0f / cruiseGlideSinkCmS();
    return easeS * (logf(cruiseGlideSinkCmS() / flareSinkCmS()) + 1.0f);
}

// The attitude it glides at: as it is, but no higher than it glides down its path at without power.
static void flareStart(landingWingFlare_t *flare, timeUs_t nowUs)
{
    flare->active = true;
    flare->unknownGround = false;
    flare->powered = false;
    flare->startUs = nowUs;
    flare->lastUs = nowUs;
    flare->entrySinkCmS = fmaxf(-positionEstimatorGetEstimate()->velocity.v[ENU_U], flareSinkCmS());
    flare->glidePitchDeg = fminf(autopilotWingAttitudePitchDeg(), autopilotWingIdlePitchDeg(-flare->entrySinkCmS));
    flare->pitchDeg = flare->glidePitchDeg;
}

static bool flareGivenUp(const landingWingFlare_t *flare, timeUs_t nowUs)
{
    return !flare->powered
        && (flare->unknownGround || cmpTimeUs(nowUs, flare->startUs) > flareTimeS() * 1e6f + LANDING_WING_FLARE_GUARD_US);
}

// Sets the flare's limits; returns the sink it demands.
static float flareLimits(landingWingFlare_t *flare, timeUs_t nowUs, float heightM, autopilotWingLimits_t *limits)
{
    const float dtS = cmpTimeUs(nowUs, flare->lastUs) * 1e-6f;
    flare->lastUs = nowUs;

    limits->climbRateOverride = true;
    limits->throttleMin = 0.0f;
    if (flare->powered) {
        limits->throttleForPath = true;
    } else {
        limits->throttleMax = 0.0f;
        limits->motorStop = true;
    }

    float sinkCmS;
    if (flareGivenUp(flare, nowUs)) {
        flare->pitchDeg = fmaxf(flare->pitchDeg - LANDING_WING_FLARE_PITCH_DPS * dtS, flare->glidePitchDeg);
        limits->pitchMaxDeg = flare->pitchDeg;
        sinkCmS = fmaxf(flare->entrySinkCmS, LANDING_WING_DESCENT_MIN_SINK_CMS);
    } else {
        const float ceilingDeg = fmaxf(autopilotWingConfig()->landFlarePitch, flare->glidePitchDeg);
        sinkCmS = fmaxf(flareSinkCmS(), fminf(flare->entrySinkCmS, heightM / flareHeightM() * cruiseGlideSinkCmS()));
        limits->pitchMaxDeg = ceilingDeg;
        if (!flare->powered) {
            // without power the nose has to come up as the speed goes to hold the sink
            const float climbCmS = positionEstimatorGetEstimate()->velocity.v[ENU_U];
            const float pitchRateDps = constrainf(LANDING_WING_FLARE_PITCH_GAIN * (-climbCmS - sinkCmS) * 0.01f,
                                                  -LANDING_WING_FLARE_PITCH_DPS, LANDING_WING_FLARE_PITCH_DPS);
            flare->pitchDeg = constrainf(flare->pitchDeg + pitchRateDps * dtS, flare->glidePitchDeg, ceilingDeg);
            limits->pitchMinDeg = flare->pitchDeg;
        }
    }
    limits->climbRateCmS = -sinkCmS;
    return sinkCmS;
}

// Below the flare height for long enough that it is not a spike in the height.
static bool lowEnoughToFlare(landingWingTouchdown_t *touchdown, timeUs_t nowUs, float heightM)
{
    if (heightM > flareHeightM()) {
        touchdown->low = false;
        return false;
    }
    if (!touchdown->low) {
        touchdown->low = true;
        touchdown->lowSinceUs = nowUs;
    }
    return cmpTimeUs(nowUs, touchdown->lowSinceUs) >= LANDING_WING_FLARE_CONFIRM_US;
}

bool landingWingDescend(autopilotWingLimitsOwner_e owner, landingWingTouchdown_t *touchdown, timeUs_t nowUs, float sinkDemandCmS)
{
    if (touchdown->flare.active && !touchdown->contact && flareGivenUp(&touchdown->flare, nowUs)) {
        touchdown->flareSpent = true;
    }
    const float heightM = landingWingHeightM(landingWingHomeGroundM());
    const bool heightKnown = groundUnderKnown() && !touchdown->flareSpent;
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    if (!touchdown->flare.active && lowEnoughToFlare(touchdown, nowUs, heightM)) {
        flareStart(&touchdown->flare, nowUs);
    }
    touchdown->flare.unknownGround = !heightKnown;
    touchdown->flare.powered = !heightKnown && !touchdown->contact;
    if (heightKnown) {
        limits.wingsLevel = touchdown->flare.active || heightM < levelHeightM();
    } else {
        limits.bankLimitDeg = heightM < levelHeightM() ? LANDING_WING_UNKNOWN_GROUND_LOW_BANK_DEG : LANDING_WING_UNKNOWN_GROUND_BANK_DEG;
    }
    if (touchdown->flare.active) {
        sinkDemandCmS = flareLimits(&touchdown->flare, nowUs, heightM, &limits);
    } else {
        limits.throttleForPath = true;
        limits.throttleMin = 0.0f;
    }
    cutOnContact(touchdown, nowUs, heightM, heightKnown, sinkDemandCmS, &limits);
    autopilotWingSetLimits(owner, &limits);

    return landingWingTouchdownUpdate(touchdown, nowUs, heightM, sinkDemandCmS);
}

static struct {
    bool known;
    float courseDeg;
    bool straight;
    float straightCourseDeg;
    timeUs_t straightSinceUs;
} departure;

void landingWingNoteDepartureCourse(timeUs_t nowUs)
{
    if (!ARMING_FLAG(ARMED)) {
        departure.known = false;
        departure.straight = false;
        return;
    }
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    if (departure.known) {
        return;
    }
    if (!est->isValidXY || positionEstimateGroundspeedCmS(est) < LANDING_WING_DEPARTURE_SPEED_CMS
        || est->position.v[ENU_U] > LANDING_WING_DEPARTURE_CEILING_CM) {
        departure.straight = false;
        return;
    }
    const float nowCourseDeg = courseDeg(est);
    if (!departure.straight || fabsf(wrapDeg180f(nowCourseDeg - departure.straightCourseDeg)) > LANDING_WING_DEPARTURE_STRAIGHT_DEG) {
        departure.straight = true;
        departure.straightCourseDeg = nowCourseDeg;
        departure.straightSinceUs = nowUs;
    } else if (cmpTimeUs(nowUs, departure.straightSinceUs) >= LANDING_WING_DEPARTURE_TIME_US) {
        departure.known = true;
        departure.courseDeg = departure.straightCourseDeg;
    }
}

#if ENABLE_FLIGHT_PLAN

static struct {
    landingWingPhase_e phase;
    timeUs_t phaseUs;
    landingWingSite_t site;
    bool hasHeading;
    float headingDeg;               // before any turn about for the wind
    bool hasPattern;
    landingWingPattern_t pattern;
    float finalCourseDeg;
    vector2_t alongFinal;           // unit vector down final
    vector2_t legStartEnuM;
    vector2_t legEndEnuM;
    float turnDistM;                // where the leg being flown hands over to the next
    uint8_t attempts;
    landingWingGoAround_e goAround;
    bool pullUp;                    // gone around from low on the approach: climbing out flat out first
    bool finalEstablished;
    float glideSinkCmS;
    float floatM;                   // reckoned level ahead of the slope and held down it: speeding up down the slope must not move its aim
    float reckonedAltitudeM;        // on its own climb rate from final's start, a step in the altitude coming in far faster than it leaks back
    timeUs_t reckonedUs;
    // a lap of the loiter down, and the wind it shows
    bool hasCourse;
    float lastCourseDeg;
    float lapDeg;
    float slowCmS;
    float fastCmS;
    float fastCourseDeg;
    landingWingTouchdown_t touchdown;
} landing;

static float altitudeM(void)
{
    return getAltitudeCmControl() * 0.01f;
}

static float approachAltM(void)
{
    return landing.site.touchdownEnuM.v[ENU_U] + autopilotWingConfig()->landApproachAlt;
}

static float heightM(void)
{
    return landingWingHeightM(landing.site.touchdownEnuM.v[ENU_U]);
}

// Below this the landing carries on down whatever becomes of the position, and may be on the ground.
static float lowHeightM(void)
{
    return LANDING_WING_LOW_FLARES * flareHeightM();
}

static bool committed(void)
{
    return landing.attempts >= autopilotWingConfig()->landAttempts;
}

// The along-track distance left to b on the line from a, and the cross-track error, right positive.
static float toGoM(const vector2_t *a, const vector2_t *b, const vector2_t *craftM, float *crossTrackM)
{
    vector2_t track;
    vector2Sub(&track, b, a);
    const float lengthM = fmaxf(vector2Norm(&track), LANDING_WING_MIN_LEG_M);
    vector2Scale(&track, &track, 1.0f / lengthM);
    vector2_t fromA;
    vector2Sub(&fromA, craftM, a);
    if (crossTrackM) {
        *crossTrackM = vector2Cross(&fromA, &track);
    }
    return lengthM - vector2Dot(&fromA, &track);
}

static float turnDeg(const vector2_t *a, const vector2_t *b, const vector2_t *c)
{
    vector2_t in, out;
    vector2Sub(&in, b, a);
    vector2Sub(&out, c, b);
    return RADIANS_TO_DEGREES(vector2Angle(&in, &out));
}

// How much further the flare carries the aircraft at a groundspeed than the glide slope would have
// from the flare height: the slope aims that far short of the touchdown.
static float flareFloatM(float speedCmS)
{
    const float floatM = speedCmS * 0.01f * flareTimeS() - flareHeightM() / tan_approx(glideSlopeRad());
    return constrainf(floatM, 0.0f, LANDING_WING_FLOAT_MAX_FINAL * autopilotWingConfig()->landFinalLength);
}

STATIC_UNIT_TESTED void landingWingPattern(const landingWingSite_t *site, float headingDeg, int8_t side, landingWingPattern_t *out)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    const float headingRad = DEGREES_TO_RADIANS(headingDeg);
    const vector2_t along = {{ sin_approx(headingRad), cos_approx(headingRad) }};
    const vector2_t outside = {{ side * along.y, -side * along.x }};
    const float finalM = cfg->landFinalLength;
    const float widthM = fmaxf(LANDING_WING_PATTERN_WIDTH_FINAL * finalM, LANDING_WING_PATTERN_WIDTH_TURNS * autopilotWingMinTurnRadiusM());
    const float beyondM = fmaxf(autopilotWingLoiterRadiusM(), LANDING_WING_PATTERN_BEYOND_FINAL * finalM);
    // final is turned onto far enough out to be settled on before the glide slope starts
    const float finalStartM = finalM + autopilotWingLineSettleDistanceM();

    out->touchdownEnuM = (vector2_t){{ site->touchdownEnuM.v[ENU_E], site->touchdownEnuM.v[ENU_N] }};
    out->finalEnuM.x = out->touchdownEnuM.x - finalStartM * along.x;
    out->finalEnuM.y = out->touchdownEnuM.y - finalStartM * along.y;
    out->baseEnuM.x = out->finalEnuM.x + widthM * outside.x;
    out->baseEnuM.y = out->finalEnuM.y + widthM * outside.y;
    out->entryEnuM.x = out->touchdownEnuM.x + widthM * outside.x + beyondM * along.x;
    out->entryEnuM.y = out->touchdownEnuM.y + widthM * outside.y + beyondM * along.y;
    out->finalHeightM = fminf((finalM - flareFloatM(cfg->cruiseSpeed * 10.0f)) * tan_approx(glideSlopeRad()), cfg->landApproachAlt);
}

STATIC_UNIT_TESTED float landingWingGlideAltM(float toTouchdownM, float finalHeightM, float slopeRad)
{
    return fminf(finalHeightM, fmaxf(toTouchdownM, 0.0f) * tan_approx(slopeRad));
}

static void setPhase(landingWingPhase_e phase, timeUs_t nowUs)
{
    landing.phase = phase;
    landing.phaseUs = nowUs;
}

static void flyLegFrom(landingWingPhase_e phase, const vector2_t *startEnuM, const vector2_t *endEnuM, float altM, float vertRateMps,
                       float startAltM, float turnDistM, timeUs_t nowUs)
{
    landing.legStartEnuM = *startEnuM;
    landing.legEndEnuM = *endEnuM;
    landing.turnDistM = turnDistM;
    flightPlanWingFlyLine(startEnuM, endEnuM, altM, vertRateMps, startAltM);
    setPhase(phase, nowUs);
}

static void flyLeg(landingWingPhase_e phase, const vector2_t *startEnuM, const vector2_t *endEnuM, float altM, float vertRateMps,
                   float turnDistM, timeUs_t nowUs)
{
    flyLegFrom(phase, startEnuM, endEnuM, altM, vertRateMps, flightPlanWingCommandedAltitudeM(), turnDistM, nowUs);
}

static vector2_t touchdownM(void)
{
    return (vector2_t){{ landing.site.touchdownEnuM.v[ENU_E], landing.site.touchdownEnuM.v[ENU_N] }};
}

static void loiterDown(timeUs_t nowUs)
{
    autopilotWingClearLimits(AP_WING_LIMITS_LANDING);
    const vector2_t centreEnuM = touchdownM();
    flightPlanWingLoiterAt(&centreEnuM, approachAltM(), landing.site.sinkRateMps, flightPlanWingCommandedAltitudeM());
    landing.hasPattern = false;
    landing.hasCourse = false;
    landing.lapDeg = 0.0f;
    landing.slowCmS = FLT_MAX;
    landing.fastCmS = 0.0f;
    landingWingTouchdownReset(&landing.touchdown);
    setPhase(LANDING_WING_LOITER_DOWN, nowUs);
}

static float descentSinkMps(void)
{
    return fmaxf(landing.site.sinkRateMps, LANDING_WING_DESCENT_MIN_SINK_CMS * 0.01f);
}

static void descend(timeUs_t nowUs)
{
    const vector2_t centreEnuM = touchdownM();
    const float startAltM = flightPlanWingCommandedAltitudeM();
    flightPlanWingLoiterAt(&centreEnuM, startAltM - LANDING_WING_DESCENT_DEPTH_M, descentSinkMps(), startAltM);
    landingWingTouchdownReset(&landing.touchdown);
    setPhase(LANDING_WING_DESCEND, nowUs);
}

void landingWingStart(const landingWingSite_t *site, timeUs_t nowUs)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    landing.site = *site;
    landing.attempts = 0;
    landing.goAround = LANDING_WING_GO_AROUND_NONE;
    landing.hasPattern = false;
    if (!site->groundKnown) {
        descend(nowUs);
        return;
    }
    landing.hasHeading = true;
    if (cfg->landHeading >= 0) {
        landing.headingDeg = cfg->landHeading;
    } else if (site->headingDeg >= 0.0f) {
        landing.headingDeg = site->headingDeg;
    } else if (!launchWingGetCourseDeg(&landing.headingDeg)) {
        landing.hasHeading = departure.known;
        landing.headingDeg = departure.courseDeg;
    }
    loiterDown(nowUs);
}

void landingWingStop(void)
{
    if (landing.phase != LANDING_WING_IDLE) {
        autopilotWingClearLimits(AP_WING_LIMITS_LANDING);
    }
    landing.phase = LANDING_WING_IDLE;
}

landingWingPhase_e landingWingGetPhase(void)
{
    return landing.phase;
}

#ifdef UNIT_TEST
uint8_t landingWingGetAttempts(void)
{
    return landing.attempts;
}

landingWingGoAround_e landingWingGetGoAround(void)
{
    return landing.goAround;
}

float landingWingGetFinalCourseDeg(void)
{
    return landing.hasPattern ? landing.finalCourseDeg : -1.0f;
}
#endif

// The heading to land on, turned about when the wind measured round the loiter would have the
// aircraft land with more tailwind than it may.
static float windwardHeadingDeg(float headingDeg)
{
    const float maxTailwindCmS = autopilotWingConfig()->landMaxTailwind * 10.0f;
    const float windCmS = 0.5f * (landing.fastCmS - landing.slowCmS);
    const float tailwindCmS = windCmS * cos_approx(DEGREES_TO_RADIANS(landing.fastCourseDeg - headingDeg));
    if (maxTailwindCmS > 0.0f && tailwindCmS > maxTailwindCmS) {
        return headingDeg + 180.0f;
    }
    return headingDeg;
}

static void updateLoiterDown(const positionEstimate3d_t *est, timeUs_t nowUs)
{
    const float speedCmS = positionEstimateGroundspeedCmS(est);
    if (est->isValidXY && speedCmS > LANDING_WING_COURSE_MIN_SPEED_CMS) {
        const float nowCourseDeg = courseDeg(est);
        if (landing.hasCourse) {
            landing.lapDeg += fabsf(wrapDeg180f(nowCourseDeg - landing.lastCourseDeg));
        }
        landing.hasCourse = true;
        landing.lastCourseDeg = nowCourseDeg;
        landing.slowCmS = fminf(landing.slowCmS, speedCmS);
        if (speedCmS > landing.fastCmS) {
            landing.fastCmS = speedCmS;
            landing.fastCourseDeg = nowCourseDeg;
        }
    }

    if (!landing.hasPattern) {
        if (landing.lapDeg < 360.0f || fabsf(altitudeM() - approachAltM()) >= LANDING_WING_APPROACH_ALT_WINDOW_M) {
            return;
        }
        if (!landing.hasHeading) {
            landing.hasHeading = true;
            landing.headingDeg = landing.lastCourseDeg;
        }
        const float headingDeg = windwardHeadingDeg(landing.headingDeg);
        landingWingPattern(&landing.site, headingDeg, wingTurnSign(autopilotWingConfig()->landSide), &landing.pattern);
        const float headingRad = DEGREES_TO_RADIANS(headingDeg);
        landing.finalCourseDeg = fmodf(headingDeg + 360.0f, 360.0f);
        landing.alongFinal = (vector2_t){{ sin_approx(headingRad), cos_approx(headingRad) }};
        landing.hasPattern = true;
    }

    if (!est->isValidXY) {
        return;
    }
    const vector2_t craftM = positionEstimateHorizontalM(est);
    vector2_t toEntryM;
    vector2Sub(&toEntryM, &landing.pattern.entryEnuM, &craftM);
    const float bearingDeg = RADIANS_TO_DEGREES(atan2_approx(toEntryM.x, toEntryM.y));
    if (fabsf(wrapDeg180f(bearingDeg - courseDeg(est))) < LANDING_WING_ENTRY_BEARING_DEG) {
        const float turnDistM = autopilotWingTurnDistanceM(turnDeg(&craftM, &landing.pattern.entryEnuM, &landing.pattern.baseEnuM));
        flyLeg(LANDING_WING_ALIGN, &craftM, &landing.pattern.entryEnuM, approachAltM(), landing.site.sinkRateMps, turnDistM, nowUs);
    }
}

static void goAround(landingWingGoAround_e cause, const positionEstimate3d_t *est, timeUs_t nowUs)
{
    const vector2_t craftM = positionEstimateHorizontalM(est);
    const vector2_t aheadM = {{ craftM.x + LANDING_WING_LINE_AHEAD_M * landing.alongFinal.x,
                                craftM.y + LANDING_WING_LINE_AHEAD_M * landing.alongFinal.y }};
    landing.goAround = cause;
    landing.pullUp = landing.phase == LANDING_WING_FINAL || landing.phase == LANDING_WING_FLARE;
    landingWingTouchdownReset(&landing.touchdown);
    // climbing from wherever it is, even above where the approach had it
    flyLegFrom(LANDING_WING_GO_AROUND, &craftM, &aheadM, approachAltM(), autopilotWingConfig()->maxClimbRate * 0.1f,
               fmaxf(flightPlanWingCommandedAltitudeM(), altitudeM()), 0.0f, nowUs);
}

static bool pilotGoesAround(void)
{
    return rxAreFlightChannelsValid() && !failsafeIsActive()
        && (getRcDeflectionAbs(FD_ROLL) > LANDING_WING_STICK_GO_AROUND || getRcDeflectionAbs(FD_PITCH) > LANDING_WING_STICK_GO_AROUND);
}

static void enterFinal(const positionEstimate3d_t *est, timeUs_t nowUs)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    const float speedMps = positionEstimateGroundspeedCmS(est) * 0.01f;
    const float vertRateMps = fminf(LANDING_WING_FINAL_SINK_MARGIN * speedMps * tan_approx(glideSlopeRad()), cfg->maxSinkRate * 0.1f);
    landing.finalEstablished = false;
    landing.floatM = flareFloatM(positionEstimateGroundspeedCmS(est));
    landing.reckonedAltitudeM = altitudeM();
    landing.reckonedUs = nowUs;
    flyLeg(LANDING_WING_FINAL, &landing.pattern.finalEnuM, &landing.pattern.touchdownEnuM,
           landing.site.touchdownEnuM.v[ENU_U] + landing.pattern.finalHeightM, vertRateMps, 0.0f, nowUs);
}

static void updateFinal(const positionEstimate3d_t *est, timeUs_t nowUs)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    const vector2_t craftM = positionEstimateHorizontalM(est);
    float crossTrackM;
    const float toTouchdownM = toGoM(&landing.pattern.finalEnuM, &landing.pattern.touchdownEnuM, &craftM, &crossTrackM);
    const float nowHeightM = heightM();
    const float speedCmS = positionEstimateGroundspeedCmS(est);
    if (landingWingGlideAltM(toTouchdownM - landing.floatM, landing.pattern.finalHeightM, glideSlopeRad()) >= landing.pattern.finalHeightM) {
        landing.floatM = flareFloatM(speedCmS);
    }
    const float glideM = landingWingGlideAltM(toTouchdownM - landing.floatM, landing.pattern.finalHeightM, glideSlopeRad());
    const float flareM = flareHeightM();
    landing.glideSinkCmS = fmaxf(speedCmS * tan_approx(glideSlopeRad()), flareSinkCmS());
    const float reckonS = cmpTimeUs(nowUs, landing.reckonedUs) * 1e-6f;
    landing.reckonedAltitudeM += est->velocity.v[ENU_U] * 0.01f * reckonS
                               + (altitudeM() - landing.reckonedAltitudeM) * fminf(reckonS / LANDING_WING_RECKON_TAU_S, 1.0f);
    landing.reckonedUs = nowUs;

    positionNavLowerTargetAltitude(landing.site.touchdownEnuM.v[ENU_U] + glideM);

    // ahead of the slope it is still settling onto final, and may bank as it needs to
    if (glideM < landing.pattern.finalHeightM
        && fabsf(wrapDeg180f(courseDeg(est) - landing.finalCourseDeg)) < LANDING_WING_FINAL_ESTABLISHED_DEG) {
        landing.finalEstablished = true;
    }
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    limits.throttleMax = fminf(limits.throttleMax, cfg->cruiseThrottle * 0.01f);
    if (glideM < landing.pattern.finalHeightM) {
        limits.throttleForPath = true;
        limits.throttleMin = 0.0f;
    }
    if (landing.finalEstablished) {
        limits.bankLimitDeg = fmaxf(cfg->landFinalBank * constrainf((nowHeightM - flareM) / flareM, 0.0f, 1.0f), LANDING_WING_FLARE_BANK_DEG);
    }
    if (!est->isValidXY && nowHeightM <= lowHeightM()) {
        // too low to go around for the position: straight on down the glide slope
        limits.wingsLevel = true;
        limits.climbRateOverride = true;
        limits.climbRateCmS = -landing.glideSinkCmS;
    }
    autopilotWingSetLimits(AP_WING_LIMITS_LANDING, &limits);

    if (pilotGoesAround()) {
        goAround(LANDING_WING_GO_AROUND_STICK, est, nowUs);
        return;
    }
    if (!committed()) {
        landingWingGoAround_e cause = LANDING_WING_GO_AROUND_NONE;
        if (!est->isValidXY && nowHeightM > lowHeightM()) {
            cause = LANDING_WING_GO_AROUND_POSITION;
        } else if (toTouchdownM < -fmaxf(LANDING_WING_OVERSHOOT_MIN_M, LANDING_WING_OVERSHOOT_TIME_S * speedCmS * 0.01f) && nowHeightM > flareM) {
            cause = LANDING_WING_GO_AROUND_OVERSHOOT;
        } else if ((toTouchdownM > LANDING_WING_SLOPE_CHECK_M && fabsf(nowHeightM - glideM) > cfg->landSlopeTolerance)
            || (fabsf(altitudeM() - landing.reckonedAltitudeM) > cfg->landSlopeTolerance && nowHeightM > flareM)) {
            cause = LANDING_WING_GO_AROUND_SLOPE;
        } else if (landing.finalEstablished && fabsf(crossTrackM) > LANDING_WING_CROSS_TRACK_MAX_M
            && nowHeightM < LANDING_WING_CROSS_TRACK_HEIGHT_M) {
            cause = LANDING_WING_GO_AROUND_CROSS_TRACK;
        }
        if (cause != LANDING_WING_GO_AROUND_NONE) {
            goAround(cause, est, nowUs);
            return;
        }
    }

    if (nowHeightM <= flareM) {
        flareStart(&landing.touchdown.flare, nowUs);
        setPhase(LANDING_WING_FLARE, nowUs);
    } else if (nowHeightM < lowHeightM() && landingWingTouchdownUpdate(&landing.touchdown, nowUs, nowHeightM, landing.glideSinkCmS)) {
        setPhase(LANDING_WING_TOUCHDOWN, nowUs);
    }
}

static void updateFlare(const positionEstimate3d_t *est, timeUs_t nowUs)
{
    const float nowHeightM = heightM();
    autopilotWingLimits_t limits;
    autopilotWingDefaultLimits(&limits);
    const float sinkCmS = flareLimits(&landing.touchdown.flare, nowUs, nowHeightM, &limits);
    limits.wingsLevel = !est->isValidXY;
    limits.bankLimitDeg = LANDING_WING_FLARE_BANK_DEG;
    autopilotWingSetLimits(AP_WING_LIMITS_LANDING, &limits);

    if (pilotGoesAround()) {
        goAround(LANDING_WING_GO_AROUND_STICK, est, nowUs);
    } else if (landingWingTouchdownUpdate(&landing.touchdown, nowUs, nowHeightM, sinkCmS)
        || (cmpTimeUs(nowUs, landing.phaseUs) > LANDING_WING_FLARE_GIVE_UP_US && positionEstimateGroundspeedCmS(est) < LANDING_WING_FLARE_GIVE_UP_SPEED_CMS)) {
        setPhase(LANDING_WING_TOUCHDOWN, nowUs);
    }
}

static void updateGoAround(const positionEstimate3d_t *est, timeUs_t nowUs)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    const bool pullingUp = landing.pullUp
        && (cmpTimeUs(nowUs, landing.phaseUs) < LANDING_WING_PULL_UP_US
            || heightM() < flareHeightM() + LANDING_WING_PULL_UP_CLEARANCE_M);
    if (pullingUp) {
        autopilotWingLimits_t limits;
        autopilotWingDefaultLimits(&limits);
        limits.wingsLevel = true;
        limits.climbRateOverride = true;
        limits.climbRateCmS = cfg->maxClimbRate * 10.0f;
        limits.throttleMin = limits.throttleMax;
        autopilotWingSetLimits(AP_WING_LIMITS_LANDING, &limits);
        return;
    }
    autopilotWingClearLimits(AP_WING_LIMITS_LANDING);
    if (est->isValidXY && fabsf(altitudeM() - approachAltM()) < LANDING_WING_GO_AROUND_ALT_WINDOW_M) {
        landing.attempts++;
        loiterDown(nowUs);
    }
}

static void updateLeg(const positionEstimate3d_t *est, timeUs_t nowUs)
{
    if (!est->isValidXY && !committed() && heightM() > lowHeightM()) {
        goAround(LANDING_WING_GO_AROUND_POSITION, est, nowUs);
        return;
    }
    const vector2_t craftM = positionEstimateHorizontalM(est);
    vector2_t toEndM;
    vector2Sub(&toEndM, &landing.legEndEnuM, &craftM);
    if (toGoM(&landing.legStartEnuM, &landing.legEndEnuM, &craftM, NULL) > landing.turnDistM
        && vector2Norm(&toEndM) > landing.turnDistM) {
        return;
    }

    const float rightAngleTurnM = autopilotWingTurnDistanceM(90.0f);
    switch (landing.phase) {
    case LANDING_WING_ALIGN:
        flyLeg(LANDING_WING_DOWNWIND, &landing.pattern.entryEnuM, &landing.pattern.baseEnuM, approachAltM(),
               landing.site.sinkRateMps, rightAngleTurnM, nowUs);
        break;
    case LANDING_WING_DOWNWIND: {
        // base comes down from the approach altitude to the top of the glide slope over its length
        const autopilotWingConfig_t *cfg = autopilotWingConfig();
        vector2_t baseM;
        vector2Sub(&baseM, &landing.pattern.finalEnuM, &landing.pattern.baseEnuM);
        const float baseTimeS = vector2Norm(&baseM) / fmaxf(positionEstimateGroundspeedCmS(est) * 0.01f, LANDING_WING_COURSE_MIN_SPEED_CMS * 0.01f);
        const float vertRateMps = fminf((cfg->landApproachAlt - landing.pattern.finalHeightM) / baseTimeS, cfg->maxSinkRate * 0.1f);
        flyLeg(LANDING_WING_BASE, &landing.pattern.baseEnuM, &landing.pattern.finalEnuM,
               landing.site.touchdownEnuM.v[ENU_U] + landing.pattern.finalHeightM, vertRateMps, rightAngleTurnM, nowUs);
        break;
    }
    default:
        enterFinal(est, nowUs);
        break;
    }
}

static void updateDescent(timeUs_t nowUs)
{
    if (landingWingDescend(AP_WING_LIMITS_LANDING, &landing.touchdown, nowUs, descentSinkMps() * 100.0f)) {
        setPhase(LANDING_WING_TOUCHDOWN, nowUs);
    }
}

bool landingWingUpdate(timeUs_t nowUs)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();

    switch (landing.phase) {
    case LANDING_WING_LOITER_DOWN:
        updateLoiterDown(est, nowUs);
        break;
    case LANDING_WING_ALIGN:
    case LANDING_WING_DOWNWIND:
    case LANDING_WING_BASE:
        updateLeg(est, nowUs);
        break;
    case LANDING_WING_FINAL:
        updateFinal(est, nowUs);
        break;
    case LANDING_WING_FLARE:
        updateFlare(est, nowUs);
        break;
    case LANDING_WING_GO_AROUND:
        updateGoAround(est, nowUs);
        break;
    case LANDING_WING_DESCEND:
        updateDescent(nowUs);
        break;
    default:
        break;
    }

    if (landing.phase != LANDING_WING_IDLE) {
        const vector2_t craftM = positionEstimateHorizontalM(est);
        float crossTrackM = 0.0f;
        const float toTouchdownM = landing.hasPattern
            ? toGoM(&landing.pattern.finalEnuM, &landing.pattern.touchdownEnuM, &craftM, &crossTrackM) : 0.0f;
        DEBUG_SET(DEBUG_AUTOPILOT_LANDING, 0, landing.phase);                                             //!< Landing Phase [enum:landingWingPhase_e]
        DEBUG_SET(DEBUG_AUTOPILOT_LANDING, 1, landing.attempts);                                          //!< Go-Arounds Flown
        DEBUG_SET(DEBUG_AUTOPILOT_LANDING, 2, landing.goAround);                                          //!< Last Go-Around Cause [enum:landingWingGoAround_e]
        DEBUG_SET(DEBUG_AUTOPILOT_LANDING, 3, lrintf(toTouchdownM));                                      //!< Distance To Touchdown Along Final [unit:m]
        DEBUG_SET(DEBUG_AUTOPILOT_LANDING, 4, lrintf(crossTrackM * 10.0f));                               //!< Cross Track Error Right Of Final [unit:0.1m]
        DEBUG_SET(DEBUG_AUTOPILOT_LANDING, 5, lrintf(heightM() * 100.0f));                                //!< Height Above Touchdown [unit:cm]
        DEBUG_SET(DEBUG_AUTOPILOT_LANDING, 6, landing.hasPattern ? lrintf(landing.finalCourseDeg) : -1);  //!< Landing Heading [unit:deg]
        DEBUG_SET(DEBUG_AUTOPILOT_LANDING, 7, landing.touchdown.still ? 1 : 0);                           //!< Still On The Ground
    }

    return landing.phase == LANDING_WING_TOUCHDOWN;
}

#endif // ENABLE_FLIGHT_PLAN

#endif // USE_WING

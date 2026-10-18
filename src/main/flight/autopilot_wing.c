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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#ifdef USE_WING

#include "build/build_config.h"
#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "drivers/time.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/launch_wing.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pos_hold.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "flight/position_nav.h"
#include "rx/rx.h"
#include "scheduler/scheduler.h"
#include "sensors/gyro.h"

#include "pg/autopilot.h"
#include "pg/autopilot_wing.h"
#include "autopilot.h"

#define WING_RESEED_GAP_US            200000  // a pause this long leaves the pitch and throttle state stale
#define WING_THROTTLE_SLEW_PER_S      0.5f
#define WING_BANK_LIMIT_DEG           60.0f   // turn compensation stops growing here
#define WING_TURN_FF_FADE_DEG         70.0f   // and fades out from here to nothing at WING_TURN_FF_ZERO_DEG
#define WING_TURN_FF_ZERO_DEG         80.0f
#define WING_INTEGRATOR_BANK_MARGIN_DEG 5.0f  // banked past guidance's own limit, the lost lift is not a trim error
#define WING_ROLL_SLEW_DPS            45.0f
#define WING_MIN_GROUNDSPEED_CMS      300.0f  // slower than this the track is noise, so the nose gives the direction
#define WING_NO_POSITION_MIN_BANK_DEG 10.0f
#define WING_L1_MIN_CM                1000.0f
#define WING_TRACK_BEHIND_COS         -0.7071f // further round than 135 deg from the track, behind its start
#define WING_TRACK_CAPTURE_SIN        0.866f   // the cross-track error closes at no steeper than 60 deg
#define WING_TURN_LATCH_RAD           DEGREES_TO_RADIANS(135.0f)
#define WING_MIN_TURN_BANK_FRACTION   0.8f     // the turns planned for leave this much of the bank in hand
#define WING_LINE_SETTLE_L1           2.0f     // look-ahead distances flown settling onto a line turned onto
#define WING_PATH_IDLE_DEG            2.0f     // the nose this far above the path still glides
#define WING_PATH_CRUISE_DEG          5.0f     // and this far above needs the cruise throttle
#define GRAVITY_CMSS                  (G_ACCELERATION * 100.0f)

typedef enum {
    WING_LATERAL_CAPTURE = 1 << 0,
    WING_LATERAL_NO_POSITION = 1 << 1,
    WING_LATERAL_PILOT = 1 << 2,
} wingLateralFlags_e;

#ifndef POSHOLD_TASK_RATE_HZ
#define POSHOLD_TASK_RATE_HZ 100
#endif

float autopilotAngle[RP_AXIS_COUNT];

// Pitch is carried nose-up from the trim attitude (angle_pitch_offset), the opposite sign to
// autopilotAngle[] and the attitude estimate.
typedef struct {
    bool engaged;
    float pitchDeg;
    float integralDeg;
    float throttle;
    float climbDemandCmS;
    float turnPitchRateDps;
    timeUs_t lastRunUs;
} wingVertical_t;

static wingVertical_t wingVert;

typedef enum {
    WING_GUIDANCE_LOITER = 0,
    WING_GUIDANCE_LINE,
    WING_GUIDANCE_NO_POSITION,
    WING_GUIDANCE_PILOT,
} wingGuidance_e;

typedef struct {
    bool engaged;
    bool sticksActive;
    bool hasCentre;
    vector2_t centreCm;         // ENU, where the loiter flies without a nav target
    bool navActive;
    bool hasPointStart;
    uint32_t navSequence;
    vector2_t pointStartCm;     // ENU, where the aircraft was when the nav target was set
    int8_t turnLatch;
    float rollDeg;
    timeUs_t lastRunUs;
} wingLateral_t;

static wingLateral_t wingLat;

static bool wingLimitsSet[AP_WING_LIMITS_OWNER_COUNT];
static autopilotWingLimits_t wingLimits[AP_WING_LIMITS_OWNER_COUNT];

void resetPositionControl(unsigned taskRateHz)
{
    UNUSED(taskRateHz);
    wingLat.engaged = false;
    wingLat.sticksActive = false;
    wingLat.hasCentre = false;
    wingLat.hasPointStart = false;
    wingLat.turnLatch = 0;
}

void autopilotInit(void)
{
    resetAltitudeControl();
    resetPositionControl(POSHOLD_TASK_RATE_HZ);
    for (int owner = 0; owner < AP_WING_LIMITS_OWNER_COUNT; owner++) {
        autopilotWingClearLimits(owner);
    }
}

void autopilotWingDefaultLimits(autopilotWingLimits_t *limits)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    *limits = (autopilotWingLimits_t){
        .bankLimitDeg = cfg->maxBank,
        .pitchMinDeg = -cfg->maxDiveAngle,
        .pitchMaxDeg = cfg->maxClimbAngle,
        .throttleMin = cfg->minThrottle * 0.01f,
        .throttleMax = cfg->maxThrottle * 0.01f,
    };
}

void autopilotWingSetLimits(autopilotWingLimitsOwner_e owner, const autopilotWingLimits_t *limits)
{
    wingLimits[owner] = *limits;
    wingLimitsSet[owner] = true;
}

void autopilotWingClearLimits(autopilotWingLimitsOwner_e owner)
{
    wingLimitsSet[owner] = false;
}

// The narrowest bounds every owner allows. A closed throttle wins over a raised floor, and the first
// owner to set a climb rate flies it.
static void currentLimits(autopilotWingLimits_t *limits)
{
    bool merged = false;
    for (int owner = 0; owner < AP_WING_LIMITS_OWNER_COUNT; owner++) {
        if (!wingLimitsSet[owner]) {
            continue;
        }
        const autopilotWingLimits_t *own = &wingLimits[owner];
        if (!merged) {
            *limits = *own;
            merged = true;
            continue;
        }
        limits->bankLimitDeg = fminf(limits->bankLimitDeg, own->bankLimitDeg);
        limits->pitchMinDeg = fmaxf(limits->pitchMinDeg, own->pitchMinDeg);
        limits->pitchMaxDeg = fminf(limits->pitchMaxDeg, own->pitchMaxDeg);
        limits->throttleMin = fmaxf(limits->throttleMin, own->throttleMin);
        limits->throttleMax = fminf(limits->throttleMax, own->throttleMax);
        limits->wingsLevel = limits->wingsLevel || own->wingsLevel;
        limits->motorStop = limits->motorStop || own->motorStop;
        limits->throttleForPath = limits->throttleForPath || own->throttleForPath;
        if (!limits->climbRateOverride && own->climbRateOverride) {
            limits->climbRateOverride = true;
            limits->climbRateCmS = own->climbRateCmS;
        }
    }
    if (!merged) {
        autopilotWingDefaultLimits(limits);
    }
    limits->throttleMin = fminf(limits->throttleMin, limits->throttleMax);
}

bool autopilotWingMotorStopRequested(void)
{
    for (int owner = 0; owner < AP_WING_LIMITS_OWNER_COUNT; owner++) {
        if (wingLimitsSet[owner] && wingLimits[owner].motorStop) {
            return true;
        }
    }
    return false;
}

void resetAltitudeControl(void)
{
    wingVert.engaged = false;
    wingVert.integralDeg = 0.0f;
    wingVert.turnPitchRateDps = 0.0f;
}

static float flightPathAngleDeg(float climbRateCmS, float speedCmS)
{
    return RADIANS_TO_DEGREES(asin_approx(constrainf(climbRateCmS / speedCmS, -1.0f, 1.0f)));
}

static float throttleForPitch(const autopilotWingConfig_t *cfg, const autopilotWingLimits_t *limits, float pitchDeg, float tanSqBank)
{
    const float cruise = cfg->cruiseThrottle * 0.01f;
    const float min = cfg->minThrottle * 0.01f;
    const float max = cfg->maxThrottle * 0.01f;
    float throttle = (pitchDeg > 0.0f) ? cruise + (max - cruise) * pitchDeg / cfg->maxClimbAngle
                                       : cruise + (min - cruise) * pitchDeg / -cfg->maxDiveAngle;
    throttle += cfg->bankThrottle * 0.01f * tanSqBank;
    return constrainf(throttle, limits->throttleMin, limits->throttleMax);
}

static float throttleForPath(const autopilotWingConfig_t *cfg, const autopilotWingLimits_t *limits, float aboveDeg, float tanSqBank)
{
    float throttle = cfg->cruiseThrottle * 0.01f
                   * constrainf((aboveDeg - WING_PATH_IDLE_DEG) / (WING_PATH_CRUISE_DEG - WING_PATH_IDLE_DEG), 0.0f, 1.0f);
    throttle += cfg->bankThrottle * 0.01f * tanSqBank;
    return constrainf(throttle, limits->throttleMin, limits->throttleMax);
}

// The commanded climb's path over the ground: into a headwind it is steeper there than through the
// air, so the nose stands further above it and the throttle opens sooner, keeping the speed up. No
// steeper than the nose may dive, so that the nose pushed down there idles.
static float pathOverGroundDeg(float climbRateCmS)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float speedCmS = est->isValidXY ? positionEstimateGroundspeedCmS(est) : cfg->cruiseSpeed * 10.0f;
    return fmaxf(RADIANS_TO_DEGREES(atan2_approx(climbRateCmS, fmaxf(speedCmS, WING_MIN_GROUNDSPEED_CMS))), -cfg->maxDiveAngle);
}

float autopilotWingIdlePitchDeg(float climbRateCmS)
{
    return pathOverGroundDeg(climbRateCmS) + WING_PATH_IDLE_DEG;
}

float autopilotWingAttitudePitchDeg(void)
{
    return -(attitude.values.pitch - currentPidProfile->angle_pitch_offset) / 10.0f;
}

void altitudeControl(float targetAltitudeCm, timeUs_t taskIntervalUs, float targetAltitudeVelCmS, float velLimitCmS)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    autopilotWingLimits_t limits;
    currentLimits(&limits);
    const float dtS = US_TO_INTERVAL(taskIntervalUs);
    const timeUs_t nowUs = micros();
    const float speedCmS = cfg->cruiseSpeed * 10.0f;
    const float maxClimbDeg = fminf(limits.pitchMaxDeg, cfg->maxClimbAngle);
    const float maxDiveDeg = fminf(-limits.pitchMinDeg, cfg->maxDiveAngle);
    const float integralLimitDeg = MAX(cfg->maxClimbAngle, cfg->maxDiveAngle);
    const float altitudeCm = getAltitudeCmControl();
    const float climbRateCmS = getAltitudeDerivativeControl();

    // The integrator starts out holding the angle of attack the aircraft is flying at, so a hold
    // entered from steady flight carries on at the same pitch.
    if (!wingVert.engaged || cmpTimeUs(nowUs, wingVert.lastRunUs) > WING_RESEED_GAP_US) {
        const float pitchDeg = autopilotWingAttitudePitchDeg();
        wingVert.pitchDeg = constrainf(pitchDeg, -maxDiveDeg, maxClimbDeg);
        wingVert.integralDeg = constrainf(pitchDeg - flightPathAngleDeg(climbRateCmS, speedCmS), -integralLimitDeg, integralLimitDeg);
        if (!wingVert.engaged) {
            wingVert.throttle = (FLIGHT_MODE(LAUNCH_MODE) && launchWingThrottleValid()) ? launchWingGetThrottle() : mixerGetThrottle();
        }
    }
    wingVert.lastRunUs = nowUs;

    float climbDemandCmS = limits.climbRateOverride ? limits.climbRateCmS
                         : targetAltitudeVelCmS + (targetAltitudeCm - altitudeCm) / (cfg->altTimeConstant * 0.1f);
    climbDemandCmS = constrainf(climbDemandCmS, -cfg->maxSinkRate * 10.0f, cfg->maxClimbRate * 10.0f);
    if (velLimitCmS > 1.0f) {
        climbDemandCmS = constrainf(climbDemandCmS, -velLimitCmS, velLimitCmS);
    }

    wingVert.climbDemandCmS = climbDemandCmS;
    const float climbErrorMs = (climbDemandCmS - climbRateCmS) * 0.01f;
    const float pitchDemandDeg = flightPathAngleDeg(climbDemandCmS, speedCmS)
                               + cfg->climbP * 0.1f * climbErrorMs
                               + wingVert.integralDeg;

    const float rollDeg = attitude.values.roll / 10.0f;
    const bool saturated = (climbErrorMs > 0.0f && pitchDemandDeg >= maxClimbDeg)
                        || (climbErrorMs < 0.0f && pitchDemandDeg <= -maxDiveDeg);
    if (!saturated && fabsf(rollDeg) <= cfg->maxBank + WING_INTEGRATOR_BANK_MARGIN_DEG) {
        wingVert.integralDeg += cfg->climbI * 0.1f * climbErrorMs * dtS;
        wingVert.integralDeg = constrainf(wingVert.integralDeg, -integralLimitDeg, integralLimitDeg);
    }

    const float pitchStepDeg = RADIANS_TO_DEGREES(cfg->vertAccel * 10.0f / speedCmS) * dtS;
    wingVert.pitchDeg += constrainf(pitchDemandDeg - wingVert.pitchDeg, -pitchStepDeg, pitchStepDeg);
    wingVert.pitchDeg = constrainf(wingVert.pitchDeg, -maxDiveDeg, maxClimbDeg);

    const float bankRad = DEGREES_TO_RADIANS(constrainf(rollDeg, -WING_BANK_LIMIT_DEG, WING_BANK_LIMIT_DEG));
    const float turnFfScale = constrainf((WING_TURN_FF_ZERO_DEG - fabsf(rollDeg)) / (WING_TURN_FF_ZERO_DEG - WING_TURN_FF_FADE_DEG), 0.0f, 1.0f);
    const float sinBank = sin_approx(bankRad);
    const float cosBank = cos_approx(bankRad);
    const float tanSqBank = sq(sinBank / cosBank);

    const float throttleDemand = limits.throttleForPath
        ? throttleForPath(cfg, &limits, wingVert.pitchDeg - pathOverGroundDeg(climbDemandCmS), tanSqBank)
        : throttleForPitch(cfg, &limits, wingVert.pitchDeg, tanSqBank);
    const float throttleStep = WING_THROTTLE_SLEW_PER_S * dtS;
    wingVert.throttle += constrainf(throttleDemand - wingVert.throttle, -throttleStep, throttleStep);
    if (limits.throttleMin > cfg->minThrottle * 0.01f) {
        wingVert.throttle = fmaxf(wingVert.throttle, limits.throttleMin);
    }

    // A level turn pitches the nose up through the turn at g * tan(bank) * sin(bank) / V; the
    // angle loop only sees attitude, so without this it holds the turn rate off as an error. Rolled
    // towards inverted, nose up is towards the ground.
    wingVert.turnPitchRateDps = -cfg->turnPitchFf * 0.01f * turnFfScale
                              * RADIANS_TO_DEGREES(GRAVITY_CMSS * sinBank * sinBank / (cosBank * speedCmS));

    autopilotAngle[AI_PITCH] = -wingVert.pitchDeg;
    wingVert.engaged = true;

    const float throttlePwm = scaleRangef(wingVert.throttle, 0.0f, 1.0f, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX);
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 0, lrintf(throttlePwm));        //!< Throttle Output [unit:us]
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(targetAltitudeCm));   //!< Target Altitude [unit:cm]
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 2, lrintf(altitudeCm));         //!< Current Altitude [unit:cm]

    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 2, lrintf(altitudeCm));        //!< Current Altitude [unit:cm]
    DEBUG_SET(DEBUG_GPS_RESCUE_TRACKING, 3, lrintf(targetAltitudeCm));  //!< Target Altitude [unit:cm]

    DEBUG_SET(DEBUG_AUTOPILOT_CLIMB, 0, lrintf(climbDemandCmS));                     //!< Climb Rate Demand [unit:cm/s]
    DEBUG_SET(DEBUG_AUTOPILOT_CLIMB, 1, lrintf(climbRateCmS));                       //!< Climb Rate [unit:cm/s]
    DEBUG_SET(DEBUG_AUTOPILOT_CLIMB, 2, lrintf(wingVert.integralDeg * 10.0f));       //!< Pitch Integrator [unit:0.1deg]
    DEBUG_SET(DEBUG_AUTOPILOT_CLIMB, 3, lrintf(wingVert.pitchDeg * 10.0f));          //!< Pitch Demand From Trim [unit:0.1deg]
    DEBUG_SET(DEBUG_AUTOPILOT_CLIMB, 4, lrintf(wingVert.turnPitchRateDps * 10.0f));  //!< Turn Pitch Rate [unit:0.1dps]
}

float autopilotGetTurnPitchRateDps(void)
{
    return wingVert.engaged ? wingVert.turnPitchRateDps : 0.0f;
}

#ifdef UNIT_TEST
float autopilotWingGetPitchIntegratorDeg(void)
{
    return wingVert.integralDeg;
}

float autopilotWingGetClimbDemandCmS(void)
{
    return wingVert.climbDemandCmS;
}
#endif

// TASK_ALTHOLD runs off the estimator's updates, so the interval is measured. The bounds keep the
// first run after the task is enabled, or one starved behind a long-running peer, from handing the
// integrator a step of arbitrary size.
#define AP_TASK_INTERVAL_MIN_DIVIDER    4
#define AP_TASK_INTERVAL_MAX_MULTIPLIER 4

timeUs_t autopilotTaskIntervalUs(timeUs_t nominalIntervalUs)
{
    timeDelta_t intervalUs = getTaskDeltaTimeUs(TASK_SELF);
    if (intervalUs <= 0) {
        intervalUs = nominalIntervalUs;
    }
    return constrain(intervalUs,
                     (timeDelta_t)(nominalIntervalUs / AP_TASK_INTERVAL_MIN_DIVIDER),
                     (timeDelta_t)(nominalIntervalUs * AP_TASK_INTERVAL_MAX_MULTIPLIER));
}


static float l1DistanceCm(float speedCmS, float periodS, float damping)
{
    return fmaxf(damping * periodS * speedCmS / M_PIf, WING_L1_MIN_CM);
}

// Positive where b lies clockwise of a, so a lateral acceleration toward b from a is a right turn.
static float clockwiseOf(const vector2_t *a, const vector2_t *b)
{
    return vector2Cross(b, a);
}

// Lateral acceleration, right positive, that brings the aircraft onto a circle about the centre and
// holds it there. Inputs are ENU, the aircraft relative to the centre, in cm and cm/s.
static float loiterLateralAccelCmSS(const autopilotWingConfig_t *cfg, const vector2_t *offset, const vector2_t *vel,
                                    float radiusCm, float direction)
{
    const float periodS = cfg->l1Period * 0.1f;
    const float damping = cfg->l1Damping * 0.01f;
    const float omega = 2.0f * M_PIf / periodS;

    const float speedCmS = vector2Norm(vel);
    const float distanceCm = fmaxf(vector2Norm(offset), 1.0f);
    vector2_t radial;
    vector2Scale(&radial, offset, 1.0f / distanceCm);
    const float radialSpeedCmS = vector2Dot(vel, &radial);
    const float clockwiseSpeedCmS = clockwiseOf(&radial, vel);

    // capture: turn the track towards the centre, saturating once the centre is abeam or behind
    const float sinTrackError = (radialSpeedCmS > 0.0f) ? copysignf(1.0f, clockwiseSpeedCmS) : clockwiseSpeedCmS / speedCmS;
    const float captureAccel = 4.0f * damping * M_PIf * speedCmS / periodS * sinTrackError;

    // circle: the centripetal acceleration for the tangential speed, plus a second order correction
    // of the radius error at the guidance period and damping
    const float tangentialSpeedCmS = direction * clockwiseSpeedCmS;
    float correction = sq(omega) * (distanceCm - radiusCm) + 2.0f * damping * omega * radialSpeedCmS;
    if (radialSpeedCmS > 0.0f && tangentialSpeedCmS < 0.0f) {
        // heading out the wrong way round it may only turn the loiter's way, or it would carry on round backwards
        correction = fmaxf(correction, 0.0f);
    }
    const float centripetal = sq(tangentialSpeedCmS) / fmaxf(0.5f * radiusCm, distanceCm);
    const float circleAccel = direction * (correction + centripetal);

    // the two meet where the capture has turned the aircraft onto the circle; within a look-ahead
    // distance outside it the circle's own correction turns it on the loiter's way, where the
    // capture would head it at the centre and across the circle
    const bool capturing = distanceCm > radiusCm + l1DistanceCm(speedCmS, periodS, damping)
                        && direction * captureAccel < direction * circleAccel;

    DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 2, lrintf(distanceCm - radiusCm));         //!< Radius Error [unit:cm]
    DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 3, lrintf(radialSpeedCmS));                //!< Radial Speed [unit:cm/s]
    DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 4, lrintf(tangentialSpeedCmS));            //!< Tangential Speed [unit:cm/s]
    DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 5, capturing ? WING_LATERAL_CAPTURE : 0);  //!< Guidance State [flags:Capture|No Position|Pilot]

    return capturing ? captureAccel : circleAccel;
}

// Lateral acceleration, right positive, that brings the aircraft onto the line from a through b and
// holds it there: it steers for the point on the line one look-ahead distance away, closing a
// cross-track error at no steeper than 60 deg. Beyond b it carries on along the line. Far enough
// behind a, facing away from it, it flies to a first. turnLatch holds the direction of a turn through
// the reverse of the track, which would otherwise flip as the heading swings across it. ENU, cm and
// cm/s.
STATIC_UNIT_TESTED float wingLineLateralAccelCmSS(const vector2_t *pos, const vector2_t *vel, const vector2_t *a, const vector2_t *b,
                                                  float periodS, float damping, int8_t *turnLatch)
{
    const float speedCmS = vector2Norm(vel);
    const float l1Cm = l1DistanceCm(speedCmS, periodS, damping);

    vector2_t track;
    vector2Sub(&track, b, a);
    const float trackCm = vector2Norm(&track);
    vector2_t fromA;
    vector2Sub(&fromA, pos, a);
    const float fromACm = vector2Norm(&fromA);
    vector2_t trackDir = { .x = 0.0f, .y = 1.0f };
    if (trackCm > 1.0f) {
        vector2Scale(&trackDir, &track, 1.0f / trackCm);
    }
    const float alongCm = vector2Dot(&fromA, &trackDir);
    const float crossTrackCm = -clockwiseOf(&fromA, &trackDir);     // right of the line positive

    float etaRad;
    if (trackCm <= 1.0f || (fromACm > l1Cm && alongCm < WING_TRACK_BEHIND_COS * fromACm)) {
        vector2_t toPoint;
        vector2Sub(&toPoint, (trackCm <= 1.0f) ? b : a, pos);
        etaRad = atan2_approx(clockwiseOf(vel, &toPoint), vector2Dot(vel, &toPoint));
    } else {
        const float headingErrorRad = atan2_approx(clockwiseOf(vel, &trackDir), vector2Dot(vel, &trackDir));
        const float captureRad = asin_approx(constrainf(-crossTrackCm / l1Cm, -WING_TRACK_CAPTURE_SIN, WING_TRACK_CAPTURE_SIN));
        etaRad = headingErrorRad + captureRad;
        if (etaRad > M_PIf) {
            etaRad -= 2.0f * M_PIf;
        } else if (etaRad < -M_PIf) {
            etaRad += 2.0f * M_PIf;
        }
    }

    if (fabsf(etaRad) > WING_TURN_LATCH_RAD) {
        if (*turnLatch == 0) {
            *turnLatch = (etaRad > 0.0f) ? 1 : -1;
        }
        etaRad = *turnLatch * fabsf(etaRad);
    } else {
        *turnLatch = 0;
    }
    etaRad = constrainf(etaRad, -M_PIf / 2.0f, M_PIf / 2.0f);

    DEBUG_SET(DEBUG_FLIGHT_PLAN, 3, lrintf((trackCm - alongCm) * 0.01f));  //!< Distance To Go Along The Track [unit:m]
    DEBUG_SET(DEBUG_FLIGHT_PLAN, 4, lrintf(crossTrackCm * 0.1f));          //!< Cross Track Error Right [unit:0.1m]
    DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 7, lrintf(l1Cm * 0.01f));          //!< Look-Ahead Distance [unit:m]

    return 4.0f * sq(damping) * sq(speedCmS) / l1Cm * sin_approx(etaRad);
}

float autopilotWingL1DistanceM(void)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    const float speedCmS = fmaxf(positionEstimateGroundspeedCmS(positionEstimatorGetEstimate()), WING_MIN_GROUNDSPEED_CMS);
    return l1DistanceCm(speedCmS, cfg->l1Period * 0.1f, cfg->l1Damping * 0.01f) * 0.01f;
}

// The tightest turn planned at the cruise speed with a bank limit.
static float turnRadiusM(float bankLimitDeg)
{
    const float bankRad = DEGREES_TO_RADIANS(WING_MIN_TURN_BANK_FRACTION * bankLimitDeg);
    return sq(autopilotWingConfig()->cruiseSpeed * 0.1f) / (G_ACCELERATION * tan_approx(bankRad));
}

float autopilotWingMinTurnRadiusM(void)
{
    return turnRadiusM(autopilotWingConfig()->maxBank);
}

float autopilotWingTurnDistanceM(float turnDeg)
{
    return autopilotWingL1DistanceM() * fminf(fabsf(turnDeg) / 90.0f, 1.0f);
}

float autopilotWingLineSettleDistanceM(void)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    return WING_LINE_SETTLE_L1 * l1DistanceCm(cfg->cruiseSpeed * 10.0f, cfg->l1Period * 0.1f, cfg->l1Damping * 0.01f) * 0.01f;
}

float autopilotWingLoiterRadiusM(void)
{
    return fmaxf(autopilotWingConfig()->loiterRadius, autopilotWingMinTurnRadiusM());
}

bool positionControl(void)
{
    const autopilotWingConfig_t *cfg = autopilotWingConfig();
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float dtS = US_TO_INTERVAL(autopilotTaskIntervalUs(TASK_PERIOD_HZ(POSHOLD_TASK_RATE_HZ)));
    const timeUs_t nowUs = micros();

    positionNavUpdate(dtS, est);

    const positionNavCommand_t *navCmd = positionNavGetActiveCommand();
    const bool navActive = positionNavHasActiveTarget();
    if (wingLat.navActive && !navActive) {
        // with the nav target gone the aircraft loiters about wherever it is
        wingLat.hasCentre = false;
    }
    wingLat.navActive = navActive;

    if (wingLat.sticksActive) {
        // the circle is drawn afresh around wherever the pilot lets go
        wingLat.engaged = false;
        wingLat.hasCentre = false;
        DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 5, WING_LATERAL_PILOT);   //!< Guidance State [flags:Capture|No Position|Pilot]
        DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 6, WING_GUIDANCE_PILOT);  //!< Guidance [enum:wingGuidance_e]
        return true;
    }

    if (!wingLat.engaged || cmpTimeUs(nowUs, wingLat.lastRunUs) > WING_RESEED_GAP_US) {
        wingLat.rollDeg = attitude.values.roll / 10.0f;
    }
    wingLat.lastRunUs = nowUs;
    wingLat.engaged = true;

    autopilotWingLimits_t limits;
    currentLimits(&limits);
    const float bankLimitDeg = limits.wingsLevel ? 0.0f : fminf(limits.bankLimitDeg, cfg->maxBank);
    const float leastRadiusM = (bankLimitDeg > 0.0f) ? turnRadiusM(bankLimitDeg) : 0.0f;

    wingGuidance_e guidance = WING_GUIDANCE_LOITER;
    float accelCmSS;
    if (est->isValidXY) {
        const vector2_t pos = { .x = est->position.v[ENU_E], .y = est->position.v[ENU_N] };
        vector2_t vel = { .x = est->velocity.v[ENU_E], .y = est->velocity.v[ENU_N] };
        if (vector2NormSq(&vel) < sq(WING_MIN_GROUNDSPEED_CMS)) {
            const float headingRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
            vel.x = WING_MIN_GROUNDSPEED_CMS * sin_approx(headingRad);
            vel.y = WING_MIN_GROUNDSPEED_CMS * cos_approx(headingRad);
        }
        if (navActive && (!wingLat.hasPointStart || wingLat.navSequence != navCmd->sequence)) {
            wingLat.navSequence = navCmd->sequence;
            wingLat.pointStartCm = pos;
            wingLat.hasPointStart = true;
            wingLat.turnLatch = 0;
        }

        if (!navActive) {
            if (!wingLat.hasCentre) {
                wingLat.centreCm = pos;
                wingLat.hasCentre = true;
            }
            vector2_t offset;
            vector2Sub(&offset, &pos, &wingLat.centreCm);
            accelCmSS = loiterLateralAccelCmSS(cfg, &offset, &vel, fmaxf(autopilotWingLoiterRadiusM(), leastRadiusM) * 100.0f,
                                               wingTurnSign(cfg->loiterDirection));
        } else {
            const vector2_t targetCm = { .x = navCmd->targetPosEfM.v[ENU_E] * 100.0f, .y = navCmd->targetPosEfM.v[ENU_N] * 100.0f };
            if (navCmd->track == NAV_TRACK_LOITER) {
                vector2_t offset;
                vector2Sub(&offset, &pos, &targetCm);
                accelCmSS = loiterLateralAccelCmSS(cfg, &offset, &vel, fmaxf(navCmd->loiterRadiusM, leastRadiusM) * 100.0f,
                                                   navCmd->loiterDirection);
            } else {
                vector2_t startCm = wingLat.pointStartCm;
                if (navCmd->track == NAV_TRACK_LINE) {
                    vector2Scale(&startCm, &navCmd->trackStartEfM, 100.0f);
                }
                accelCmSS = wingLineLateralAccelCmSS(&pos, &vel, &startCm, &targetCm,
                                                     cfg->l1Period * 0.1f, cfg->l1Damping * 0.01f, &wingLat.turnLatch);
                guidance = WING_GUIDANCE_LINE;
                DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 5, 0);  //!< Guidance State [flags:Capture|No Position|Pilot]
            }
        }
    } else {
        // Without a position, circle the way the loiter would in still air. The centre is kept, so
        // the loiter picks up where it was when the position returns.
        const float speedCmS = cfg->cruiseSpeed * 10.0f;
        const float minAccelCmSS = GRAVITY_CMSS * tan_approx(DEGREES_TO_RADIANS(WING_NO_POSITION_MIN_BANK_DEG));
        accelCmSS = wingTurnSign(cfg->loiterDirection)
                  * fmaxf(sq(speedCmS) / (fmaxf(autopilotWingLoiterRadiusM(), leastRadiusM) * 100.0f), minAccelCmSS);
        guidance = WING_GUIDANCE_NO_POSITION;
        DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 5, WING_LATERAL_NO_POSITION);  //!< Guidance State [flags:Capture|No Position|Pilot]
    }

    const float bankDeg = constrainf(RADIANS_TO_DEGREES(atan2_approx(accelCmSS, GRAVITY_CMSS)), -bankLimitDeg, bankLimitDeg);
    const float rollStepDeg = WING_ROLL_SLEW_DPS * dtS;
    wingLat.rollDeg += constrainf(bankDeg - wingLat.rollDeg, -rollStepDeg, rollStepDeg);
    autopilotAngle[AI_ROLL] = wingLat.rollDeg;

    DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 0, lrintf(accelCmSS));                //!< Lateral Acceleration Demand [unit:cm/s2]
    DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 1, lrintf(wingLat.rollDeg * 10.0f));  //!< Bank Demand [unit:0.1deg]
    DEBUG_SET(DEBUG_AUTOPILOT_GUIDANCE, 6, guidance);                         //!< Guidance [enum:wingGuidance_e]

    return est->isValidXY;
}

void setSticksActiveStatus(bool areSticksActive)
{
    wingLat.sticksActive = areSticksActive;
}

bool isBelowLandingAltitude(void)
{
    return false;
}

float getAutopilotThrottle(void)
{
    return wingVert.throttle;
}

bool isAutopilotInControl(void)
{
    return wingLat.engaged;
}

float autopilotGetYawRate(void)
{
    return 0.0f;
}

bool autopilotYawControlActive(void)
{
    return false;
}

void autopilotSetYawRateLimit(float rateLimitDps)
{
    UNUSED(rateLimitDps);
}

void autopilotDisableYawControl(void)
{
}

void updateHeadingHold(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
}

void autopilotSetNavHeadingOverride(bool valid, float headingDeg)
{
    UNUSED(valid);
    UNUSED(headingDeg);
}

vector2_t autopilotGetPositionErrorCm(void)
{
    if (!wingLat.hasCentre) {
        return (vector2_t){{ 0.0f, 0.0f }};
    }
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    vector2_t errorCm;
    errorCm.v[ENU_E] = wingLat.centreCm.v[ENU_E] - est->position.v[ENU_E];
    errorCm.v[ENU_N] = wingLat.centreCm.v[ENU_N] - est->position.v[ENU_N];
    return errorCm;
}

void autopilotForceLevelPark(bool request)
{
    UNUSED(request);
}

void autopilotHeadingRecovery(bool active, float pitchDeg)
{
    UNUSED(active);
    UNUSED(pitchDeg);
}

bool autopilotAltitudeControlAvailable(void)
{
    return true;
}

bool autopilotPositionControlAvailable(void)
{
    return true;
}

bool autopilotThrottleValid(void)
{
    return wingVert.engaged;
}

#endif // USE_WING

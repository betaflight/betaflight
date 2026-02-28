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

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/position.h"
#include "rx/rx.h"
#include "sensors/gyro.h"

#include "pg/autopilot.h"
#include "pg/flight_plan.h"
#include "autopilot.h"
#include "autopilot_waypoint.h"
#if ENABLE_FLIGHT_PLAN
#include "autopilot_guidance.h"
#endif

#define ALTITUDE_P_SCALE  0.01f
#define ALTITUDE_I_SCALE  0.003f
#define ALTITUDE_D_SCALE  0.01f
#define ALTITUDE_F_SCALE  0.01f
#define POSITION_P_SCALE  0.0012f
#define POSITION_I_SCALE  0.0001f
#define POSITION_D_SCALE  0.0015f
#define POSITION_A_SCALE  0.0008f
#define UPSAMPLING_CUTOFF_HZ 5.0f
#define YAW_FILTER_CUTOFF_HZ 5.0f
#define THROTTLE_FILTER_CUTOFF_HZ 2.0f

static pidCoefficient_t altitudePidCoeffs;
static pidCoefficient_t positionPidCoeffs;

static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

typedef struct efPidAxis_s {
    bool isStopping;
    bool needsInit;               // Seed previousDistance on first GPS sample
    float previousDistance;
    float previousVelocity;
    float integral;               // Position loop integral
    pt1Filter_t velocityLpf;
    pt1Filter_t accelerationLpf;
    // Velocity controller state
    float velocitySetpoint;       // Desired velocity (cm/s)
    float velocityIntegral;       // Velocity loop integral
    float previousVelocityError;  // For velocity D-term
} efPidAxis_t;

typedef enum {
    // axes are for NED system (North-East-Down); standard aviation coordinate frame
    EAST = 0,   // Y-axis in NED, East
    NORTH       // X-axis in NED, North (swapped with EAST for backward compatibility with array indexing)
} axisEF_e;

typedef struct autopilotState_s {
    gpsLocation_t targetLocation;       // active / current target
    float sanityCheckDistance;
    float upsampleLpfGain;              // for the earth frame upsample filter for position control
    float vaLpfCutoff;                  // velocity + acceleration lowpass filter cutoff
    bool sticksActive;
    float maxAngle;
    vector2_t pidSumEF;                 // pid output in earth frame (East, North), updated at GPS rate
    pt3Filter_t upsampleLpfEF[EF_AXIS_COUNT];    // upsampling filter (earth frame)
    efPidAxis_t efAxis[EF_AXIS_COUNT];

    // Yaw coordination (follows roll for coordinated turns)
    float desiredYawRate;               // deg/s output for RC command
    pt3Filter_t yawRateFilter;          // smooth ramping on yaw rate

    // Throttle smoothing
    pt1Filter_t throttleFilter;         // smooth ramping on throttle output

    // Pilot override detection
    bool pilotOverrideRoll;
    bool pilotOverridePitch;
    bool pilotOverrideYaw;
    bool pilotOverrideThrottle;

    // Climb phase: hold position (or orbit for wing) while climbing to minNavAlt
    bool climbPhaseActive;              // true while below minNavAlt
    bool climbComplete;                 // one-way: prevents re-entering climb after nav starts
    gpsLocation_t holdLocation;         // position to hold during climb
    bool primeUpsampleFilters;          // seed upsampling filters after climb→nav transition
    float climbOrbitAngle;              // wing: current orbit angle during climb phase

    // Glide slope: limits forward velocity to maintain altitude profile
    float glideSlopeStartDist;          // horizontal distance when slope was established (cm)
    float glideSlopeStartAlt;           // altitude when slope was established (cm, relative to home)
    bool glideSlopeActive;              // true once drone has reached minNavAlt
    float navVelocityScale;             // velocity scaling from glide slope (0.0-1.0)
    float altitudeVelocityScale;        // velocity scaling from altitude deficit (0.0-1.0)

    // Velocity buildup: gradual angle authority increase from stationary
    float buildupAngleLimit;            // current dynamic angle limit (degrees), ramps with speed

    // Smooth navigation scaling
    pt1Filter_t navScaleFilter;         // smooth altitude-priority scaling to prevent speed oscillation

    // Ground track: bearing from activation point to current position
    gpsLocation_t activationLocation;   // GPS position when autopilot mode activated

    // Track intercept: heading-based controller for smooth WP→WP navigation.
    // Replaces position PID during L1 guidance for gentle, rate-limited turns.
    float interceptRollCmd;             // current body-frame roll output (degrees)
    float interceptPitchCmd;            // current body-frame pitch output (degrees)
    float previousCourseDeg;            // GPS ground course previous cycle (degrees)
    float measuredTurnRate;             // filtered actual turn rate (deg/s)
    pt1Filter_t turnRateFilter;         // low-pass filter for turn rate measurement
    float previousSpeedCmS;             // previous ground speed for acceleration measurement
    bool interceptInitialized;          // first GPS sample received (for turn rate derivation)
    bool wasInterceptActive;            // previous cycle state (for transition detection)
    float interceptHeadingIntegral;     // AP I-term: persistent heading correction for crosswind
    float interceptSpeedIntegral;       // AP I-term: persistent speed correction for headwind/tailwind
} autopilotState_t;

static autopilotState_t ap = {
    .sanityCheckDistance = 1000.0f,
    .upsampleLpfGain = 1.0f,
    .vaLpfCutoff = 1.0f,
    .sticksActive = false,
};

float autopilotAngle[RP_AXIS_COUNT];

static autopilotDebugState_t apDebug;

const autopilotDebugState_t *autopilotGetDebugState(void)
{
    return &apDebug;
}

static void resetEFAxisFilters(efPidAxis_t* efAxis, const float vaGain)
{
    pt1FilterInit(&efAxis->velocityLpf, vaGain);
    pt1FilterInit(&efAxis->accelerationLpf, vaGain);
}

static void resetEFAxisParams(efPidAxis_t *efAxis, const float vaGain)
{
    // at start only
    resetEFAxisFilters(efAxis, vaGain);
    efAxis->isStopping = true; // Enter starting (deceleration) 'phase'
    efAxis->needsInit = true;  // Seed previousDistance on first GPS sample
    efAxis->integral = 0.0f;
    // Velocity controller state
    efAxis->velocitySetpoint = 0.0f;
    efAxis->velocityIntegral = 0.0f;
    efAxis->previousVelocityError = 0.0f;
}

static void resetUpsampleFilters(void)
{
    for (unsigned i = 0; i < ARRAYLEN(ap.upsampleLpfEF); i++) {
        pt3FilterInit(&ap.upsampleLpfEF[i], ap.upsampleLpfGain);
    }
}

// get sanity distance based on speed
static inline float sanityCheckDistance(const float gpsGroundSpeedCmS)
{
    return fmaxf(1000.0f, gpsGroundSpeedCmS * 2.0f);
    // distance flown in 2s at current speed. with minimum of 10m
}

static void initSmoothingFilters(void)
{
    pt3FilterInit(&ap.yawRateFilter, pt3FilterGain(YAW_FILTER_CUTOFF_HZ, 0.01f));
    pt1FilterInit(&ap.throttleFilter, pt1FilterGain(THROTTLE_FILTER_CUTOFF_HZ, 0.01f));
    pt1FilterInit(&ap.navScaleFilter, pt1FilterGain(1.0f, 0.01f)); // 1Hz: ~0.16s time constant
}

static void primeSmoothingFilters(float yawRate, float throttle)
{
    // Prime yaw PT3 filter (3 cascaded states)
    ap.yawRateFilter.state = yawRate;
    ap.yawRateFilter.state1 = yawRate;
    ap.yawRateFilter.state2 = yawRate;
    // Prime throttle PT1 filter
    ap.throttleFilter.state = throttle;
    // Prime navScale filter at 1.0 (full authority)
    ap.navScaleFilter.state = 1.0f;
}

void resetPositionControl(const gpsLocation_t *initialTargetLocation, unsigned taskRateHz)
{
    // from pos_hold.c (or other client) when initiating position hold at target location
    ap.targetLocation = *initialTargetLocation;
    ap.activationLocation = gpsSol.llh;
    ap.sticksActive = false;
    // set sanity check distance: at least 10m or 2x speed, and for waypoint navigation
    // also at least the distance to the initial target (with 50% margin for overshoot)
    uint32_t distToTarget = 0;
    GPS_distance_cm_bearing(&gpsSol.llh, initialTargetLocation, false, &distToTarget, NULL);
    ap.sanityCheckDistance = fmaxf(sanityCheckDistance(gpsSol.groundSpeed), distToTarget * 1.5f);
    for (unsigned i = 0; i < ARRAYLEN(ap.efAxis); i++) {
        // clear anything stored in the filter at first call
        resetEFAxisParams(&ap.efAxis[i], 1.0f);
    }
    const float taskInterval = 1.0f / taskRateHz;
    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, taskInterval); // 5Hz; normally at 100Hz task rate
    resetUpsampleFilters(); // clear accumulator from previous iterations

    // Reset climb phase state so holdLocation is re-captured on mode re-enable.
    // Without this, a mode toggle (sanity check → disable → re-enable) would reuse
    // a stale holdLocation, causing the sanity check to fail immediately on every cycle.
    ap.climbPhaseActive = false;
    ap.climbComplete = false;
    ap.climbOrbitAngle = 0.0f;

    // Prime upsampling filters on first PID computation so corrections are immediate.
    // Without priming, the PT3 filter ramps from zero, allowing the drone to drift
    // uncorrected until the filter catches up.
    ap.primeUpsampleFilters = true;

    // Reset smoothing filters and prime with safe defaults
    initSmoothingFilters();
    const float hoverThrottleNorm = scaleRangef(autopilotConfig()->hoverThrottle,
        MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    primeSmoothingFilters(0.0f, hoverThrottleNorm);

    // Velocity buildup: start at maxAngle so position hold has full authority.
    // updateAutopilot() will override this for waypoint navigation.
    ap.buildupAngleLimit = ap.maxAngle;

    // Reset intercept controller for clean start
    ap.interceptRollCmd = 0.0f;
    ap.interceptPitchCmd = 0.0f;
    ap.interceptHeadingIntegral = 0.0f;
    ap.interceptSpeedIntegral = 0.0f;
    ap.interceptInitialized = false;
    ap.wasInterceptActive = false;
}

void autopilotInit(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    ap.sticksActive = false;
    ap.maxAngle = cfg->maxAngle;
    altitudePidCoeffs.Kp = cfg->altitudeP * ALTITUDE_P_SCALE;
    altitudePidCoeffs.Ki = cfg->altitudeI * ALTITUDE_I_SCALE;
    altitudePidCoeffs.Kd = cfg->altitudeD * ALTITUDE_D_SCALE;
    altitudePidCoeffs.Kf = cfg->altitudeF * ALTITUDE_F_SCALE;
    positionPidCoeffs.Kp = cfg->positionP * POSITION_P_SCALE;
    positionPidCoeffs.Ki = cfg->positionI * POSITION_I_SCALE;
    positionPidCoeffs.Kd = cfg->positionD * POSITION_D_SCALE;
    positionPidCoeffs.Kf = cfg->positionA * POSITION_A_SCALE; // Kf used for acceleration
    // initialise filters with approximate filter gains; location isn't used at this point.
    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, 0.01f); // 5Hz, assuming 100Hz task rate at init
    resetUpsampleFilters();
    // Initialise PT1 filters for velocity and acceleration in earth frame axes
    ap.vaLpfCutoff = cfg->positionCutoff * 0.01f;
    const float vaGain = pt1FilterGain(ap.vaLpfCutoff,  0.1f); // assume 10Hz GPS connection at start; value is overwritten before first filter use
    for (unsigned i = 0; i < ARRAYLEN(ap.efAxis); i++) {
        resetEFAxisFilters(&ap.efAxis[i], vaGain);
    }

    // Initialize smoothing filters
    initSmoothingFilters();
    primeSmoothingFilters(0.0f, 0.0f);

#if ENABLE_FLIGHT_PLAN
    // Initialize waypoint and L1 guidance systems
    waypointInit();
    l1GuidanceInit();

    // Initialize yaw coordination
    ap.desiredYawRate = 0.0f;

    // Initialize pilot override flags
    ap.pilotOverrideRoll = false;
    ap.pilotOverridePitch = false;
    ap.pilotOverrideYaw = false;
    ap.pilotOverrideThrottle = false;

    // Initialize climb phase state
    ap.climbPhaseActive = false;
    ap.climbComplete = false;
    ap.primeUpsampleFilters = false;
    ap.climbOrbitAngle = 0.0f;

    // Initialize glide slope state
    ap.glideSlopeActive = false;
    ap.glideSlopeStartDist = 0.0f;
    ap.glideSlopeStartAlt = 0.0f;
    ap.navVelocityScale = 1.0f;

    // Velocity buildup: full authority until mode activation
    ap.buildupAngleLimit = ap.maxAngle;

    // Track intercept controller
    ap.interceptRollCmd = 0.0f;
    ap.interceptPitchCmd = 0.0f;
    ap.interceptHeadingIntegral = 0.0f;
    ap.interceptSpeedIntegral = 0.0f;
    ap.previousCourseDeg = 0.0f;
    ap.measuredTurnRate = 0.0f;
    ap.previousSpeedCmS = 0.0f;
    ap.interceptInitialized = false;
    ap.wasInterceptActive = false;
    pt1FilterInit(&ap.turnRateFilter, pt1FilterGain(2.0f, 0.1f));
#endif
}

void resetAltitudeControl (void) {
    altitudeI = 0.0f;
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeStep)
{
    const float verticalVelocityCmS = getAltitudeDerivative();
    const float altitudeErrorCm = targetAltitudeCm - getAltitudeCm();
    const float altitudeP = altitudeErrorCm * altitudePidCoeffs.Kp;

    // reduce the iTerm gain for errors greater than 200cm (2m), otherwise it winds up too much
    const float itermRelax = (fabsf(altitudeErrorCm) < 200.0f) ? 1.0f : 0.1f;
    altitudeI += altitudeErrorCm * altitudePidCoeffs.Ki * itermRelax * taskIntervalS;
    // limit iTerm to not more than 200 throttle units
    altitudeI = constrainf(altitudeI, -200.0f, 200.0f);

    // increase D when velocity is high, typically when initiating hold at high vertical speeds
    // 1.0 when less than 5 m/s, 2x at 10m/s, 2.5 at 20 m/s, 2.8 at 50 m/s, asymptotes towards max 3.0.
    float dBoost = 1.0f;
    {
        const float startValue = 500.0f; // velocity at which D should start to increase
        const float altDeriv = fabsf(verticalVelocityCmS);
        if (altDeriv > startValue) {
            const float ratio = altDeriv / startValue;
            dBoost = (3.0f * ratio - 2.0f) / ratio;
        }
    }

    const float altitudeD = verticalVelocityCmS * dBoost * altitudePidCoeffs.Kd;

    const float altitudeF = targetAltitudeStep * altitudePidCoeffs.Kf;

    const float hoverOffset = autopilotConfig()->hoverThrottle - PWM_RANGE_MIN;
    float throttleOffset = altitudeP + altitudeI - altitudeD + altitudeF + hoverOffset;

    const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
    // 1 = flat, 1.3 at 40 degrees, 1.56 at 50 deg, max 2.0 at 60 degrees or higher
    // note: the default limit of Angle Mode is 60 degrees

    throttleOffset *= tiltMultiplier;

    float newThrottle = PWM_RANGE_MIN + throttleOffset;
    newThrottle = constrainf(newThrottle, autopilotConfig()->throttleMin, autopilotConfig()->throttleMax);
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 0, lrintf(newThrottle)); // normal range 1000-2000 but is before constraint

    newThrottle = scaleRangef(newThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);

    throttleOut = constrainf(newThrottle, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(tiltMultiplier * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 3, lrintf(targetAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 4, lrintf(altitudeP));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 5, lrintf(altitudeI));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 6, lrintf(-altitudeD));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, lrintf(altitudeF));
}

void setSticksActiveStatus(bool areSticksActive)
{
    ap.sticksActive = areSticksActive;
}

static void setTargetLocationByAxis(const gpsLocation_t* newTargetLocation, axisEF_e efAxisIdx)
// not used at present but needed by upcoming GPS code
{
    if (efAxisIdx == EAST) {
        ap.targetLocation.lon = newTargetLocation->lon; // update East-West / / longitude position
    } else {
        ap.targetLocation.lat = newTargetLocation->lat; // update North-South / latitude position
    }
}

// Note: calculateMaxBankAngle (turn rate coordination) was removed here.
// The configured maxAngle is used directly as the bank limit.

// Cascaded velocity controller — inner loop (velocity → angle)
// The outer loop (position → velocity setpoint) is in the caller, where
// both axes are computed first and then vector-limited to maxVelocity.
static float lastDragCompensation;  // for debug capture

static float velocityToAngle(efPidAxis_t *efAxis, float velocitySetpoint, float currentVelocity, float gpsDataInterval, const autopilotConfig_t *cfg)
{
    efAxis->velocitySetpoint = velocitySetpoint;

    // Velocity error → Angle with drag compensation
    const float velocityError = efAxis->velocitySetpoint - currentVelocity;

    // Velocity PID controller
    const float velP = velocityError * (cfg->velocityP * 0.1f);

    // Velocity I-term (with anti-windup)
    if (!efAxis->isStopping) {
        efAxis->velocityIntegral += velocityError * (cfg->velocityI * 0.1f) * gpsDataInterval;
        // Limit integral to ±10 degrees
        efAxis->velocityIntegral = constrainf(efAxis->velocityIntegral, -10.0f, 10.0f);
    } else {
        // Decay integral when stopping
        efAxis->velocityIntegral *= 0.98f;
    }
    const float velI = efAxis->velocityIntegral;

    // Velocity D-term (on error, not measurement)
    const float velocityErrorRate = (velocityError - efAxis->previousVelocityError) / gpsDataInterval;
    efAxis->previousVelocityError = velocityError;
    const float velD = velocityErrorRate * (cfg->velocityD * 0.1f);

    // Quadratic drag compensation (feedforward on setpoint)
    // To maintain the target velocity against drag F = k*v², need angle ∝ k*v².
    // Uses the velocity SETPOINT, not current velocity: feedforward compensates
    // for the expected drag at the desired speed. Using current velocity would
    // fight the PID during deceleration (large reverse velocity → large reverse
    // drag comp → cancels the PID's braking output, preventing deceleration).
    const float dragCoeff = cfg->velocityDragCoeff * 0.0001f;  // Scale from 10000
    const float setpointMS = efAxis->velocitySetpoint * 0.01f;  // cm/s to m/s
    const float dragCompensation = dragCoeff * setpointMS * fabsf(setpointMS);
    lastDragCompensation = dragCompensation;

    // Total angle output (degrees)
    float angleOutput = velP + velI + velD + dragCompensation;

    // Limit to max angle
    angleOutput = constrainf(angleOutput, -cfg->maxAngle, cfg->maxAngle);

    return angleOutput;
}

#if ENABLE_FLIGHT_PLAN
// Heading-based track intercept controller.
// Replaces position PID during L1 guidance for smooth, rate-limited turns.
//
// Roll: banks to achieve a heading change toward the L1 carrot point.
//   - Starts gentle, monitors actual turn rate from GPS ground course.
//   - If turn rate < rate-1 (3 deg/s), increases bank angle.
//   - Roll rate is limited to 10 deg/s for smooth transitions.
//
// Pitch: manages along-track speed.
//   - Targets maxVelocity (scaled by altitude priority).
//   - Hard speed limit: no forward pitch when above maxVelocity.
//
// Output: stores earth-frame equivalent in ap.pidSumEF for the existing
// PT3 upsampling and body-frame rotation path at 100Hz.
static void trackInterceptCompute(float gpsDataInterval)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // Desired bearing from L1 guidance (degrees, 0-360)
    const float desiredBearingDeg = l1GuidanceGetDesiredBearing() * 0.01f;

    // Current ground course (degrees, 0-360)
    const float courseDeg = gpsSol.groundCourse * 0.1f;

    // Heading error (shortest path, positive = need to turn right)
    float headingError = desiredBearingDeg - courseDeg;
    if (headingError > 180.0f) headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;

    // Measure actual turn rate from ground course derivative
    if (ap.interceptInitialized) {
        float courseDelta = courseDeg - ap.previousCourseDeg;
        if (courseDelta > 180.0f) courseDelta -= 360.0f;
        if (courseDelta < -180.0f) courseDelta += 360.0f;
        const float rawTurnRate = courseDelta / gpsDataInterval;
        pt1FilterUpdateCutoff(&ap.turnRateFilter, pt1FilterGain(2.0f, gpsDataInterval));
        ap.measuredTurnRate = pt1FilterApply(&ap.turnRateFilter, rawTurnRate);
    } else {
        ap.interceptInitialized = true;
        ap.measuredTurnRate = 0.0f;
        pt1FilterInit(&ap.turnRateFilter, pt1FilterGain(2.0f, gpsDataInterval));
    }
    ap.previousCourseDeg = courseDeg;

    // Speed and authority scaling
    const float speedCmS = fmaxf((float)gpsSol.groundSpeed, 100.0f); // min 1 m/s

    // Low-speed scaling: reduce heading controller authority when GPS course is unreliable.
    // Below minForwardVelocity, GPS ground course is noisy. Ramps from 0 to 1.
    const float speedAuth = constrainf(
        (speedCmS - 100.0f) / fmaxf((float)cfg->minForwardVelocity - 100.0f, 1.0f),
        0.0f, 1.0f);

    // Determine target turn rate using PD controller on heading error.
    //
    // P-term: turn rate proportional to heading error — larger error, faster turn.
    // D-term: anticipatory damping from measured turn rate. Since heading error
    //   derivative ≈ -measuredTurnRate (desired bearing changes slowly), the D-term
    //   preemptively reduces the turn as the ground course approaches the desired
    //   heading. This prevents heading overshoot: the roll starts reducing well
    //   before the heading is aligned, allowing the craft to settle smoothly.
    //
    // Rate-1 minimum (3 deg/s) ensures meaningful turns when heading error is large.
    const float rate1 = 3.0f;
    const float absHeadingError = fabsf(headingError);
    const float turnDir = (headingError >= 0) ? 1.0f : -1.0f;

    const float headingKp = 0.5f;   // deg/s per deg heading error
    const float headingKd = 0.8f;   // damping: reduces turn rate as heading converges
    const float headingKi = 0.05f;  // deg/s per deg·s accumulated heading error
    float targetTurnRate = headingError * headingKp - ap.measuredTurnRate * headingKd;

    // AP I-term: compensate for persistent crosswind pushing craft off track.
    // Relax integration when heading error is large (intentional turn, not wind drift).
    // Anti-windup: constrain to ±3 deg/s contribution to prevent excessive bank from wind alone.
    {
        const float iRelax = (absHeadingError < 20.0f) ? 1.0f : 0.1f;
        ap.interceptHeadingIntegral += headingError * headingKi * iRelax * gpsDataInterval;
        ap.interceptHeadingIntegral = constrainf(ap.interceptHeadingIntegral, -3.0f, 3.0f);
        targetTurnRate += ap.interceptHeadingIntegral;
    }

    // Constrain to configured max turn rate
    targetTurnRate = constrainf(targetTurnRate,
                                -(float)cfg->maxTurnRate, (float)cfg->maxTurnRate);

    // Ensure at least rate-1 turn when heading error is significant (>10 deg).
    // The D-term may reduce the target below rate-1, but for large errors we
    // need a minimum turn rate to make meaningful progress toward the track.
    if (absHeadingError > 10.0f && fabsf(targetTurnRate) < rate1) {
        targetTurnRate = turnDir * rate1;
    }

    // Bank angle for target turn rate: bank = atan(V * omega / g)
    // Compute from absolute turn rate (turnDir provides sign)
    const float omegaRad = fabsf(targetTurnRate) * (M_PIf / 180.0f);
    float bankAngleDeg = atan2f(speedCmS * omegaRad, 980.0f) * (180.0f / M_PIf);
    bankAngleDeg = constrainf(bankAngleDeg, 0.0f, (float)cfg->maxAngle);

    // Turn rate monitoring: if actual turn rate is below rate-1 despite needing it,
    // boost the bank angle. This handles wind, drag, or other effects that slow the turn.
    if (absHeadingError > 10.0f && fabsf(ap.measuredTurnRate) < rate1 && speedAuth > 0.5f) {
        const float deficit = rate1 - fabsf(ap.measuredTurnRate);
        bankAngleDeg = fminf(bankAngleDeg + deficit * 0.5f, (float)cfg->maxAngle);
    }

    // Roll command (signed, with low-speed authority scaling)
    const float rollTarget = turnDir * bankAngleDeg * speedAuth;

    // Rate limit roll changes: max 10 deg/s for smooth entry and exit
    const float maxRollRate = 10.0f;
    const float maxRollDelta = maxRollRate * gpsDataInterval;
    float rollDelta = rollTarget - ap.interceptRollCmd;
    rollDelta = constrainf(rollDelta, -maxRollDelta, maxRollDelta);
    ap.interceptRollCmd = constrainf(ap.interceptRollCmd + rollDelta,
                                     -(float)cfg->maxAngle, (float)cfg->maxAngle);

    // Pitch: anticipatory speed management (PD controller)
    // Monitors both speed error AND rate of change. When acceleration is pushing
    // toward maxVelocity, preemptively reduces pitch so the craft coasts smoothly
    // to the target speed without overshooting.
    const float targetSpeed = fmaxf(
        (float)cfg->maxVelocity * ap.navVelocityScale * ap.altitudeVelocityScale,
        (float)cfg->minForwardVelocity);
    const float speedError = targetSpeed - speedCmS;

    // Speed rate of change (acceleration): positive = speeding up
    float speedRate = 0.0f;
    if (ap.interceptInitialized && gpsDataInterval > 0.0f) {
        speedRate = (speedCmS - ap.previousSpeedCmS) / gpsDataInterval;
    }
    ap.previousSpeedCmS = speedCmS;

    // PID controller: P brings craft toward target speed, I compensates for
    // persistent headwind/tailwind, D damps the approach.
    // When accelerating toward max: speedRate > 0 → D reduces forward pitch,
    // causing the craft to coast toward the setpoint instead of slamming into it.
    const float speedKp = 0.01f;   // degrees per cm/s error
    const float speedKi = 0.001f;  // degrees per cm/s·s accumulated speed error
    const float speedKd = 0.005f;  // degrees per (cm/s)/s acceleration
    float pitchTarget = speedError * speedKp - speedRate * speedKd;

    // AP I-term: compensate for persistent headwind/tailwind.
    // Anti-windup: constrain to ±5 degrees of pitch contribution.
    {
        ap.interceptSpeedIntegral += speedError * speedKi * gpsDataInterval;
        ap.interceptSpeedIntegral = constrainf(ap.interceptSpeedIntegral, -5.0f, 5.0f);
        pitchTarget += ap.interceptSpeedIntegral;
    }

    // Limit pitch to buildup angle limit (gradual authority increase from stationary)
    pitchTarget = constrainf(pitchTarget, -(float)cfg->maxAngle, ap.buildupAngleLimit);

    // Rate limit pitch: same rate as roll for consistency
    const float maxPitchDelta = maxRollRate * gpsDataInterval;
    float pitchDelta = pitchTarget - ap.interceptPitchCmd;
    pitchDelta = constrainf(pitchDelta, -maxPitchDelta, maxPitchDelta);
    ap.interceptPitchCmd = constrainf(ap.interceptPitchCmd + pitchDelta,
                                      -(float)cfg->maxAngle, (float)cfg->maxAngle);

    // Debug capture for intercept-specific state
    apDebug.interceptTurnRate = ap.measuredTurnRate;
    apDebug.interceptHeadingError = headingError;

    // Convert body-frame (roll, pitch) to earth-frame (East, North).
    // Uses inverse yaw rotation so the existing 100Hz earth-to-body path
    // correctly reproduces the intended body-frame angles, and naturally
    // handles yaw changes between GPS updates.
    const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    const float sinYaw = sin_approx(yawRad);
    const float cosYaw = cos_approx(yawRad);

    // Inverse rotation: body → earth
    ap.pidSumEF.v[EAST]  =  ap.interceptRollCmd * cosYaw + ap.interceptPitchCmd * sinYaw;
    ap.pidSumEF.v[NORTH] = -ap.interceptRollCmd * sinYaw + ap.interceptPitchCmd * cosYaw;

    // Decay position PID integrals to prevent windup while intercept is active.
    // When L1 deactivates and PID takes over, stale integrals would cause a spike.
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        ap.efAxis[i].integral *= 0.95f;
        ap.efAxis[i].velocityIntegral *= 0.95f;
    }
}
#endif // ENABLE_FLIGHT_PLAN

bool positionControl(void)
{
    unsigned debugAxis = gyroConfig()->gyro_filter_debug_axis;
    static vector2_t debugGpsDistance = { 0 };     // keep last calculated distance for DEBUG
    static vector2_t debugPidSumEF = { 0 };        // and last pidsum in EF
    static uint16_t gpsStamp = 0;
    if (gpsHasNewData(&gpsStamp)) {
        const float gpsDataInterval = getGpsDataIntervalSeconds(); // interval for current GPS data value 0.05 - 2.5s
        const float gpsDataFreq = getGpsDataFrequencyHz();
        apDebug.gpsDataIntervalS = gpsDataInterval;

        // Update rolling origin (resets every 500m for precision)
        navOriginUpdate(&gpsSol.llh);

        // Convert current and target positions to NED coordinates using rolling origin
        vector3_t currentNED, targetNED;
        navOriginLLHtoNED(&gpsSol.llh, &currentNED);
        navOriginLLHtoNED(&ap.targetLocation, &targetNED);

        // Extract 2D position (North, East) from 3D NED
        vector2_t currentPos2D = { .x = currentNED.x, .y = currentNED.y };  // North, East
        vector2_t targetPos2D = { .x = targetNED.x, .y = targetNED.y };     // North, East

#if ENABLE_FLIGHT_PLAN
        // Deferred L1 activation: waypointUpdate() may have skipped L1 setup
        // because navOrigin wasn't valid yet. Now that navOriginUpdate() has
        // been called, the origin is guaranteed valid and NED conversion is accurate.
        {
            const autopilotConfig_t *l1cfg = autopilotConfig();
            if (l1cfg->l1Enable && !l1GuidanceIsActive()
                && waypointGetState() == WP_STATE_APPROACHING) {
                const flightPlanConfig_t *plan = flightPlanConfig();
                if (plan->waypointCount > 1) {
                    vector2_t pathStart = { .x = currentPos2D.x, .y = currentPos2D.y };
                    vector2_t pathEnd = { .x = targetPos2D.x, .y = targetPos2D.y };
                    // No arc start here — this is a re-activation for the same
                    // waypoint (L1 deactivated near the endpoint).  Arc transitions
                    // are only set during actual waypoint transitions.
                    l1GuidanceSetPath(&pathStart, &pathEnd);
                    l1GuidanceSetActive(true);
                }
            }
        }
#endif

        // ** Sanity check ** against actual target (before L1 override)
        // Use distance to real waypoint target, not the L1 carrot point,
        // to detect flyaway even when L1 produces a nearby carrot.
        {
            vector2_t realDistance;
            realDistance.x = targetPos2D.y - currentPos2D.y;  // East error
            realDistance.y = targetPos2D.x - currentPos2D.x;  // North error
            const float realDistNormCm = vector2Norm(&realDistance);
            if (realDistNormCm > ap.sanityCheckDistance) {
                return false;
            }
        }

#if ENABLE_FLIGHT_PLAN
        // L1 Nonlinear Guidance - use carrot point if active
        const autopilotConfig_t *cfg = autopilotConfig();
        if (cfg->l1Enable && l1GuidanceIsActive()) {
            // Update L1 with current position and groundspeed
            l1GuidanceUpdate(&currentPos2D, gpsSol.groundSpeed, gpsDataInterval);

            // Use carrot point as target (if L1 is still active after update)
            if (l1GuidanceIsActive()) {
                const vector2_t *carrot = l1GuidanceGetCarrot();
                targetPos2D = *carrot;
            }
        }

        // Debug output for navigation (rolling origin + L1 guidance)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 0, lrintf(navOriginGetDistanceCm() / 100.0f));  // Origin distance (m)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 1, lrintf(l1GuidanceGetCrossTrackError()));     // XTE (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 2, lrintf(l1GuidanceGetAlongTrackDistance())); // Along-track (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 3, lrintf(l1GuidanceGetLookaheadDistance()));  // Lookahead (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 4, lrintf(currentPos2D.x));                    // Current North (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 5, lrintf(currentPos2D.y));                    // Current East (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 6, lrintf(targetPos2D.x));                     // Target/Carrot North (cm)
        DEBUG_SET(DEBUG_AUTOPILOT_NAVIGATION, 7, lrintf(targetPos2D.y));                     // Target/Carrot East (cm)

        // Track intercept: heading-based controller for wing platforms.
        // Replaces the position PID with bank-to-turn heading changes.
        // Multirotors use the position PID targeting the L1 carrot point instead;
        // the PID naturally handles deceleration, turning, and re-acceleration.
        {
            const bool isWingPlatform = (cfg->yawMode == YAW_MODE_DAMPENER);
            const bool isIntercept = (cfg->l1Enable && l1GuidanceIsActive() && isWingPlatform);

            if (isIntercept && !ap.wasInterceptActive) {
                // Intercept just activated: reset for gentle start
                ap.interceptRollCmd = 0.0f;
                ap.interceptPitchCmd = 0.0f;
                ap.interceptHeadingIntegral = 0.0f;
                ap.interceptSpeedIntegral = 0.0f;
                ap.interceptInitialized = false;
            } else if (!isIntercept && ap.wasInterceptActive) {
                // Intercept just deactivated: reset PID for clean handoff
                for (unsigned i = 0; i < ARRAYLEN(ap.efAxis); i++) {
                    ap.efAxis[i].needsInit = true;
                    ap.efAxis[i].integral = 0.0f;
                    ap.efAxis[i].velocityIntegral = 0.0f;
                }
            }
            ap.wasInterceptActive = isIntercept;

            if (isIntercept) {
                trackInterceptCompute(gpsDataInterval);
                debugPidSumEF = ap.pidSumEF;
                apDebug.pidSumEast = ap.pidSumEF.v[EAST];
                apDebug.pidSumNorth = ap.pidSumEF.v[NORTH];
                if (ap.sticksActive) {
                    ap.pidSumEF = (vector2_t){{0, 0}};
                    ap.targetLocation = gpsSol.llh;
                    ap.sanityCheckDistance = sanityCheckDistance(gpsSol.groundSpeed);
                }
            }
        }
#endif

        if (!
#if ENABLE_FLIGHT_PLAN
            ap.wasInterceptActive
#else
            false
#endif
        ) {
        // Calculate position error in NED frame (uses L1 carrot when active)
        // NED 3D vector: North=X, East=Y, Down=Z
        // PID controller 2D array: efAxis[EAST]=index 0, efAxis[NORTH]=index 1
        vector2_t gpsDistance;
        gpsDistance.x = targetPos2D.y - currentPos2D.y;  // East error → efAxis[EAST]
        gpsDistance.y = targetPos2D.x - currentPos2D.x;  // North error → efAxis[NORTH]
        debugGpsDistance = gpsDistance;
        const float distanceNormCm = vector2Norm(&gpsDistance);

        // update filters according to current GPS update rate
        const float vaGain = pt1FilterGain(ap.vaLpfCutoff, gpsDataInterval);
        const float iTermLeakGain = 1.0f - pt1FilterGainFromDelay(2.5f, gpsDataInterval);   // 2.5s time constant
        vector2_t pidSum = { 0 };       // P+I in loop, D+A added after the axis loop (after limiting it)
        vector2_t pidDA;                // D+A

        // Pre-compute velocity setpoints for both axes, then limit the
        // vector magnitude so diagonal flight doesn't target above maxVelocity.
        // Also applies per-axis braking distance limits.
        vector2_t velSetpoint = { 0 };
        if (autopilotConfig()->velocityControlEnable) {
            const autopilotConfig_t *cfg = autopilotConfig();
            const float posToVelGain = cfg->positionP * 0.1f;
            for (axisEF_e i = EAST; i <= NORTH; i++) {
                velSetpoint.v[i] = gpsDistance.v[i] * posToVelGain;
            }

            // Limit combined velocity setpoint magnitude to cruise speed.
            // Both glide slope (navVelocityScale) and altitude deficit (altitudeVelocityScale)
            // reduce cruise speed when below the desired altitude profile.
            // Floor at minForwardVelocity: speed is never reduced below this regardless
            // of altitude deficit (GPS course reliability / stall prevention).
            const float maxVel = fmaxf((float)cfg->maxVelocity * ap.navVelocityScale * ap.altitudeVelocityScale,
                                       (float)cfg->minForwardVelocity);
            const float velMag = vector2Norm(&velSetpoint);
            if (velMag > maxVel && velMag > 0.0f) {
                vector2Scale(&velSetpoint, &velSetpoint, maxVel / velMag);
            }

            // Per-axis braking distance limit: v_max = sqrt(2 * a_max * distance)
            // Uses half the theoretical max deceleration for PID headroom
            const float maxDecel = 980.0f * sin_approx((float)cfg->maxAngle * (M_PIf / 180.0f)) * 0.5f;
            for (axisEF_e i = EAST; i <= NORTH; i++) {
                const float brakingLimit = sqrtf(2.0f * maxDecel * fabsf(gpsDistance.v[i]));
                velSetpoint.v[i] = constrainf(velSetpoint.v[i], -brakingLimit, brakingLimit);
            }

            // Velocity buildup: scale velocity target to match available angle authority.
            // Without this, the velocity PID integral winds up against the body-frame
            // angle clamp, then overshoots when the buildup limit relaxes.
            if (ap.buildupAngleLimit < ap.maxAngle) {
                const float buildupRatio = ap.buildupAngleLimit / fmaxf(ap.maxAngle, 1.0f);
                vector2Scale(&velSetpoint, &velSetpoint, buildupRatio);
            }
        }

        for (axisEF_e efAxisIdx = EAST; efAxisIdx <= NORTH; efAxisIdx++) {
            efPidAxis_t *efAxis = &ap.efAxis[efAxisIdx];
            const autopilotConfig_t *cfg = autopilotConfig();
            // separate PID controllers for longitude (EastWest or EW, X) and latitude (NorthSouth or NS, Y)
            const float axisDistance = gpsDistance.v[efAxisIdx];

            // Seed previousDistance on first sample to avoid a huge velocity spike
            if (efAxis->needsInit) {
                efAxis->previousDistance = axisDistance;
                efAxis->needsInit = false;
            }

            // Calculate current velocity, clamped to prevent spikes from L1 target changes or GPS glitches
            const float maxVel = autopilotConfig()->maxVelocity * 2.0f;
            const float velocity = constrainf((axisDistance - efAxis->previousDistance) * gpsDataFreq, -maxVel, maxVel); // cm/s
            efAxis->previousDistance = axisDistance;
            pt1FilterUpdateCutoff(&efAxis->velocityLpf, vaGain);
            const float velocityFiltered = pt1FilterApply(&efAxis->velocityLpf, velocity);

            float pidP = 0.0f, pidI = 0.0f, pidD = 0.0f, pidA = 0.0f;

            if (cfg->velocityControlEnable) {
                // ** Cascaded velocity controller with drag compensation **
                // velocityFiltered is dError/dt: NEGATIVE when approaching the target
                // (error decreases as drone approaches). The velocity controller expects
                // positive = approaching, so negate it. This also corrects drag
                // compensation direction: positive velocity → lean forward to maintain speed.
                // The legacy D-term (below) correctly uses dError/dt directly as damping.
                const float approachVelocity = -velocityFiltered;
                const float angleOutput = velocityToAngle(efAxis, velSetpoint.v[efAxisIdx], approachVelocity, gpsDataInterval, cfg);
                pidSum.v[efAxisIdx] = angleOutput;
                // No separate D and A terms in velocity mode
                pidP = angleOutput;  // For debug display
                // Capture debug state per axis
                if (efAxisIdx == EAST) {
                    apDebug.velSetpointEast = efAxis->velocitySetpoint;
                    apDebug.currentVelEast = approachVelocity;
                    apDebug.dragCompEast = lastDragCompensation;
                } else {
                    apDebug.velSetpointNorth = efAxis->velocitySetpoint;
                    apDebug.currentVelNorth = approachVelocity;
                    apDebug.dragCompNorth = lastDragCompensation;
                }
            } else {
                // ** LEGACY: Direct position→angle PID controller **
                // ** P **
                pidP = axisDistance * positionPidCoeffs.Kp;
                pidSum.v[efAxisIdx] += pidP;

                // ** I **
                // only add to iTerm while in hold phase
                efAxis->integral += efAxis->isStopping ? 0.0f : axisDistance * gpsDataInterval;
                pidI = efAxis->integral * positionPidCoeffs.Ki;
                pidSum.v[efAxisIdx] += pidI;

                // ** D **
                pidD = velocityFiltered * positionPidCoeffs.Kd;

                // ** A (acceleration feedforward) **
                float acceleration = (velocityFiltered - efAxis->previousVelocity) * gpsDataFreq;
                efAxis->previousVelocity = velocityFiltered;
                pt1FilterUpdateCutoff(&efAxis->accelerationLpf, vaGain);
                const float accelerationFiltered = pt1FilterApply(&efAxis->accelerationLpf, acceleration);
                pidA = accelerationFiltered * positionPidCoeffs.Kf;
            }

            if (ap.sticksActive) {
                // sticks active 'phase', prepare to enter stopping
                efAxis->isStopping = true;
                // slowly leak iTerm away (both position and velocity integrals)
                efAxis->integral *= iTermLeakGain;
                efAxis->velocityIntegral *= iTermLeakGain;
                efAxis->previousDistance = 0.0f; // avoid D and A spikes
                efAxis->previousVelocityError = 0.0f;
                // rest is handled after axis loop
            } else if (efAxis->isStopping) {
                // 'phase' after sticks are centered, but before craft has stopped; in given Earth axis
                if (!cfg->velocityControlEnable) {
                    pidD *= 1.6f; // arbitrary D boost to stop more quickly (legacy mode only)
                }
                // detect when axis has nearly stopped by sign reversal of velocity
                if (velocity * velocityFiltered < 0.0f) {
                    setTargetLocationByAxis(&gpsSol.llh, efAxisIdx);  // reset target location for this axis, forcing P to zero
                    efAxis->previousDistance = 0.0f;                  // ensure minimal D jump from the updated location
                    efAxis->previousVelocityError = 0.0f;
                    efAxis->isStopping = false;                       // end the 'stopping' phase
                    if (ap.efAxis[NORTH].isStopping == ap.efAxis[EAST].isStopping) {
                        // when both axes have stopped moving, reset the sanity distance to 10m default
                        ap.sanityCheckDistance = sanityCheckDistance(1000);
                    }
                }
            }
            pidDA.v[efAxisIdx] = pidD + pidA;    // save DA here, processed after axis loop (legacy mode only)
            if (debugAxis == efAxisIdx) {
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceNormCm));   // same for both axes
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 4, lrintf(pidP * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 5, lrintf(pidI * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 6, lrintf(pidD * 10));
                DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 7, lrintf(pidA * 10));
            }
        } // end for loop

        // D+A limiting (legacy mode only)
        if (!autopilotConfig()->velocityControlEnable) {
            // limit sum of D and A per axis based on total DA vector length, otherwise can be too aggressive when starting at speed
            // limit is 35 degrees from D and A alone, arbitrary value.  20 is a bit too low, allows a lot of overshoot
            // note: an angle of more than 35 degrees can still be achieved as P and I grow
            const float maxDAAngle = 35.0f; // D+A limit in degrees; arbitrary angle
            const float mag = vector2Norm(&pidDA);
            if (mag > maxDAAngle) {
                vector2Scale(&pidDA, &pidDA, maxDAAngle / mag);
            }
        }

        // add constrained DA to sum (legacy mode only)
        if (!autopilotConfig()->velocityControlEnable) {
            vector2Add(&pidSum, &pidSum, &pidDA);
        }
        debugPidSumEF = pidSum;
        apDebug.pidSumEast = pidSum.v[EAST];
        apDebug.pidSumNorth = pidSum.v[NORTH];
        if (ap.sticksActive) {
            // if a Position Hold deadband is set, and sticks are outside deadband, allow pilot control in angle mode
            ap.pidSumEF = (vector2_t){{0, 0}};  // set output PIDs to 0; upsampling filter will smooth this
            // reset target location each cycle (and set previousDistance to zero in for loop), to keep D current, and avoid a spike when stopping
            ap.targetLocation = gpsSol.llh;
            // keep updating sanity check distance while sticks are out because speed may get high
            ap.sanityCheckDistance = sanityCheckDistance(gpsSol.groundSpeed);
        } else {
            // Store earth-frame PID sum for upsampling.
            // Body-frame rotation is deferred to the 100Hz output below,
            // using the current yaw each cycle. This avoids stale body-frame
            // values when the drone yaws between 10Hz GPS updates.
            ap.pidSumEF = pidSum;
        }
        } // end if (!interceptActive) — position PID block

        // Prime upsampling filters on first PID computation after mode activation
        // or climb→nav transition, so corrections take effect immediately.
        // Must be inside the GPS block to ensure we prime with a fresh PID output.
        if (ap.primeUpsampleFilters) {
            for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
                ap.upsampleLpfEF[i].state = ap.pidSumEF.v[i];
                ap.upsampleLpfEF[i].state1 = ap.pidSumEF.v[i];
                ap.upsampleLpfEF[i].state2 = ap.pidSumEF.v[i];
            }
            ap.primeUpsampleFilters = false;
        }
    }

    // Final output to pid.c Angle Mode at 100Hz with PT3 upsampling.
    // Upsample in earth frame, then rotate to body frame using current yaw.
    // This prevents navigation errors when the drone yaws between GPS updates.
    vector2_t upsampledEF;
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        upsampledEF.v[i] = pt3FilterApply(&ap.upsampleLpfEF[i], ap.pidSumEF.v[i]);
    }

    // Debug capture: upsampled EF values
    apDebug.upsampledEast = upsampledEF.v[EAST];
    apDebug.upsampledNorth = upsampledEF.v[NORTH];
    apDebug.sticksActive = ap.sticksActive;

    // Blackbox: velocity controller state (set debug_mode = AP_VEL)
    DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 0, lrintf(apDebug.currentVelEast));
    DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 1, lrintf(apDebug.currentVelNorth));
    DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 2, lrintf(apDebug.velSetpointEast));
    DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 3, lrintf(apDebug.velSetpointNorth));

    // Rotate earth frame to body frame using current heading
    const float yawAngle = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    vector2_t pidBodyFrame;
    vector2Rotate(&pidBodyFrame, &upsampledEF, yawAngle);

    // Map to roll/pitch
    vector2_t anglesBF;
    anglesBF.v[AI_ROLL] = pidBodyFrame.x;       // body right: positive roll for right bank
    anglesBF.v[AI_PITCH] = pidBodyFrame.y;      // body forward: positive pitch for nose down (forward flight)

    // Note: no pitchAlign scaling here. The velocity PID naturally handles turns
    // by generating both roll (to redirect) and pitch (to manage speed). Reducing
    // pitch based on heading error (bank-to-turn) causes instability: the coordinated
    // turn yaw controller accelerates yaw rotation during the bank, leading to
    // tumbles at high speed. Letting the PID decelerate-and-turn is more stable.
    apDebug.pitchAlign = 1.0f;

    // Limit combined magnitude
    const float bfMag = vector2Norm(&anglesBF);
    if (bfMag > ap.maxAngle && bfMag > 0.0f) {
        vector2Scale(&anglesBF, &anglesBF, ap.maxAngle / bfMag);
    }

    autopilotAngle[AI_ROLL] = anglesBF.v[AI_ROLL];
    autopilotAngle[AI_PITCH] = anglesBF.v[AI_PITCH];

    // Debug capture (body frame, after rotation + limiting)
    apDebug.yawHeading = attitude.values.yaw;
    apDebug.pidSumBFRoll = anglesBF.v[AI_ROLL];
    apDebug.pidSumBFPitch = anglesBF.v[AI_PITCH];

    if (debugAxis < 2) {
        // this is different from @ctzsnooze version
        // debugAxis = 0: store longitude + roll
        // debugAxis = 1: store latitude + pitch
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(debugGpsDistance.v[debugAxis]));    // cm
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(debugPidSumEF.v[debugAxis] * 10));  // deg * 10
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[debugAxis] * 10));   // deg * 10
    }
    return true;
}

bool isBelowLandingAltitude(void)
{
    return getAltitudeCm() < 100.0f * autopilotConfig()->landingAltitudeM;
}

float getAutopilotThrottle(void)
{
    return throttleOut;
}

bool isAutopilotInControl(void)
{
    return !ap.sticksActive;
}

#if ENABLE_FLIGHT_PLAN

// Multirotor yaw: re-orient nose to direction of travel.
//
// Without a magnetometer, GPS ground course is the ONLY heading reference.
// GPS course is only valid above minForwardVelocity — at lower speeds the
// course reading is dominated by noise. Therefore:
//
// 1. Below minForwardVelocity: zero yaw rate. The craft must first establish
//    forward velocity (via pitch) before any heading change is meaningful.
// 2. Above minForwardVelocity: align nose to GPS ground course. As the craft
//    turns (via roll), the ground course changes, and yaw tracks it so the
//    nose follows the new direction of travel.
//
// The authority ramp from 0 to 1 across minForwardVelocity ensures no yaw
// command is issued until GPS course is reliable.
static void calculateYawControlMultirotor(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    const float minVel = (float)cfg->minForwardVelocity;
    const float yawPGain = cfg->yawP * 0.01f;  // 50 → 0.50 deg/s per deg error

    // GPS course authority: zero below minForwardVelocity, ramps to full above it.
    // Without a magnetometer, GPS course is meaningless at low speed.
    const float courseAuth = constrainf(
        ((float)gpsSol.groundSpeed - minVel * 0.5f) / fmaxf(minVel * 0.5f, 1.0f),
        0.0f, 1.0f);

    if (courseAuth < 0.01f) {
        // Too slow for reliable GPS course — no yaw command
        ap.desiredYawRate = pt3FilterApply(&ap.yawRateFilter, 0.0f);
        return;
    }

    // Target heading: GPS ground course (direction of travel)
    const float groundCourseDeg = gpsSol.groundCourse * 0.1f;
    const float currentHeadingDeg = attitude.values.yaw * 0.1f;

    float headingError = groundCourseDeg - currentHeadingDeg;
    if (headingError > 180.0f) headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;

    // Scale by GPS course authority — gentle onset as speed crosses threshold
    const float maxHoldRate = (float)cfg->maxYawRate * 0.5f;
    const float rawYawRate = constrainf(headingError * yawPGain * courseAuth,
                                        -maxHoldRate, maxHoldRate);

    ap.desiredYawRate = pt3FilterApply(&ap.yawRateFilter, rawYawRate);
}

// Wing yaw: coordinated turn damper.
//
// Yaw never steers — rudder only prevents side-slip during banked flight.
// Two components:
// 1. Coordinated turn: yaw_rate = g * tan(bank) / V
//    Keeps the nose aligned with the flight path during steady banked turns.
// 2. Adverse yaw damping: yaw proportional to roll rate
//    Counteracts the yaw moment caused by differential lift when roll changes.
//    Without this, entering or exiting a bank causes the nose to swing opposite
//    to the turn direction (adverse yaw), leading to uncoordinated flight.
static void calculateYawControlWing(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    const float yawDGain = cfg->yawD * 0.01f;  // 10 → 0.10 deg/s per deg/s roll rate

    // --- Coordinated turn: steady-state sideslip prevention ---
    float coordTurnRate = 0.0f;
    if (gpsSol.groundSpeed > 100) {  // Need some airspeed for meaningful coordination
        const float rollRad = autopilotAngle[AI_ROLL] * (M_PIf / 180.0f);
        const float cosRoll = cos_approx(rollRad);
        const float tanRoll = (fabsf(cosRoll) > 0.1f) ? sin_approx(rollRad) / cosRoll : 0.0f;
        const float gravityCmS2 = 980.0f;
        // Standard coordinated turn: yaw_rate = g * tan(bank) / V
        coordTurnRate = (gravityCmS2 * tanRoll / (float)gpsSol.groundSpeed) * (180.0f / M_PIf);
    }

    // --- Adverse yaw damping: counteract yaw from roll rate changes ---
    // When roll is changing (entering or exiting a bank), differential lift
    // creates a yaw moment opposite to the roll direction. Apply rudder
    // proportional to the roll rate to cancel this effect.
    // Uses the derivative of the roll command as a proxy for actual roll rate.
    static float previousRollCmd = 0.0f;
    const float rollRate = (autopilotAngle[AI_ROLL] - previousRollCmd) * 100.0f;  // 100Hz → deg/s
    previousRollCmd = autopilotAngle[AI_ROLL];
    const float adverseYawComp = rollRate * yawDGain;

    const float rawYawRate = constrainf(coordTurnRate + adverseYawComp,
                                        -(float)cfg->maxYawRate, (float)cfg->maxYawRate);

    ap.desiredYawRate = pt3FilterApply(&ap.yawRateFilter, rawYawRate);
}

// Dispatch yaw control to platform-specific implementation.
// YAW_MODE_DAMPENER selects wing control (coordinated turns + adverse yaw damping).
// All other yaw modes select multirotor control (GPS-course-based yaw rotation).
static void calculateYawControl(void)
{
    if (autopilotConfig()->yawMode == YAW_MODE_DAMPENER || isFixedWing()) {
        calculateYawControlWing();
    } else {
        calculateYawControlMultirotor();
    }
}

// Getter function for rc.c integration
float autopilotGetYawRate(void)
{
    return ap.desiredYawRate;
}

// Detect pilot override on any axis
static void detectPilotOverride(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // Check stick deflection against deadband
    // rcCommandFromReceiver[] contains raw receiver data (bypasses autopilot)
    // This is updated by updateRcCommands() before autopilot task runs
    // Roll/Pitch/Yaw: ±500 range
    // Throttle: 1000-2000 range

    const float stickDeadband = cfg->stickDeadband;

    // Roll, Pitch, Yaw: Check deflection from center position (0)
    ap.pilotOverrideRoll = (fabsf(rcCommandFromReceiver[ROLL]) > stickDeadband);
    ap.pilotOverridePitch = (fabsf(rcCommandFromReceiver[PITCH]) > stickDeadband);
    ap.pilotOverrideYaw = (fabsf(rcCommandFromReceiver[YAW]) > stickDeadband);

    // Throttle: Override only if pilot throttle exceeds autopilot throttle command
    const float minThrottle = MAX(rxConfig()->mincheck, PWM_RANGE_MIN);
    const float autopilotThrottlePWM = minThrottle + getAutopilotThrottle() * (PWM_RANGE_MAX - minThrottle);
    ap.pilotOverrideThrottle = (rcCommandFromReceiver[THROTTLE] > autopilotThrottlePWM);

    // Update stick status for position controller
    const bool sticksActive = ap.pilotOverrideRoll || ap.pilotOverridePitch;
    setSticksActiveStatus(sticksActive);
}

// Generate RC commands from autopilot outputs
static void generateRcCommands(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // CRITICAL: Fuse pilot and autopilot commands
    // If pilot is overriding an axis, use receiver data
    // Otherwise, use autopilot command

    // Roll and Pitch: Convert angles (degrees) to RC range (±500)
    // autopilotAngle[] is in degrees, maxAngle is in degrees
    // Note: PID angle mode reads autopilotAngle directly; rcCommand is set here
    // for pilot override blending and telemetry consistency
    if (!ap.pilotOverrideRoll) {
        rcCommand[ROLL] = constrainf(autopilotAngle[ROLL] * 500.0f / cfg->maxAngle, -500.0f, 500.0f);
    } else {
        rcCommand[ROLL] = rcCommandFromReceiver[ROLL];
    }

    if (!ap.pilotOverridePitch) {
        rcCommand[PITCH] = constrainf(autopilotAngle[PITCH] * 500.0f / cfg->maxAngle, -500.0f, 500.0f);
    } else {
        rcCommand[PITCH] = rcCommandFromReceiver[PITCH];
    }

    // Yaw: Convert rate (deg/s) to RC range (±500)
    if (!ap.pilotOverrideYaw) {
        rcCommand[YAW] = constrainf(ap.desiredYawRate * 500.0f / cfg->maxYawRate, -500.0f, 500.0f);
    } else {
        rcCommand[YAW] = rcCommandFromReceiver[YAW];
    }

    // Throttle: Convert 0.0-1.0 back to PWM using the same range as altitudeControl()
    // altitudeControl() scales [mincheck, PWM_MAX] → [0, 1], so invert with matching range
    if (!ap.pilotOverrideThrottle) {
        float throttle = getAutopilotThrottle();
        const float minThrottle = MAX(rxConfig()->mincheck, PWM_RANGE_MIN);
        rcCommand[THROTTLE] = minThrottle + throttle * (PWM_RANGE_MAX - minThrottle);
        rcCommand[THROTTLE] = constrainf(rcCommand[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);
    } else {
        rcCommand[THROTTLE] = rcCommandFromReceiver[THROTTLE];
    }
}

// Main autopilot update function, called at 100Hz from tasks.c.
// Position PID runs at GPS rate (5-10Hz) via gpsHasNewData() guard in positionControl(),
// then PT3 upsamples the output to 100Hz for smooth angle mode control.
// Altitude, yaw, orbit patterns, and RC generation run every cycle.
void updateAutopilot(void)
{
    // 0. Defensive check: should not be called if mode is not active
    if (!FLIGHT_MODE(AUTOPILOT_MODE)) {
        return;
    }

    // 1. Update waypoint state machine
    waypointUpdate(micros());

    // 2. Safety: check if waypoint system is valid
    if (!waypointIsSystemValid()) {
        DISABLE_FLIGHT_MODE(AUTOPILOT_MODE);
        return;
    }

    // 3. Get current target from waypoint tracker
    const gpsLocation_t *target = waypointGetTarget();

    // 4. Climb-then-navigate: hold position (multirotor) or orbit (wing) while
    //    below minNavAlt, then navigate. This prevents flyaway during initial
    //    climb by keeping the craft near the launch point until safe altitude.
    {
        const autopilotConfig_t *cfg = autopilotConfig();
        const float currentAltCm = getAltitudeCm();
        const float minNavAltCm = cfg->minNavAltitudeM * 100.0f;
        const bool isLandingOrComplete = (waypointGetState() == WP_STATE_LANDING
                                          || waypointGetState() == WP_STATE_COMPLETE);
        const bool belowMinNavAlt = (minNavAltCm > 0.0f && currentAltCm < minNavAltCm
                                     && !isLandingOrComplete);

        // Climb phase entry (GPS-gated — only update orbit/sanity on fresh data)
        static uint16_t gpsStamp = 0;
        if (belowMinNavAlt && !ap.climbComplete) {
            if (!ap.climbPhaseActive) {
                // Entering climb phase — capture hold position for wing orbit
                ap.holdLocation = gpsSol.llh;
                ap.climbPhaseActive = true;
                ap.climbOrbitAngle = 0.0f;
            }
            if (gpsHasNewData(&gpsStamp)) {
                if (autopilotConfig()->yawMode == YAW_MODE_DAMPENER || isFixedWing()) {
                    // Wing: climbing orbit — maintain forward velocity above stall speed
                    ap.climbOrbitAngle += cfg->maxTurnRate * 0.01f;
                    if (ap.climbOrbitAngle >= 360.0f) {
                        ap.climbOrbitAngle -= 360.0f;
                    }
                    calculateOrbitPosition(ap.holdLocation.lat, ap.holdLocation.lon,
                                           cfg->holdOrbitRadius, ap.climbOrbitAngle,
                                           &ap.targetLocation);
                }
                ap.sanityCheckDistance = sanityCheckDistance(gpsSol.groundSpeed);
            }
        } else if (ap.climbPhaseActive) {
            // Crossed above minNavAlt — transition to waypoint navigation.
            // This check runs at 100Hz (not GPS-gated) so the transition fires
            // as soon as altitude crosses the threshold.
            ap.climbPhaseActive = false;
            ap.climbComplete = true;  // prevent re-entering climb if altitude dips briefly
            waypointSetState(WP_STATE_APPROACHING);
            ap.targetLocation = *target;
            // Reset position controller state for clean transition.
            for (unsigned i = 0; i < ARRAYLEN(ap.efAxis); i++) {
                ap.efAxis[i].needsInit = true;
                ap.efAxis[i].velocityIntegral = 0.0f;
                ap.efAxis[i].integral = 0.0f;
                ap.efAxis[i].velocityLpf.state = 0.0f;
                ap.efAxis[i].accelerationLpf.state = 0.0f;
                ap.efAxis[i].previousVelocity = 0.0f;
                ap.efAxis[i].previousVelocityError = 0.0f;
            }
            // Let upsampling filters ramp from zero — do NOT prime them.
            resetUpsampleFilters();
            // Prime smoothing filters for clean transition
            primeSmoothingFilters(0.0f, throttleOut);
            // Set sanity distance based on new waypoint target
            uint32_t distToNewTarget = 0;
            GPS_distance_cm_bearing(&gpsSol.llh, target, false, &distToNewTarget, NULL);
            ap.sanityCheckDistance = fmaxf(sanityCheckDistance(gpsSol.groundSpeed),
                                           distToNewTarget * 2.0f + gpsSol.groundSpeed * 5.0f);
            ap.glideSlopeActive = false;
        } else if (gpsHasNewData(&gpsStamp)) {
            // Normal navigation — update target from waypoint tracker
            if (ap.targetLocation.lat != target->lat || ap.targetLocation.lon != target->lon) {
                uint32_t distToNewTarget = 0;
                GPS_distance_cm_bearing(&gpsSol.llh, target, false, &distToNewTarget, NULL);
                ap.sanityCheckDistance = fmaxf(sanityCheckDistance(gpsSol.groundSpeed),
                                               distToNewTarget * 2.0f + gpsSol.groundSpeed * 5.0f);
                ap.glideSlopeActive = false;
            }
            ap.targetLocation = *target;
        }
    }

    // 4a. Safety: abort if drifting too fast during climb phase
    // During pos-hold/orbit climb, excessive speed indicates the hold has failed
    // (e.g., strong wind, sensor issue). Abort before the craft flies away.
    if (ap.climbPhaseActive) {
        const uint16_t maxClimbPhaseSpeed = autopilotConfig()->maxVelocity / 2;  // half cruise speed
        if (gpsSol.groundSpeed > maxClimbPhaseSpeed) {
            DISABLE_FLIGHT_MODE(AUTOPILOT_MODE);
            return;
        }
    }

    // 4b. Compute glide slope velocity limiting
    // Limits the navigation cruise speed when below the expected altitude profile.
    // This ensures altitude requirements are met (e.g. terrain clearance)
    // without sacrificing altitude for speed.
    {
        const autopilotConfig_t *cfg = autopilotConfig();
        const float currentAltCm = getAltitudeCm();
        const float minNavAltCm = cfg->minNavAltitudeM * 100.0f;
        const float targetAltCmRel = target->altCm - GPS_home_llh.altCm;
        const float currentDist = (float)waypointGetDistanceCm();

        // Activate glide slope once the craft reaches minNavAlt
        if (!ap.glideSlopeActive && currentAltCm >= minNavAltCm
            && waypointGetState() != WP_STATE_LANDING
            && waypointGetState() != WP_STATE_COMPLETE
            && !ap.climbPhaseActive) {
            ap.glideSlopeStartDist = currentDist;
            ap.glideSlopeStartAlt = currentAltCm;
            ap.glideSlopeActive = true;
        }

        // Re-establish slope if craft has drifted further from waypoint (overshoot / wind)
        if (ap.glideSlopeActive && currentDist > ap.glideSlopeStartDist * 1.1f) {
            ap.glideSlopeStartDist = currentDist;
            ap.glideSlopeStartAlt = currentAltCm;
        }

        // Disable during landing / complete / climb phase
        if (waypointGetState() == WP_STATE_LANDING || waypointGetState() == WP_STATE_COMPLETE
            || ap.climbPhaseActive) {
            ap.glideSlopeActive = false;
        }

        if (ap.glideSlopeActive && ap.glideSlopeStartDist > 100.0f) {
            // Linear interpolation: expected altitude at current distance
            const float progress = 1.0f - constrainf(currentDist / ap.glideSlopeStartDist, 0.0f, 1.0f);
            const float expectedAlt = ap.glideSlopeStartAlt
                + (targetAltCmRel - ap.glideSlopeStartAlt) * progress;
            const float deviation = currentAltCm - expectedAlt; // +above, -below

            // Scale cruise speed: full when on/above slope, zero at 2m below
            const float tolerance = 200.0f; // cm
            ap.navVelocityScale = constrainf(1.0f + deviation / tolerance, 0.0f, 1.0f);

            apDebug.glideSlopeExpectedAlt = expectedAlt;
            apDebug.glideSlopeDeviation = deviation;
            apDebug.navVelocityScale = ap.navVelocityScale;
        } else {
            ap.navVelocityScale = 1.0f;
            apDebug.glideSlopeExpectedAlt = 0.0f;
            apDebug.glideSlopeDeviation = 0.0f;
            apDebug.navVelocityScale = 1.0f;
        }
    }

    // 5. Linear altitude track per leg
    // Interpolate altitude between previous and current waypoint altitudes based on
    // along-track progress. First leg interpolates from minNavAltitudeM to WP0 altitude,
    // subsequent legs from WP(n-1) altitude to WP(n) altitude.
    float targetAltCm;
    {
        const bool isLandingOrComplete = (waypointGetState() == WP_STATE_LANDING
                                          || waypointGetState() == WP_STATE_COMPLETE);
        if (ap.climbPhaseActive) {
            // During climb: target the minimum navigation altitude
            targetAltCm = autopilotConfig()->minNavAltitudeM * 100.0f;
        } else if (isLandingOrComplete) {
            // During landing/complete: use waypoint altitude directly
            targetAltCm = (float)(target->altCm - GPS_home_llh.altCm);
        } else {
            // Normal navigation: linear interpolation between waypoint altitudes
            const float prevAltRel = (float)(waypointGetPreviousAltCm() - GPS_home_llh.altCm);
            const float endAltRel = (float)(target->altCm - GPS_home_llh.altCm);
            float progress;
#if ENABLE_FLIGHT_PLAN
            if (autopilotConfig()->l1Enable && l1GuidanceIsActive() && l1GuidanceGetPathLength() > 0.0f) {
                progress = constrainf(l1GuidanceGetAlongTrackDistance() / l1GuidanceGetPathLength(), 0.0f, 1.0f);
            } else
#endif
            {
                const float legLen = waypointGetLegLengthCm();
                progress = (legLen > 0.0f) ? constrainf(1.0f - (float)waypointGetDistanceCm() / legLen, 0.0f, 1.0f) : 1.0f;
            }
            targetAltCm = prevAltRel + (endAltRel - prevAltRel) * progress;
        }
    }
    apDebug.targetAltCm = targetAltCm;

    // Ground track: bearing from activation point to current position (degrees, 0.1-360.0)
    {
        uint32_t groundTrackDist = 0;
        int32_t groundTrackBearing = 0;
        GPS_distance_cm_bearing(&ap.activationLocation, &gpsSol.llh, false, &groundTrackDist, &groundTrackBearing);
        float trackDeg = groundTrackBearing * 0.01f; // centidegrees → degrees
        if (trackDeg <= 0.0f) {
            trackDeg = 360.0f;
        }
        apDebug.groundTrack = (groundTrackDist > 100) ? trackDeg : 0.0f; // suppress noise when stationary
    }

    // 5a. Climb phase: level flight only. Altitude is the sole priority.
    // No position corrections — accept wind drift until minNavAlt is reached.
    // The waypoint state reflects CLIMBING so telemetry shows the correct phase.
    if (ap.climbPhaseActive) {
        waypointSetState(WP_STATE_CLIMBING);
        autopilotAngle[AI_ROLL] = 0.0f;
        autopilotAngle[AI_PITCH] = 0.0f;
        // Keep buildupAngleLimit primed at buildupMaxPitch for climb→nav transition
        const autopilotConfig_t *cfg = autopilotConfig();
        ap.buildupAngleLimit = (cfg->velocityControlEnable && cfg->velocityBuildupMaxPitch > 0)
            ? (float)cfg->velocityBuildupMaxPitch : ap.maxAngle;

        // Debug capture
        apDebug.vertPhase = 1;
        apDebug.navScale = 0.0f;
        apDebug.buildupAngleLimit = ap.buildupAngleLimit;
        apDebug.desiredBearing = waypointGetBearingCdeg() * 0.1f;
        apDebug.interceptBearing = 0.0f;
        apDebug.groundCourse = gpsSol.groundCourse;
        apDebug.l1Active = l1GuidanceIsActive();
        apDebug.l1PathLength = l1GuidanceGetPathLength();
        apDebug.l1DistToEnd = l1GuidanceGetPathLength() - l1GuidanceGetAlongTrackDistance();
        apDebug.navOriginValid = navOriginIsValid();
        DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 4, 0);
        DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 5, 0);
        DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 6, 0);
        DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 7, 0);
    } else {
        // 5b. Altitude-error velocity scaling (velocity control mode)
        // When off target altitude (above OR below), reduce the cruise speed so
        // the velocity PID properly tracks a lower setpoint — giving the altitude
        // controller authority to correct. Symmetric: prevents both diving for
        // speed AND re-accelerating after a zoom climb.
        {
            const float currentAltCm = getAltitudeCm();
            const float altError = fabsf(targetAltCm - currentAltCm);
            if (altError > 0.0f) {
                ap.altitudeVelocityScale = constrainf(1.0f - altError / 100.0f, 0.10f, 1.0f);
            } else {
                ap.altitudeVelocityScale = 1.0f;
            }
        }

        // 5c. Velocity buildup: altitude-gated ramp of angle authority.
        // After climb→nav transition, the angle limit ramps from buildupMaxPitch
        // to maxAngle, but ONLY when altitude is at or above target. If altitude
        // drops below target, the ramp freezes and reverses. This ensures the
        // craft finds the equilibrium pitch angle that can be sustained without
        // altitude loss, rather than blindly ramping into a dive.
        {
            const autopilotConfig_t *cfg = autopilotConfig();
            if (cfg->velocityControlEnable && cfg->velocityBuildupMaxPitch > 0) {
                const float buildupMax = (float)cfg->velocityBuildupMaxPitch;
                const float rampSeconds = 10.0f;
                const float rampRate = (ap.maxAngle - buildupMax) / rampSeconds; // deg/s
                const float dt = 0.01f;  // 100Hz task rate
                const float currentAltCm = getAltitudeCm();
                const float altDeficit = targetAltCm - currentAltCm;

                if (altDeficit <= 0.0f) {
                    // At or above target altitude: ramp up slowly
                    ap.buildupAngleLimit = fminf(ap.buildupAngleLimit + rampRate * dt, ap.maxAngle);
                } else {
                    // Below target altitude: reduce limit (2x ramp rate for faster recovery)
                    ap.buildupAngleLimit = fmaxf(ap.buildupAngleLimit - rampRate * 2.0f * dt, buildupMax);
                }
            } else {
                ap.buildupAngleLimit = ap.maxAngle;
            }
        }

        // 6. Run position controller (updates autopilotAngle[] array)
        if (!positionControl()) {
            // Flyaway detection triggered
            DISABLE_FLIGHT_MODE(AUTOPILOT_MODE);
            return;
        }

        // 6a. Velocity buildup: clamp body-frame angles to the current buildup limit.
        // positionControl() already clamps to maxAngle; this further restricts when
        // building speed from stationary.  The velocity setpoint scaling (inside
        // positionControl) should normally prevent the PID from exceeding this, but
        // this clamp acts as a safety net for transients and legacy mode.
        if (ap.buildupAngleLimit < ap.maxAngle) {
            const float bfMag = sqrtf(autopilotAngle[AI_ROLL] * autopilotAngle[AI_ROLL]
                                     + autopilotAngle[AI_PITCH] * autopilotAngle[AI_PITCH]);
            if (bfMag > ap.buildupAngleLimit && bfMag > 0.0f) {
                const float scale = ap.buildupAngleLimit / bfMag;
                autopilotAngle[AI_ROLL] *= scale;
                autopilotAngle[AI_PITCH] *= scale;
            }
        }

        // Altitude priority (PRIMAL LAW): reduce pitch/roll when off target altitude.
        // Symmetric: applies both BELOW target (prevents diving for speed) and
        // ABOVE target (prevents zoom-climb energy converting back to speed).
        // The raw navScale is PT1-filtered at 1Hz to prevent abrupt speed changes
        // that cause oscillation (altitude drops → angles slashed → speed drops →
        // altitude recovers → angles restored → speed jumps).
        {
            const float currentAltCm = getAltitudeCm();
            const float altError = fabsf(targetAltCm - currentAltCm);
            const float minNavScale = 3.5f / fmaxf(ap.maxAngle, 1.0f);
            const float rawNavScale = constrainf(1.0f - altError / 100.0f, minNavScale, 1.0f);
            float navScale = pt1FilterApply(&ap.navScaleFilter, rawNavScale);
            navScale = constrainf(navScale, minNavScale, 1.0f);
            autopilotAngle[AI_PITCH] *= navScale;
            const float rollScale = fminf(navScale * 2.0f, 1.0f);
            autopilotAngle[AI_ROLL] *= rollScale;

            // 6c. Soft overspeed limiter: smoothly reduce pitch authority as speed
            // approaches maxVelocity. Ramp starts at 80% of maxVel and reaches zero
            // at 120%. Roll retains 50% authority at full overspeed so the craft
            // can still steer while coasting for speed reduction.
            const uint16_t maxVel = autopilotConfig()->maxVelocity;
            if (maxVel > 0) {
                const float softStart = (float)maxVel * 0.8f;
                if ((float)gpsSol.groundSpeed > softStart) {
                    const float range = (float)maxVel * 0.4f; // 80% to 120%
                    const float speedScale = constrainf(1.0f - ((float)gpsSol.groundSpeed - softStart) / range, 0.0f, 1.0f);
                    autopilotAngle[AI_PITCH] *= speedScale;
                    // Retain 50% roll authority even at max overspeed for steering while coasting
                    const float rollSpeedScale = constrainf(speedScale + 0.5f, 0.0f, 1.0f);
                    autopilotAngle[AI_ROLL] *= rollSpeedScale;
                }
            }

            // Debug capture (after all limiters: buildup + navScale + overspeed)
            apDebug.vertPhase = 0;
            apDebug.navScale = navScale;
            apDebug.buildupAngleLimit = ap.buildupAngleLimit;
            apDebug.desiredBearing = waypointGetBearingCdeg() * 0.1f;
#if ENABLE_FLIGHT_PLAN
            apDebug.interceptBearing = l1GuidanceIsActive()
                ? l1GuidanceGetDesiredBearing() * 0.01f : 0.0f;
            apDebug.l1Active = l1GuidanceIsActive();
            apDebug.l1PathLength = l1GuidanceGetPathLength();
            apDebug.l1DistToEnd = l1GuidanceGetPathLength() - l1GuidanceGetAlongTrackDistance();
#else
            apDebug.interceptBearing = 0.0f;
            apDebug.l1Active = false;
            apDebug.l1PathLength = 0.0f;
            apDebug.l1DistToEnd = 0.0f;
#endif
            apDebug.groundCourse = gpsSol.groundCourse;
            apDebug.navOriginValid = navOriginIsValid();

            // Overwrite body-frame debug with FINAL values (after all limiters).
            // positionControl() captured pre-limiter values; these are the actual output.
            apDebug.pidSumBFRoll = autopilotAngle[AI_ROLL];
            apDebug.pidSumBFPitch = autopilotAngle[AI_PITCH];

            // Blackbox: body-frame output + scaling (slots 4-7 of AP_VEL)
            DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 4, lrintf(autopilotAngle[AI_ROLL] * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 5, lrintf(autopilotAngle[AI_PITCH] * 10));
            DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 6, lrintf(apDebug.pitchAlign * 1000));
            DEBUG_SET(DEBUG_AUTOPILOT_VELOCITY, 7, lrintf(navScale * 1000));
        }
    }

    // 7. Run altitude controller (updates internal throttle)
    altitudeControl(targetAltCm, 0.01f, 0.0f);

    // 7a. Translational lift throttle coast: when at or above target altitude AND
    // forward velocity requirement is met, reduce throttle to account for the
    // additional lift generated by forward flight. At speed, rotors are more
    // efficient and the craft needs less throttle to maintain altitude.
    // The altitude PID I-term naturally adapts to the reduced baseline.
    if (!ap.climbPhaseActive) {
        const autopilotConfig_t *cfg = autopilotConfig();
        const float currentAltCm = getAltitudeCm();
        if (currentAltCm >= targetAltCm && gpsSol.groundSpeed >= cfg->minForwardVelocity) {
            const float speedRatio = constrainf(
                ((float)gpsSol.groundSpeed - (float)cfg->minForwardVelocity) /
                fmaxf((float)cfg->maxVelocity - (float)cfg->minForwardVelocity, 1.0f),
                0.0f, 1.0f);
            // Up to 5% throttle reduction at max velocity
            throttleOut *= (1.0f - speedRatio * 0.05f);
        }
    }

    // 7b. Smooth throttle: PT1 filter at 2 Hz for silky smooth throttle transitions
    throttleOut = pt1FilterApply(&ap.throttleFilter, throttleOut);

    // 8. Calculate yaw coordination (follows roll for anti-slip)
    calculateYawControl();

    // 9. Detect pilot override
    detectPilotOverride();

    // 10. Generate RC commands
    generateRcCommands();
}

#endif // ENABLE_FLIGHT_PLAN

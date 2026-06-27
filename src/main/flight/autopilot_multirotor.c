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

#ifndef USE_WING

#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/vector.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "rx/rx.h"
#include "scheduler/scheduler.h"
#include "sensors/gyro.h"

#include "pg/autopilot.h"
#include "autopilot.h"
#include "flight/position_nav.h"

#ifdef USE_POSITION_HOLD
#include "flight/pos_hold.h"
#endif

// DEBUG_AUTOPILOT_PID each parameter on the axis set by gyro_filter_debug_axis
// 0 - VelocityError cm/s
// 1 - DistanceError cm
// 2 - P term * 10 // based on distance from intended position
// 3 - I term * 10 // integral of distance error over time
// 4 - D term * 10 // velocity factor ( distance error derivative)
// 5 - A term * 10 // velocity derivative factor (acceleration in distance terms)
// 6 - PIDsum * 10
// 7 - Status - encodes navActive+ 10, SticksActive +5, PositionHeld +3, +1 when starting,

// DEBUG_AUTOPILOT_STOP
// 0 - distance from position-hold target (cm)
// 1 - horizontal speed (cm/s)
// 2 - sticks active
// 3 - nav active
// 4 - position held
// 6 - Roll angle command * 100
// 7 - Pitch angle command * 100

#ifndef POSHOLD_TASK_RATE_HZ
#define POSHOLD_TASK_RATE_HZ 100
#endif

#define ALTITUDE_P_SCALE       0.005f
#define ALTITUDE_I_SCALE       0.002f
#define ALTITUDE_D_SCALE       0.01f
#define ALTITUDE_FF_KF_REF    30.0f
#define ALTITUDE_F_SCALE       0.1f / ALTITUDE_FF_KF_REF // full feedforward scale value when altitudeF CLI = 30
#define ALTITUDE_VEL_CMD_MAX_DEFAULT_CM_S  1500.0f
#define ALTITUDE_I_LIMIT      150.0f

// Using optical flow PID scales as the unified set

#define POSITION_P_SCALE       0.0012f
#define POSITION_I_SCALE       0.00015f
#define POSITION_D_SCALE       0.0017f
#define POSITION_A_SCALE       0.0003f
#define SANITY_CHECK_DISTANCE 3000.0f // TO DO: test set to a useful value, this is 30m
#define ERROR_DISTANCE_LIMIT  2000.0f // TO DO: test set to a useful value, this is 20m
#define POSITION_I_LIMIT      2000.0f // TO DO: test and set to a useful value, this is 20m

static pidCoefficient_t positionPidCoeffs;

static float altitudeKp;
static float altitudeKi;
static float altitudeKd;
static float altitudeKf;

// When autopilot hoverThrottle PG is 0, altitude hold captures rcCommand[THROTTLE] on mode entry.
#define AP_HOVER_THROTTLE_CAPTURE_MIN 1100U
#define AP_HOVER_THROTTLE_CAPTURE_MAX 1700U
static uint16_t altHoldCapturedHoverPwm;
static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

// (EF_EAST/EF_NORTH) is
// defined in common/axis.h alongside the other earth-frame axis enums.

static float targetPosition[EF_AXIS_COUNT];
static float posHoldStartPosition[EF_AXIS_COUNT];
static float distanceError[EF_AXIS_COUNT]; // deviation from intended position
static float distanceErrorIntegral[EF_AXIS_COUNT]; // integral of position error
static bool isPosHoldStarting[EF_AXIS_COUNT] = { false, false }; // to adjust pids while stopping before posHold
static float initialVelocity[EF_AXIS_COUNT] = { 0.0f, 0.0f }; // to detect stopping
static float previousVelocity[EF_AXIS_COUNT] = { 0.0f, 0.0f }; // for acceleration
static float dTermRamp[EF_AXIS_COUNT] = { 1.0f, 1.0f }; // to smooth D when starting
static bool isPositionHeld;
static bool wasPositionHeld = false;
static pt2Filter_t posAccelLpf[EF_AXIS_COUNT];
static pt2Filter_t posDtermLpf[EF_AXIS_COUNT];


static bool wasNavActive = false;

typedef struct autopilotState_s {
    float sanityCheckDistance;
    bool sticksActive;
    float maxAngle;
} autopilotState_t;
float autopilotAngle[RP_AXIS_COUNT];

static autopilotState_t ap = {
    .sanityCheckDistance = SANITY_CHECK_DISTANCE,
    .sticksActive = false,
};
static void initPidLpfs(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    const float cutoffHz = fmaxf(cfg->positionCutoff * 0.1f, 0.1f); // default of 30 is 3Hz, range 1 (value 10 or less) to 25Hz (value 250)
    const float k = pt2FilterGain(cutoffHz, HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ));
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        pt2FilterInit(&posAccelLpf[i], k);
        pt2FilterInit(&posDtermLpf[i], k);
    }
}

static void updatePositionHoldTarget(const positionEstimate3d_t *est)
{
    targetPosition[EF_EAST]  = est->position.v[ENU_E];
    targetPosition[EF_NORTH] = est->position.v[ENU_N];
    posHoldStartPosition[EF_EAST]  = est->position.v[ENU_E];
    posHoldStartPosition[EF_NORTH] = est->position.v[ENU_N];
}

void autopilotInit(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    ap.sticksActive = false;
    ap.maxAngle = cfg->maxAngle;

    altitudeKp = cfg->altitudeP * ALTITUDE_P_SCALE;
    altitudeKi = cfg->altitudeI * ALTITUDE_I_SCALE;
    altitudeKd = cfg->altitudeD * ALTITUDE_D_SCALE;
    altitudeKf = cfg->altitudeF * ALTITUDE_F_SCALE;

    positionPidCoeffs.Kp  = cfg->positionP  * POSITION_P_SCALE;
    positionPidCoeffs.Ki  = cfg->positionI  * POSITION_I_SCALE;
    positionPidCoeffs.Kd  = cfg->positionD  * POSITION_D_SCALE;
    positionPidCoeffs.Ka  = cfg->positionA  * POSITION_A_SCALE;

    initPidLpfs();
    positionNavInit();
}

void resetAltitudeControl(void)
{
    altitudeI = 0.0f;
    throttleOut = 0.0f;
}

uint16_t autopilotGetEffectiveHoverThrottlePwm(void)
{
    const uint16_t cfgHover = autopilotConfig()->hoverThrottle;
    if (cfgHover != 0) {
        return cfgHover;
    }
    if (altHoldCapturedHoverPwm != 0) {
        return altHoldCapturedHoverPwm;
    }
    return AP_HOVER_THROTTLE_DEFAULT;
}

void autopilotCaptureHoverThrottleForAltHold(void)
{
    if (autopilotConfig()->hoverThrottle != 0) {
        altHoldCapturedHoverPwm = 0;
        return;
    }
    altHoldCapturedHoverPwm = (uint16_t)lrintf(constrainf(rcCommand[THROTTLE], (float)AP_HOVER_THROTTLE_CAPTURE_MIN, (float)AP_HOVER_THROTTLE_CAPTURE_MAX));
}

void autopilotClearAltHoldHoverThrottle(void)
{
    altHoldCapturedHoverPwm = 0;
}

void altitudeControl(float targetAltitudeCm, float taskIntervalS, float targetAltitudeVelCmS, float velLimitCmS)
{
    
    // PI controller on altitude error
    const float currentAltitudeCm = getAltitudeCmControl(); // un-filtered altitude from Kalman filter
    const float altitudeErrorCm = targetAltitudeCm - currentAltitudeCm;
    const float itermRelax = (fabsf(altitudeErrorCm) < 200.0f) ? 1.0f : 0.1f; // don't accumulate too much iTerm with transient but large overshoots (>2m error )
    const float altitudeP = altitudeErrorCm * altitudeKp;
    altitudeI += altitudeErrorCm * altitudeKi * itermRelax * taskIntervalS;
    altitudeI = constrainf(altitudeI, -ALTITUDE_I_LIMIT, ALTITUDE_I_LIMIT);

    // Altitude Derivative
    const float verticalVelocity = getAltitudeDerivativeControl(); // un-filtered vertical velocity from Kalman filter
    const float velMax = (velLimitCmS > 1.0f) ? velLimitCmS : ALTITUDE_VEL_CMD_MAX_DEFAULT_CM_S;
    const float targetVerticalVelocity = constrainf(targetAltitudeVelCmS, -velMax, velMax);    
    float velocityError = targetVerticalVelocity - verticalVelocity;


    float dBoost = 1.0f;
    const float boostThreshold = 500.0f; // 5m/s
        const float absVerticalVelocity = fabsf(verticalVelocity);
        if (absVerticalVelocity > boostThreshold) {
            const float ratio = absVerticalVelocity / boostThreshold;
            dBoost = (3.0f * ratio - 2.0f) / ratio; // 1 at 5m/s, 2 at 10m/s...
    }

    const float altitudeD = velocityError * altitudeKd * dBoost;
    const float altitudeF = targetVerticalVelocity * altitudeKf;
    

    const float hoverOffset = (float)autopilotGetEffectiveHoverThrottlePwm() - PWM_RANGE_MIN;

    float throttleOffset = altitudeP
                         + altitudeI
                         + altitudeD
                         + altitudeF
                         + hoverOffset;

    // Tilt Compensation and limiting
    const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
    throttleOffset *= tiltMultiplier;

    float newThrottle = PWM_RANGE_MIN + throttleOffset;
    newThrottle = constrainf(newThrottle, autopilotConfig()->throttleMin, autopilotConfig()->throttleMax);
    
throttleOut = scaleRangef(newThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
throttleOut = constrainf(throttleOut, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 0, lrintf(newThrottle));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(tiltMultiplier * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 2, lrintf(targetAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 3, lrintf(currentAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 4, lrintf(altitudeP));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 5, lrintf(altitudeI));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 6, lrintf(altitudeD)); // includes innate feedforward since D is from error
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, lrintf(altitudeF)); // feedforward
}

void setSticksActiveStatus(bool areSticksActive)
{
    ap.sticksActive = areSticksActive;
}

static inline float calculateSanityCheckDistance(const positionEstimate3d_t *est)
{
    const float velE = est->velocity.v[ENU_E];
    const float velN = est->velocity.v[ENU_N];
    // Compute the 2D speed inline on the FPU only when called
    const float speedCmS = sqrtf((velE * velE) + (velN * velN));
    return fmaxf(SANITY_CHECK_DISTANCE, speedCmS * 2.0f);
}

static void resetDistanceError(void)
{
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        distanceError[i] = 0.0f; 
    }
}
static void resetDistanceErrorIntegral(void)
{
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        distanceErrorIntegral[i] = 0.0f; 
    }
}

static void setPreviousVelocity(const positionEstimate3d_t *est)
{
    previousVelocity[EF_EAST] = est->velocity.v[ENU_E];
    previousVelocity[EF_NORTH] = est->velocity.v[ENU_N];
}

static void initPositionHold(void){
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    setPreviousVelocity(est);
    resetDistanceError();
    // nb: we do not reset the distanceError integral, to hold it's opposition to wind between quick stick inputs
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        isPosHoldStarting[i] = true;
        dTermRamp[i] = 1.0f;
    }
}

static void initNavMode(void)
{
    initPidLpfs();
    resetDistanceError();
    resetDistanceErrorIntegral();
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        isPosHoldStarting[i] = false;
    }
}

void resetPositionControl(unsigned taskRateHz)
{
    UNUSED(taskRateHz);
    ap.sticksActive = false;

    // initialise the nav sistem
    positionEstimatorEnableXY(true);
    positionNavReset();
    wasNavActive = false; // will be enabled as required
     // set an initial sanity check distance
const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    ap.sanityCheckDistance = calculateSanityCheckDistance(est);
     isPositionHeld = true; // start in position hold mode
     initPositionHold(); // sets previousVelocity to current, resets the distance error, sets positionHold start mode, resets the dTerm start ramp
        resetDistanceErrorIntegral();
}

bool positionControl(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const timeDelta_t posholdDtUs = getTaskDeltaTimeUs(TASK_SELF);
    const float dt = (posholdDtUs > 0) ? (posholdDtUs * 1e-6f) : HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);
    if (!est->isValidXY) {
        return false;
    }
    vector2_t pidSumVectorEF                 = { { 0 } };
    float * const pidSumEF = (float*)&pidSumVectorEF;
    const float * const velocity = est->velocity.v;
    float targetVelocity[EF_AXIS_COUNT]      = { 0 };
    float velocityError[EF_AXIS_COUNT]       = { 0 };
    float pidP[EF_AXIS_COUNT]                = { 0 };
    float pidI[EF_AXIS_COUNT]                = { 0 };
    float pidD[EF_AXIS_COUNT]                = { 0 };
    float pidA[EF_AXIS_COUNT]                = { 0 };
    
    // Update navigation status
    positionNavUpdate(dt, est);
    const bool navActive = positionNavHasActiveTarget() && !positionNavTargetReached();

    if (navActive || ap.sticksActive) {
        isPositionHeld = false;

        if (navActive) {
            if (!wasNavActive) {
                // things to do when starting a nav mode
                initNavMode();
            }
            const vector3_t tgtVel = positionNavGetTargetVelocityCmS(); // update target velocity
            targetVelocity[EF_EAST]  = tgtVel.v[ENU_E];
            targetVelocity[EF_NORTH] = tgtVel.v[ENU_N];
        }
     } else {
            // control mode should be position hold
            if (!isPositionHeld) {
                // currently not in position hold, so start it - do these things on starting
                updatePositionHoldTarget(est);
                initPositionHold(); // set stopping to true, zero target velocity and integrals to zero
                ap.sanityCheckDistance = calculateSanityCheckDistance(est);
                isPositionHeld = true;
                initialVelocity[EF_EAST]  = velocity[EF_EAST]; 
                initialVelocity[EF_NORTH]  = velocity[EF_NORTH]; 
            } else {
                // in posHold
                const float sanityDistanceEast = posHoldStartPosition[EF_EAST] - est->position.v[ENU_E];
                const float sanityDistanceNorth = posHoldStartPosition[EF_NORTH] - est->position.v[ENU_N];
                const float sanityDistanceCm = sqrtf((sanityDistanceEast * sanityDistanceEast) + (sanityDistanceNorth * sanityDistanceNorth));
                if (sanityDistanceCm > ap.sanityCheckDistance) {
                updatePositionHoldTarget(est);
                    resetPositionControl(POSHOLD_TASK_RATE_HZ); //re-enable position hold
                    // allows pilot use sticks to bring it back -  maybe enter failsafe mode?
                    // was return(false) which meant that the pilot could not use sticks and craft was stuck current attitude, not stopping a flyaway
            }
        }
    }
    wasPositionHeld = isPositionHeld;
    wasNavActive = navActive;

    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
     const float currentPosition = est->position.v[axis];
        if (isPosHoldStarting[axis]) {
            // just starting positionhold
            dTermRamp[axis] += (1.6f - dTermRamp[axis]) * 0.1f;
            if (dTermRamp[axis] > 1.58f) {
                dTermRamp[axis] = 1.6f;
            }
            // while starting position hold, smoothly adjust target position as velocity decreases
            float slowingDownFactor = 0.0f;
            if (fabsf(initialVelocity[axis]) > 0.01f) {
                slowingDownFactor = 1.0f - (fabsf(velocity[axis]) / fabsf(initialVelocity[axis]));
                slowingDownFactor = constrainf(slowingDownFactor, 0.0f, 1.0f);
            } else {
                slowingDownFactor = 1.0f;
            }
            // smooth transition, while starting posHold, to new target point once movement stops
            const float positionAtStart = targetPosition[axis];
            const float tempTargetPosition = positionAtStart + (currentPosition - positionAtStart) * slowingDownFactor;
            distanceError[axis] = tempTargetPosition - currentPosition;
            if (((initialVelocity[axis] * velocity[axis]) < 0.0f) || (fabsf(velocity[axis]) < 20.0f)) {
                // craft has reversed direction, or slowed down enough to consider that it has stopped, on that axis
                isPosHoldStarting[axis] = false; // position hold is no longer starting up
                dTermRamp[axis] = 1.0f; // normal DTerm
                // finalise the target position for this axis to the current position
                targetPosition[axis] = currentPosition;
            }
        } else if (isPositionHeld) {
            // stopping is complete for this axis, undo the ramped up D smoothly
            dTermRamp[axis] += (1.0f - dTermRamp[axis]) * 0.1f;
            if (fabsf(dTermRamp[axis] - 1.0f) < 0.01f) {
                dTermRamp[axis] = 1.0f;
            }
            distanceError[axis] = targetPosition[axis] - currentPosition; //normal distance error calculation for posHold
        }
            const float velocityFiltered = pt2FilterApply(&posDtermLpf[axis], velocity[axis]);
            velocityError[axis] = targetVelocity[axis] - velocityFiltered;
            const float accelerationRaw = (previousVelocity[axis] - velocityFiltered) * POSHOLD_TASK_RATE_HZ; // match sign of velocityError
            previousVelocity[axis] = velocityFiltered;
             const float acceleration = pt2FilterApply(&posAccelLpf[axis], accelerationRaw);
        if (navActive) {
            distanceError[axis] += velocityError[axis] * dt; // in nav mode, get distance error from integral of velocity error
        }  else if (!isPositionHeld) {
            // sticksActive must be true
            distanceErrorIntegral[axis] *= 0.99f;
            // ease integral back towards zero while sticks deflected 
            // perhaps reduce in proportion to the velocity on the axis since velocity is desired correction?
            distanceError[axis] = 0.0f;
        }
        distanceError[axis] = constrainf(distanceError[axis], -ERROR_DISTANCE_LIMIT, ERROR_DISTANCE_LIMIT);
        distanceErrorIntegral[axis] += distanceError[axis] * dt * (isPosHoldStarting[axis] ? 0.0f : 1.0f);
        distanceErrorIntegral[axis] = constrainf(distanceErrorIntegral[axis], -POSITION_I_LIMIT, POSITION_I_LIMIT);

        pidP[axis] = distanceError[axis] * positionPidCoeffs.Kp;
        pidI[axis] = distanceErrorIntegral[axis] * positionPidCoeffs.Ki;
        pidD[axis] = velocityError[axis] * positionPidCoeffs.Kd * dTermRamp[axis];
        pidA[axis] = acceleration * positionPidCoeffs.Ka;
         pidSumEF[axis] = pidP[axis] + pidI[axis] + pidD[axis]+ pidA[axis];
    } // end for loop


    if (ap.sticksActive && navActive) {
        autopilotAngle[AI_ROLL]  = 0.0f; 
        autopilotAngle[AI_PITCH] = 0.0f;
    } else {

        // rotation from Earth Frame to BodyFrame
        // correction angle is based on difference beween craft heading and pidSum vector angle
        // intention is to return a pure pitch  or pure roll response  if craft heading matches pidSum vector heading
        const float headingRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
        vector2_t headingV;
        vector2_t angleV;

        headingV.v[EF_EAST]  = sinf(headingRad);
        headingV.v[EF_NORTH] = cosf(headingRad);
        angleV.v[AI_PITCH] = vector2Dot(&headingV, &pidSumVectorEF);
        angleV.v[AI_ROLL]  = vector2Cross(&headingV, &pidSumVectorEF);

        const float mag = vector2Norm(&angleV);
        if (mag > ap.maxAngle && mag > 0.001f) {
            const float scale = ap.maxAngle / mag;
            vector2Scale(&angleV, &angleV, scale);
        }

        autopilotAngle[AI_ROLL]  = -angleV.v[AI_ROLL];  // positive -> bank right, negative -> bank left
        autopilotAngle[AI_PITCH] =  angleV.v[AI_PITCH]; // positive -> nose down, negative -> nose up
    }

//      alternate rotation options:

//      1. simple 2D rotation based on attitude of craft only
//      Associated with unwanted roll in hard stops at high pitch angle
//         const float headingRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
//         const float sinHead = sinf(headingRad);
//         const floatcosHead = cosf(headingRad);
//         const float anglePitch =  (pidSumEF[EF_EAST] * sinHead) + (pidSumEF[EF_NORTH] * cosHead);
//         const float angleRoll  = -(pidSumEF[EF_EAST] * cosHead) + (pidSumEF[EF_NORTH] * sinHead);
//         const float mag = sqrtf((anglePitch * anglePitch) + (angleRoll * angleRoll));
//         if (mag > ap.maxAngle && mag > 0.0f) {
//             const float scale = ap.maxAngle / mag;
//         angleRoll  *= scale;
//         anglePitch *= scale;
//         }
//         autopilotAngle[AI_ROLL]  = -angleRoll; // positive -> roll right, negative --> roll left
//         autopilotAngle[AI_PITCH] = anglePitch; // positive -> nose down, negative -> nose up
//         }

// vector difference 2D maths using atan rather than dot-cross method - not tested
// 
//         const float pidSumVectorLength = sqrtf((pidSumEF[EF_EAST] * pidSumEF[EF_EAST]) + (pidSumEF[EF_NORTH] * pidSumEF[EF_NORTH]));
//         if (pidSumVectorLength > 0.001f) {
//             const float pidSumHeadingRad = atan2f(pidSumEF[EF_EAST], pidSumEF[EF_NORTH]);
//             const float craftHeadingRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
//             const float relativeAngleRad = pidSumHeadingRad - craftHeadingRad;
//             float anglePitch = pidSumVectorLength * cosf(relativeAngleRad);
//             float angleRoll  = pidSumVectorLength * sinf(relativeAngleRad);

//         const float mag = sqrtf((anglePitch * anglePitch) + (angleRoll * angleRoll));
//         if (mag > ap.maxAngle && mag > 0.0f) {
//             const float scale = ap.maxAngle / mag;
//             angleRoll  *= scale;
//             anglePitch *= scale;
//         }
//         autopilotAngle[AI_ROLL]  = -angleRoll; // positive -> roll right, negative --> roll left
//         autopilotAngle[AI_PITCH] = anglePitch; // positive -> nose down, negative -> nose up
//     }

    const unsigned debugAxis = (gyroConfig()->gyro_filter_debug_axis == FD_PITCH) ? 1 : 0;
    // 1 or Pitch aligns with North, Roll with East
    int statusValue = 0;
    if (navActive)          statusValue += 10;
    if (isPositionHeld)     statusValue += 3; // plus 1, ie 4,  if stopping
    if (ap.sticksActive)    statusValue += 5;

    DEBUG_SET(DEBUG_AUTOPILOT_PID, 0, lrintf(velocityError[debugAxis])); // velocity error
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 1, lrintf(distanceError[debugAxis])); // distance error
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 2, lrintf(pidP[debugAxis]*10));   
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 3, lrintf(pidI[debugAxis]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 4, lrintf(pidD[debugAxis]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 5, lrintf(pidA[debugAxis]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 6, lrintf( pidSumEF[debugAxis]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, statusValue + (isPosHoldStarting[debugAxis ? 1 :0]));

    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 0, lrintf(velocityError[EF_EAST] ));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 1, lrintf(velocityError[EF_NORTH]));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 2, lrintf(pidSumEF[EF_EAST] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 3, lrintf(pidSumEF[EF_NORTH] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 4, lrintf(autopilotAngle[AI_ROLL] *10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 5, lrintf(autopilotAngle[AI_PITCH] *10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 6, statusValue + (isPosHoldStarting[EF_EAST] ? 1 : 0));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 7, statusValue + (isPosHoldStarting[EF_NORTH] ? 1 : 0));

return true;
}

bool isBelowLandingAltitude(void)
{
    return getAltitudeCmControl() < 100.0f * autopilotConfig()->landingAltitudeM;
}

float getAutopilotThrottle(void)
{
    return throttleOut;
}

bool isAutopilotInControl(void) // use normal angle mode stick control
{
    return !ap.sticksActive;
}

#endif // !USE_WING

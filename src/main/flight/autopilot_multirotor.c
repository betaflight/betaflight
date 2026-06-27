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
#define SANITY_CHECK_DISTANCE 2000.0f //20m, increased when stopping from speeds above 10m/s
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

static vector2_t targetPosition;
static vector2_t posHoldStartPosition;
static vector2_t distanceError;          // deviation from intended position
static vector2_t distanceErrorIntegral;  // integral of position error
static vector2_t initialVelocity;        // to detect stopping
static vector2_t previousVelocity;       // for acceleration
static vector2_t dTermRamp;              // to smooth D when starting position hold
static bool isPosHoldStarting[EF_AXIS_COUNT] = { false, false }; 

static bool isPositionHeld;
static bool wasPositionHeld = false;
static pt2Filter_t posAccelLpf[EF_AXIS_COUNT];
static pt2Filter_t posDtermLpf[EF_AXIS_COUNT];


static bool wasNavActive = false;

typedef struct autopilotState_s {
    float sanityCheckDistance;
    bool sticksActive;
    bool navActive;
    float maxAngle;
    unsigned debugAxis;
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

void autopilotInit(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    ap.sticksActive = false;
    ap.maxAngle = cfg->maxAngle;
    ap.debugAxis = (gyroConfig()->gyro_filter_debug_axis == FD_PITCH) ? 1 : 0; // 1 for Pitch / North, 0 for Roll / East

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

static void updatePositionHoldTarget(void)
{
    const vector2_t *currentPos = (const vector2_t *)&positionEstimatorGetEstimate()->position.v;
    targetPosition       = *currentPos;
    posHoldStartPosition = *currentPos;
}

void setSticksActiveStatus(bool areSticksActive)
{
    ap.sticksActive = areSticksActive;
}

static inline float calculateSanityCheckDistance(void)
{
    const float speedCmS = vector2Norm((const vector2_t *)&positionEstimatorGetEstimate()->velocity.v);
    return fmaxf(SANITY_CHECK_DISTANCE, speedCmS * 2.0f); // 20m floor, scales up past 10 m/s
}

static void resetDistanceError(void)
{
    distanceError = (vector2_t){{ 0.0f, 0.0f }};
}

static void resetDistanceErrorIntegral(void)
{
    distanceErrorIntegral = (vector2_t){{ 0.0f, 0.0f }};
}

static void initPositionHold(void)
{
     updatePositionHoldTarget();
    resetDistanceError();
    isPosHoldStarting[EF_EAST]  = true;
    isPosHoldStarting[EF_NORTH] = true;
    // nb: we do not reset the distanceError integral, to hold its opposition to wind between quick stick inputs
}

static void initNavMode(void)
{
    initPidLpfs();
    resetDistanceError();
    resetDistanceErrorIntegral();
    isPosHoldStarting[EF_EAST]  = false;
    isPosHoldStarting[EF_NORTH] = false;
}

void resetPositionControl(unsigned taskRateHz)
{
    UNUSED(taskRateHz);
    ap.sticksActive = false;

    // Initialise the nav system
    positionEstimatorEnableXY(true);
    positionNavReset();
    wasNavActive = false; // will be enabled as required
    // Set an initial sanity check distance
    ap.sanityCheckDistance = calculateSanityCheckDistance();
    // Initialise posHold
    initPositionHold(); // sets target location, resets distance error, enables start mode
    previousVelocity = *(const vector2_t *)&positionEstimatorGetEstimate()->velocity.v; // for smooth A in any mode

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
    
    const vector2_t currentPosition = *(const vector2_t *)&est->position.v;
    const vector2_t velocity = *(const vector2_t *)&est->velocity.v;

    // Local factor arrays fully converted to vector2_t objects
    vector2_t pidSumVectorEF     = { { 0 } };
    vector2_t targetVelocity     = { { 0 } };
    vector2_t velocityError      = { { 0 } };
    vector2_t pidP               = { { 0 } };
    vector2_t pidI               = { { 0 } };
    vector2_t pidD               = { { 0 } };
    vector2_t pidA               = { { 0 } };
    
    // Update navigation status
    positionNavUpdate(dt, est);
    ap.navActive = positionNavHasActiveTarget() && !positionNavTargetReached();

    if (ap.navActive || ap.sticksActive) {
        isPositionHeld = false;

        if (ap.navActive) {
            if (!wasNavActive) {
                initNavMode();
            }
            const vector3_t tgtVel = positionNavGetTargetVelocityCmS();
            targetVelocity = *(const vector2_t *)&tgtVel.v;
        }
    } else {
        // Control mode should be position hold
        if (!isPositionHeld) {
            // not in positionHold, but should be
            initPositionHold();
            ap.sanityCheckDistance = calculateSanityCheckDistance();
            isPositionHeld = true;
            initialVelocity = velocity;
        } else {
            // In posHold 
            vector2_t deltaPosV;
            vector2Sub(&deltaPosV, &posHoldStartPosition, &currentPosition);
            const float sanityDistanceCm = vector2Norm(&deltaPosV);
            
            if (sanityDistanceCm > ap.sanityCheckDistance) {
                setSticksActiveStatus(true); // Manual pitch/roll control in angle mode
                autopilotAngle[AI_ROLL]  = 0.0f; // Level out
                autopilotAngle[AI_PITCH] = 0.0f;
                DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, 50); // mark the failure this in the status debug channels
                DEBUG_SET(DEBUG_AUTOPILOT_STOP, 6, 50);
                DEBUG_SET(DEBUG_AUTOPILOT_STOP, 7, 50);
                return false; // Return failure and show pos hold fail message in OSD
            }
        }
    }
    wasPositionHeld = isPositionHeld;
    wasNavActive = ap.navActive;

    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
        if (isPositionHeld) {
            if (isPosHoldStarting[axis]) {
                dTermRamp.v[axis] += (1.6f - dTermRamp.v[axis]) * 0.1f;
                if (dTermRamp.v[axis] > 1.58f) {
                    dTermRamp.v[axis] = 1.6f;
                }

                float slowingDownFactor = 0.0f;
                if (fabsf(initialVelocity.v[axis]) > 0.01f) {
                    slowingDownFactor = 1.0f - (fabsf(velocity.v[axis]) / fabsf(initialVelocity.v[axis]));
                    slowingDownFactor = constrainf(slowingDownFactor, 0.0f, 1.0f);
                } else {
                    slowingDownFactor = 1.0f; // reduce abrupt P and iTerm windup by not abruptly jumping to a new target point at speed
                }

                const float tempTargetPosition = targetPosition.v[axis] + (currentPosition.v[axis] - targetPosition.v[axis]) * slowingDownFactor;
                distanceError.v[axis] = tempTargetPosition - currentPosition.v[axis];
                
                if (((initialVelocity.v[axis] * velocity.v[axis]) < 0.0f) || (fabsf(velocity.v[axis]) < 20.0f)) {
                    isPosHoldStarting[axis] = false; 
                    dTermRamp.v[axis] = 1.0f; 
                    targetPosition.v[axis] = currentPosition.v[axis];
                }
            } else {
                dTermRamp.v[axis] += (1.0f - dTermRamp.v[axis]) * 0.1f;
                if (fabsf(dTermRamp.v[axis] - 1.0f) < 0.01f) {
                    dTermRamp.v[axis] = 1.0f;
                }
                distanceError.v[axis] = targetPosition.v[axis] - currentPosition.v[axis]; 
            }
        }
        
        const float velocityFiltered = pt2FilterApply(&posDtermLpf[axis], velocity.v[axis]);
        velocityError.v[axis] = targetVelocity.v[axis] - velocityFiltered;
        const float accelerationRaw = (previousVelocity.v[axis] - velocityFiltered) * POSHOLD_TASK_RATE_HZ; 
        previousVelocity.v[axis] = velocityFiltered;
        const float acceleration = pt2FilterApply(&posAccelLpf[axis], accelerationRaw);

        if (ap.navActive) {
            distanceError.v[axis] += velocityError.v[axis] * dt; 
        } else if (!isPositionHeld) {
            // sticks active
            distanceErrorIntegral.v[axis] *= 0.99f;
            distanceError.v[axis] = 0.0f;
        }

        distanceError.v[axis] = constrainf(distanceError.v[axis], -ERROR_DISTANCE_LIMIT, ERROR_DISTANCE_LIMIT);
        distanceErrorIntegral.v[axis] += distanceError.v[axis] * dt * (isPosHoldStarting[axis] ? 0.0f : 1.0f);
        distanceErrorIntegral.v[axis] = constrainf(distanceErrorIntegral.v[axis], -POSITION_I_LIMIT, POSITION_I_LIMIT);

        pidP.v[axis] = distanceError.v[axis] * positionPidCoeffs.Kp;
        pidI.v[axis] = distanceErrorIntegral.v[axis] * positionPidCoeffs.Ki;
        pidD.v[axis] = velocityError.v[axis] * positionPidCoeffs.Kd * dTermRamp.v[axis];
        pidA.v[axis] = acceleration * positionPidCoeffs.Ka;
        
        pidSumVectorEF.v[axis] = pidP.v[axis] + pidI.v[axis] + pidD.v[axis] + pidA.v[axis];
    } // End for loop

        // Rotation from Earth Frame to Body Frame
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

        autopilotAngle[AI_ROLL]  = -angleV.v[AI_ROLL];  
        autopilotAngle[AI_PITCH] =  angleV.v[AI_PITCH]; 

    int statusValue = 0;
    if (ap.navActive)          statusValue += 10;
    if (isPositionHeld)     statusValue += 3; // plus 1, ie 4,  if stopping
    if (ap.sticksActive)    statusValue += 5;

    DEBUG_SET(DEBUG_AUTOPILOT_PID, 0, lrintf(velocityError.v[ap.debugAxis])); // velocity error
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 1, lrintf(distanceError.v[ap.debugAxis])); // distance error
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 2, lrintf(pidP.v[ap.debugAxis] * 10));   
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 3, lrintf(pidI.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 4, lrintf(pidD.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 5, lrintf(pidA.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 6, lrintf(pidSumVectorEF.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, statusValue + (isPosHoldStarting[ap.debugAxis] ? 1 : 0));

    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 0, lrintf(velocityError.v[EF_EAST]));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 1, lrintf(velocityError.v[EF_NORTH]));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 2, lrintf(pidSumVectorEF.v[EF_EAST] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 3, lrintf(pidSumVectorEF.v[EF_NORTH] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 4, lrintf(autopilotAngle[AI_ROLL] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 5, lrintf(autopilotAngle[AI_PITCH] * 10));
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

bool isAutopilotInControl(void)  // when false, pid.c allows flight in angle mode controlled by pilot
{
    // If an autonomous navigation path is active, the autopilot stays in control even if sticks are moved
    if (ap.navActive) {
        return true;
    }

    // in position hold, if sticks are moved, angle mode flight controlled by the pilot is active
    return !ap.sticksActive;
}

#endif // !USE_WING

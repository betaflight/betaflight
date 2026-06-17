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

// DEBUG_AUTOPILOT_PID
// 0 - P term (East) * 100
// 1 - P term (North) * 100
// 2 - I term (East) * 100
// 3 - I term (North) * 100
// 4 - II term (East) * 100
// 5 - II term (North) * 100
// 6 - Roll angle command * 100
// 7 - Pitch angle command * 100

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

#define ALTITUDE_P_SCALE       0.01f
#define ALTITUDE_I_SCALE       0.002f
#define ALTITUDE_D_SCALE       0.01f
#define ALTITUDE_FF_KF_REF    30.0f
#define ALTITUDE_F_SCALE       0.1f / ALTITUDE_FF_KF_REF // full feedforward scale value when altitudeF CLI = 30
#define ALTITUDE_VEL_CMD_MAX_DEFAULT_CM_S  1500.0f
#define ALTITUDE_I_LIMIT      150.0f

// Using optical flow PID scales as the unified set

#define POSITION_P_SCALE       0.001f
#define POSITION_I_SCALE      (0.1f * POSITION_P_SCALE)
#define POSITION_D_SCALE       0.002f
#define ERROR_DISTANCE_LIMIT 10000.0f // TO DO: test set to a useful value, this is 100m
#define POSITION_I_LIMIT 10000.0f // TO DO: test and set to a useful value, this is 100m

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

typedef enum {
    EF_EAST = 0,
    EF_NORTH
} efAxis_e;

static float distanceError[EF_AXIS_COUNT]; // deviation from intended position
static float distanceErrorIntegral[EF_AXIS_COUNT]; // integral of position error
static bool isStopping[EF_AXIS_COUNT] = { false, false }; // to adjust pids while stopping before posHold
static float initialVelocity[EF_AXIS_COUNT] = { 0.0f, 0.0f }; // to detect stopping
static bool isPositionHeld;
static bool wasPositionHeld = false;
static float posHoldTransition = 0.0f; 
static float previousPidSum[EF_AXIS_COUNT] = { 0.0f, 0.0f };

// PT1 on finite-difference accel for horizontal D (KF velocity is noisy at POSHOLD_TASK_RATE_HZ).
// Cutoff Hz = ap_position_cutoff × 0.01 (same scale as legacy GPS VA filters).
static pt1Filter_t posAccelLpf[EF_AXIS_COUNT];

static vector3_t targetPosition;
static bool wasNavActive = false;

typedef struct autopilotState_s {
    float sanityCheckDistance;
    bool sticksActive;
    float maxAngle;
} autopilotState_t;
float autopilotAngle[RP_AXIS_COUNT];

static autopilotState_t ap = {
    .sanityCheckDistance = 1000.0f,
    .sticksActive = false,
};
static void initPositionAccelLpf(void)
{
    // currently position acceleration not used at present and could be removed
    const autopilotConfig_t *cfg = autopilotConfig();
    const float cutoffHz = fmaxf(cfg->positionCutoff * 0.01f, 0.1f);
    const float k = pt1FilterGain(cutoffHz, HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ));
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        pt1FilterInit(&posAccelLpf[i], k);
    }
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

    initPositionAccelLpf();
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
static inline float sanityCheckDistance(const float speedCmS)
{
    return fmaxf(1000.0f, speedCmS * 2.0f);
}
static void capturePositionHoldTarget(const positionEstimate3d_t *est)
{
    targetPosition.x = est->position.x; // EAST position
    targetPosition.y = est->position.y; // NORTH position
    targetPosition.z = est->position.z;
}


static void resetDistanceErrorAndIntegral(void){

    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        distanceError[i] = 0.0f; 
        distanceErrorIntegral[i] = 0.0f;
    }
}

static void initPositionHold(void){
    initPositionAccelLpf();
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        distanceError[i] = 0.0f; 
        distanceErrorIntegral[i] = 0.0f;
        isStopping[i] = true;
    }
}


static void initNavMode(void)
{
    initPositionAccelLpf();
    resetDistanceErrorAndIntegral();
}


void resetPositionControl(unsigned taskRateHz)
{
    UNUSED(taskRateHz);
    ap.sticksActive = false;
    initPositionHold();
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    ap.sanityCheckDistance = sanityCheckDistance(vector2Norm((vector2_t*)&est->velocity));
    initialVelocity[EF_EAST]  = est->velocity.x;
    initialVelocity[EF_NORTH] = est->velocity.y;

    positionEstimatorEnableXY(true);
    capturePositionHoldTarget(est); 
    isPositionHeld = true; 

    positionNavReset();
    wasNavActive = false;
}

bool positionControl(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const timeDelta_t posholdDtUs = getTaskDeltaTimeUs(TASK_SELF);
    const float dt = (posholdDtUs > 0) ? (posholdDtUs * 1e-6f) : HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);
    if (!est->isValidXY) {
        return false;
    }

    const float speedXY = vector2Norm((vector2_t*)&est->velocity);
    const float velocity[EF_AXIS_COUNT] = { est->velocity.x, est->velocity.y };
    static float targetVelocity[EF_AXIS_COUNT] = { 0.0f, 0.0f };
    float velocityError[EF_AXIS_COUNT] = { 0.0f, 0.0f }; 
    float pidP[EF_AXIS_COUNT] = { 0.0f, 0.0f };
    float pidI[EF_AXIS_COUNT] = { 0.0f, 0.0f };
    float pidD[EF_AXIS_COUNT] = { 0.0f, 0.0f };
    static vector2_t pidSumEF = {{ 0, 0 }};
    vector2_t anglesBF = {{ 0, 0 }};

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
            const vector3_t tgtVel = positionNavGetTargetVelocityCmS();
            targetVelocity[EF_EAST]  = tgtVel.x;
            targetVelocity[EF_NORTH] = tgtVel.y;
        } else { 
            // sticks are active
            targetVelocity[EF_EAST]  = getSetpointRate(FD_ROLL); // use setpoint to soften centering
            targetVelocity[EF_NORTH] =getSetpointRate(FD_PITCH);
        }
    } else {
        // if neither In a Nav mode, nor sticksActive, should be in position hold
        if (!isPositionHeld) {
            // start position hold - things to do once on starting
                capturePositionHoldTarget(est);
            initPositionHold();
            ap.sanityCheckDistance = sanityCheckDistance(speedXY);
            isPositionHeld = true;
            initialVelocity[EF_EAST]  = est->velocity.x;
            initialVelocity[EF_NORTH] = est->velocity.y;
        } else {
            // update distance error
            distanceError[EF_EAST]  = targetPosition.x - est->position.x;
            distanceError[EF_NORTH] = targetPosition.y - est->position.y;

           const float errorDistanceCm = sqrtf(distanceError[EF_EAST] * distanceError[EF_EAST] + distanceError[EF_NORTH] * distanceError[EF_NORTH]);
            if (errorDistanceCm > ap.sanityCheckDistance) {
                return false;
            }
        }
    }
    wasNavActive = navActive;
    if (isPositionHeld != wasPositionHeld) {
        posHoldTransition = 1.0f;
        for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
            previousPidSum[i] = pidSumEF.v[i]; // update stored pidSum
        }
    }
    wasPositionHeld = isPositionHeld;
    if (posHoldTransition > 0.0f) {
        posHoldTransition *= 0.9f;
        if (posHoldTransition < 0.01f) {
            posHoldTransition = 0.0f; // Clean cutoff to zero
        }
    }

    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
        if (isStopping[axis]) {
    if ((initialVelocity[axis] * velocity[axis]) < 0.0f) {
                isStopping[axis] = false;
                targetVelocity[axis] = 0.0f;
            }
        }
        if (navActive) {
            distanceError[axis] += velocityError[axis] * dt;
        }  else if (!isPositionHeld){
            // sticksActive must be true, get target velocity from sticks, zero the distance error
            distanceError[axis] = 0.0f; // don't accumulate distance until sticks stop
        }
        // normal situation, either nav mode or posHold, may be stopping
        // limit integrals and calculate pids
        distanceError[axis] = constrainf(distanceError[axis], -ERROR_DISTANCE_LIMIT, ERROR_DISTANCE_LIMIT);
        distanceErrorIntegral[axis] += distanceError[axis] * dt * (isStopping[axis] ? 0.0f : 1.0f);
        distanceErrorIntegral[axis] = constrainf(distanceErrorIntegral[axis], -POSITION_I_LIMIT, POSITION_I_LIMIT);

        pidP[axis] = distanceError[axis] * positionPidCoeffs.Kp;
        pidI[axis] = distanceErrorIntegral[axis] * positionPidCoeffs.Ki;
        velocityError[axis] = targetVelocity[axis] - velocity[axis];
        pidD[axis] = velocityError[axis] * positionPidCoeffs.Kd * (isStopping[axis] ? 1.6f : 1.0f);
        float pidSumThisAxis = pidP[axis] + pidI[axis] + pidD[axis];
//        pidSumEF.v[axis] = pidP[axis] + pidI[axis] + pidD[axis];
        pidSumEF.v[axis] = pidSumThisAxis + (posHoldTransition * (previousPidSum[axis] - pidSumThisAxis));

    } // end for loop

    if (ap.sticksActive && navActive) {
        anglesBF = (vector2_t){{0, 0}}; // unsure what this is for, stick input stops nav and flattens out?
    } else {
        const float angle = DECIDEGREES_TO_RADIANS(attitude.values.yaw - 900);
        vector2_t pidBodyFrame;
        vector2Rotate(&pidBodyFrame, &pidSumEF, angle);
        anglesBF.v[AI_ROLL]  = -pidBodyFrame.y;
        anglesBF.v[AI_PITCH] = pidBodyFrame.x;
        const float mag = vector2Norm(&anglesBF);
        if (mag > ap.maxAngle && mag > 0.0f) {
            vector2Scale(&anglesBF, &anglesBF, ap.maxAngle / mag);
        }
    }

    autopilotAngle[AI_ROLL] = anglesBF.v[AI_ROLL]; // AI_ROLL is index 0
    autopilotAngle[AI_PITCH] = anglesBF.v[AI_PITCH];

    const unsigned debugAxis = (gyroConfig()->gyro_filter_debug_axis == FD_PITCH) ? 1 : 0;
    // Pitch aligns with North, Roll with East
    int statusValue = 0;
    if (navActive)          statusValue += 10;
    if (isPositionHeld)     statusValue += 3; // plus 1, ie 4,  if stopping
    if (ap.sticksActive)    statusValue += 5;
    
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 0, lrintf(velocityError[debugAxis])); // velocity error
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 1, lrintf(distanceError[debugAxis])); // distance error
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 2, lrintf(pidP[debugAxis]*10));   
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 3, lrintf(pidI[debugAxis]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 4, lrintf(pidD[debugAxis]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 5, lrintf( pidSumEF.v[debugAxis]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 6, lrintf(autopilotAngle[debugAxis]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, statusValue + (isStopping[debugAxis ? 1 :0]));

//    const float stopDistanceEast = est->position.x - targetPosition.x;
//    const float stopDistanceNorth = est->position.y - targetPosition.y;

//    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 0, lrintf(sqrtf(stopDistanceEast * stopDistanceEast + stopDistanceNorth * stopDistanceNorth)));
//    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 1, lrintf(speedXY));

    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 0, lrintf(velocityError[AI_ROLL]));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 1, lrintf(velocityError[AI_PITCH]));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 2, lrintf(autopilotAngle[AI_ROLL] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 3, lrintf(autopilotAngle[AI_PITCH] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 4, lrintf( pidSumEF.v[AI_ROLL]*10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 5, lrintf( pidSumEF.v[AI_PITCH]*10));

    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, statusValue + (isStopping[debugAxis ? 1 :0]));

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

bool isAutopilotInControl(void)
{
    return !ap.sticksActive;
}

#endif // !USE_WING

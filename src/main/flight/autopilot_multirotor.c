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
#define POSITION_P_SCALE       0.0033f
#define POSITION_I_SCALE       0.0007f
#define POSITION_II_SCALE      (0.12f * POSITION_I_SCALE)
#define POSITION_D_SCALE       0.00011f

#define POSITION_IWINDUP_LIMIT 250.0f
#define UPSAMPLING_CUTOFF_HZ   5.0f

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

// Per-axis position PID state (earth frame). efAxis_e (EF_EAST/EF_NORTH) is
// defined in common/axis.h alongside the other earth-frame axis enums.
static float posIntegral[EF_AXIS_COUNT];       // I term: integral of position error
static float posSlowIntegral[EF_AXIS_COUNT];   // II term: slow drift correction
static float previousVelocity[EF_AXIS_COUNT];
// True when the horizontal hold point is active for full position I/II (captured, not braking, sticks centered).
static bool isPositionHeld;

// PT1 on finite-difference accel for horizontal D (KF velocity is noisy at POSHOLD_TASK_RATE_HZ).
// Cutoff Hz = ap_position_cutoff × 0.01 (same scale as legacy GPS VA filters).
static pt1Filter_t posAccelLpf[EF_AXIS_COUNT];

static vector3_t targetPosition;
static bool wasNavActive = false;

typedef struct autopilotState_s {
    float sanityCheckDistance;
    float upsampleLpfGain;
    bool sticksActive;
    float maxAngle;
    vector2_t pidSumBF;
    pt3Filter_t upsampleLpfBF[RP_AXIS_COUNT];
} autopilotState_t;

static autopilotState_t ap = {
    .sanityCheckDistance = 1000.0f,
    .upsampleLpfGain = 1.0f,
    .sticksActive = false,
};

float autopilotAngle[RP_AXIS_COUNT];

static void resetUpsampleFilters(void)
{
    for (unsigned i = 0; i < ARRAYLEN(ap.upsampleLpfBF); i++) {
        pt3FilterInit(&ap.upsampleLpfBF[i], ap.upsampleLpfGain);
    }
}

static void initPositionAccelLpf(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    const float cutoffHz = fmaxf(cfg->positionCutoff * 0.01f, 0.1f);
    const float k = pt1FilterGain(cutoffHz, HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ));
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        pt1FilterInit(&posAccelLpf[i], k);
    }
}

static inline float sanityCheckDistance(const float speedCmS)
{
    return fmaxf(1000.0f, speedCmS * 2.0f);
}

static void capturePositionHoldTarget(const positionEstimate3d_t *est)
{
    isPositionHeld = true;
    targetPosition = est->position;  // ENU
    ap.sanityCheckDistance = sanityCheckDistance(1000.0f);
}

static void resetPositionStopState(const positionEstimate3d_t *est)
{
    // Prevent a sharp spike on D
    previousVelocity[EF_EAST] = est->velocity.v[ENU_E];
    previousVelocity[EF_NORTH] = est->velocity.v[ENU_N];

    // Reset the D filter
    initPositionAccelLpf();

    // Reset the angle filter
    resetUpsampleFilters();
}

void resetPositionControl(unsigned taskRateHz)
{
    ap.sticksActive = false;

    // Set sanity distance from current speed
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float speedXY = sqrtf(sq(est->velocity.v[ENU_E]) + sq(est->velocity.v[ENU_N]));
    ap.sanityCheckDistance = sanityCheckDistance(speedXY);

    // Reset PID state (velocity-only until capture)
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        posIntegral[i] = 0.0f;
        posSlowIntegral[i] = 0.0f;
        previousVelocity[i] = 0.0f;
    }

    const float taskInterval = 1.0f / taskRateHz;
    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, taskInterval);
    resetUpsampleFilters();
    initPositionAccelLpf();

    // Enable XY estimation and set target to current position
    positionEstimatorEnableXY(true);
    targetPosition = est->position;  // ENU
    isPositionHeld = true;

    positionNavReset();
    wasNavActive = false;
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
    positionPidCoeffs.Kii = cfg->positionII * POSITION_II_SCALE;
    positionPidCoeffs.Kd  = cfg->positionD  * POSITION_D_SCALE;

    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, 0.01f);
    resetUpsampleFilters();
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

bool positionControl(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const timeDelta_t posholdDtUs = getTaskDeltaTimeUs(TASK_SELF);
    const float dt = (posholdDtUs > 0) ? (posholdDtUs * 1e-6f) : HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);

    if (!est->isValidXY) {
        return false;
    }

    // Run the navigation outer loop (no-op when no target is set)
    positionNavUpdate(dt, est);
    const bool navActive = positionNavHasActiveTarget() && !positionNavTargetReached();

    // Smooth transition: nav just completed/cleared -> seed position hold target
    if (!navActive && wasNavActive) {
        targetPosition = est->position;  // ENU
        for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
            posSlowIntegral[i] = 0.0f;
        }
        isPositionHeld = true;
        ap.sanityCheckDistance = sanityCheckDistance(1000.0f);
    }
    wasNavActive = navActive;

    const float velEast  = est->velocity.v[ENU_E];
    const float velNorth = est->velocity.v[ENU_N];
    const float speedXY  = sqrtf(velEast * velEast + velNorth * velNorth);
    const float velocities[EF_AXIS_COUNT] = { velEast, velNorth };

    const bool capturedPositionHold = !navActive && !ap.sticksActive && !isPositionHeld
        && speedXY < autopilotConfig()->stopThreshold;
    if (capturedPositionHold) {
        capturePositionHoldTarget(est);
        resetPositionStopState(est);
    }

    vector2_t pidSumEF = {{ 0, 0 }};
    float distanceCm = 0.0f;
    float errorEast = 0.0f;

    if (navActive) {
        // Nav mode: inner velocity-tracking PID
        const vector3_t tgtVel = positionNavGetTargetVelocityCmS();
        const float velErrors[EF_AXIS_COUNT] = {
            velEast - tgtVel.v[ENU_E],
            velNorth - tgtVel.v[ENU_N]
        };

        for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
            const float pidP = velErrors[axis] * positionPidCoeffs.Kp;

            const float accelRaw = (velocities[axis] - previousVelocity[axis]) / dt;
            previousVelocity[axis] = velocities[axis];
            const float accel = pt1FilterApply(&posAccelLpf[axis], accelRaw);
            const float pidD = -accel * positionPidCoeffs.Kd;

            pidSumEF.v[axis] = pidP + pidD;

            if (axis == 0) {
                DEBUG_SET(DEBUG_AUTOPILOT_PID, 0, lrintf(pidP * 100));
                DEBUG_SET(DEBUG_AUTOPILOT_PID, 2, 0);
                DEBUG_SET(DEBUG_AUTOPILOT_PID, 4, lrintf(pidD * 100));
            } else {
                DEBUG_SET(DEBUG_AUTOPILOT_PID, 1, lrintf(pidP * 100));
                DEBUG_SET(DEBUG_AUTOPILOT_PID, 3, 0);
                DEBUG_SET(DEBUG_AUTOPILOT_PID, 5, lrintf(pidD * 100));
            }
        }
    } else {
        errorEast = est->position.v[ENU_E] - targetPosition.v[ENU_E];
        const float errorNorth = est->position.v[ENU_N] - targetPosition.v[ENU_N];
        distanceCm = sqrtf(errorEast * errorEast + errorNorth * errorNorth);

        if (distanceCm > ap.sanityCheckDistance) {
            return false;
        }

        const float errors[EF_AXIS_COUNT] = { errorEast, errorNorth };

        for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
            const float velocity = velocities[axis];

            const float pidP = velocity * positionPidCoeffs.Kp;

            float pidI = 0.0f;
            float pidII = 0.0f;

            if (isPositionHeld) {
                const float error = errors[axis];
                pidI = error * positionPidCoeffs.Ki;
                posSlowIntegral[axis] += error * dt;
                posSlowIntegral[axis] = constrainf(posSlowIntegral[axis],
                                                    -POSITION_IWINDUP_LIMIT,
                                                    POSITION_IWINDUP_LIMIT);
                pidII = posSlowIntegral[axis] * positionPidCoeffs.Kii;
            } else {
                posSlowIntegral[axis] = 0.0f;
            }

            const float accelRaw = (velocity - previousVelocity[axis]) / dt;
            previousVelocity[axis] = velocity;
            const float accel = pt1FilterApply(&posAccelLpf[axis], accelRaw);
            const float pidD = -accel * positionPidCoeffs.Kd;

            pidSumEF.v[axis] = pidP + pidI + pidII + pidD;

            DEBUG_SET(DEBUG_AUTOPILOT_PID, 0 + axis, lrintf(pidP * 100));
            DEBUG_SET(DEBUG_AUTOPILOT_PID, 2 + axis, lrintf(pidI * 100));
            DEBUG_SET(DEBUG_AUTOPILOT_PID, 4 + axis, lrintf(pidII * 100));
        }
    }

    // Handle sticks and rotate the autopilot output to body frame.
    vector2_t anglesBF;

    if (ap.sticksActive) {
        anglesBF = (vector2_t){{0, 0}};
        if (!navActive) {
            isPositionHeld = false;
            for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
                posSlowIntegral[axis] = 0.0f;
            }
            targetPosition = est->position;  // ENU
            ap.sanityCheckDistance = sanityCheckDistance(speedXY);
        }
    } else {
        // Rotate ENU PID output to body frame
        const float angle = DECIDEGREES_TO_RADIANS(attitude.values.yaw - 900);
        vector2_t pidBodyFrame;
        vector2Rotate(&pidBodyFrame, &pidSumEF, angle);
        anglesBF.v[AI_ROLL]  = -pidBodyFrame.y; // negate: body Y is leftward, positive roll is rightward
        anglesBF.v[AI_PITCH] = -pidBodyFrame.x; // negate: body X is forward, positive pitch is nose-up (rearward)

        const float mag = vector2Norm(&anglesBF);
        if (mag > ap.maxAngle && mag > 0.0f) {
            vector2Scale(&anglesBF, &anglesBF, ap.maxAngle / mag);
        }
    }

    const float stopDistanceEast = est->position.v[ENU_E] - targetPosition.v[ENU_E];
    const float stopDistanceNorth = est->position.v[ENU_N] - targetPosition.v[ENU_N];
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 0, lrintf(sqrtf(stopDistanceEast * stopDistanceEast + stopDistanceNorth * stopDistanceNorth)));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 1, lrintf(speedXY));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 2, ap.sticksActive ? 1 : 0);
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 3, navActive ? 1 : 0);
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 4, isPositionHeld ? 1 : 0);

    ap.pidSumBF = anglesBF;

    for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
        autopilotAngle[i] = pt3FilterApply(&ap.upsampleLpfBF[i], ap.pidSumBF.v[i]);
    }

    DEBUG_SET(DEBUG_AUTOPILOT_PID, 6, lrintf(autopilotAngle[X] * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, lrintf(autopilotAngle[Y] * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 6, lrintf(autopilotAngle[X] * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 7, lrintf(autopilotAngle[Y] * 100));

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

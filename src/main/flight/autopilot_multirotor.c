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
#include "sensors/gyro.h"

#include "pg/autopilot.h"
#include "autopilot.h"
#include "flight/position_nav.h"

#ifdef USE_POSITION_HOLD
#include "flight/pos_hold.h"
#endif

#ifndef POSHOLD_TASK_RATE_HZ
#define POSHOLD_TASK_RATE_HZ 100
#endif

#define ALTITUDE_P_SCALE       0.0085f
#define ALTITUDE_I_SCALE       0.00015f
#define ALTITUDE_D_SCALE       0.005f
#define ALTITUDE_F_SCALE       0.005f

// Using optical flow PID scales as the unified set
#define POSITION_P_SCALE       0.0033f
#define POSITION_I_SCALE       0.0007f
#define POSITION_II_SCALE      (0.12f * POSITION_I_SCALE)
#define POSITION_D_SCALE       0.00011f

#define POSITION_IWINDUP_LIMIT 250.0f
// Horizontal speed below this (cm/s) ends braking and snaps hold point; fusion noise may need 5–20.
#define SETTLING_VELOCITY_THRESHOLD 5.0f

#define UPSAMPLING_CUTOFF_HZ   5.0f

static pidCoefficient_t altitudePidCoeffs;
static pidCoefficient_t positionPidCoeffs;

// When autopilot hoverThrottle PG is 0, altitude hold captures rcCommand[THROTTLE] on mode entry.
#define AP_HOVER_THROTTLE_CAPTURE_MIN 1100U
#define AP_HOVER_THROTTLE_CAPTURE_MAX 1700U
static uint16_t altHoldCapturedHoverPwm;

static float altitudeI = 0.0f;
static float throttleOut = 0.0f;

// Per-axis position PID state (earth frame)
typedef enum {
    EF_EAST = 0,
    EF_NORTH
} efAxis_e;

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

void resetPositionControl(unsigned taskRateHz)
{
    ap.sticksActive = false;

    // Set sanity distance from current speed
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float speedXY = sqrtf(est->velocity.x * est->velocity.x + est->velocity.y * est->velocity.y);
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
    targetPosition.x = est->position.x;
    targetPosition.y = est->position.y;
    targetPosition.z = est->position.z;
    isPositionHeld = true;

    positionNavReset();
    wasNavActive = false;
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

    const float hoverOffset = (float)autopilotGetEffectiveHoverThrottlePwm() - PWM_RANGE_MIN;
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
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 6, lrintf(altitudeD));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, lrintf(altitudeF));
}

void setSticksActiveStatus(bool areSticksActive)
{
    ap.sticksActive = areSticksActive;
}

bool positionControl(void)
{
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const float dt = HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);

    if (!est->isValidXY) {
        return false;
    }

    // Run the navigation outer loop (no-op when no target is set)
    positionNavUpdate(dt, est);
    const bool navActive = positionNavHasActiveTarget() && !positionNavTargetReached();

    // Smooth transition: nav just completed/cleared → seed position hold target
    if (!navActive && wasNavActive) {
        targetPosition.x = est->position.x;
        targetPosition.y = est->position.y;
        targetPosition.z = est->position.z;
        for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
            posSlowIntegral[i] = 0.0f;
        }
        isPositionHeld = true;
        ap.sanityCheckDistance = sanityCheckDistance(1000.0f);
    }
    wasNavActive = navActive;

    const float velEast  = est->velocity.x;
    const float velNorth = est->velocity.y;
    const float speedXY  = sqrtf(velEast * velEast + velNorth * velNorth);
    const float velocities[EF_AXIS_COUNT] = { velEast, velNorth };

    vector2_t pidSumEF = {{ 0, 0 }};
    float distanceCm = 0.0f;
    float errorEast = 0.0f;

    if (navActive) {
        // --- Nav mode: inner velocity-tracking PID ---
        const vector3_t tgtVel = positionNavGetTargetVelocityCmS();
        const float velErrors[EF_AXIS_COUNT] = {
            velEast - tgtVel.x,
            velNorth - tgtVel.y
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
        errorEast = est->position.x - targetPosition.x;
        const float errorNorth = est->position.y - targetPosition.y;
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

    // Handle sticks and hold capture
    vector2_t anglesBF;

    if (ap.sticksActive) {
        anglesBF = (vector2_t){{0, 0}};
        if (!navActive) {
            isPositionHeld = false;
            for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
                posSlowIntegral[axis] = 0.0f;
            }
            targetPosition.x = est->position.x;
            targetPosition.y = est->position.y;
            targetPosition.z = est->position.z;
            ap.sanityCheckDistance = sanityCheckDistance(speedXY);
        }
    } else {
        if (!navActive) {
            if (!isPositionHeld) {
                if (speedXY < SETTLING_VELOCITY_THRESHOLD) {
                    isPositionHeld = true;
                    targetPosition.x = est->position.x;
                    targetPosition.y = est->position.y;
                    targetPosition.z = est->position.z;
                    ap.sanityCheckDistance = sanityCheckDistance(1000.0f);
                }
            }
        }

        // Rotate ENU PID output to body frame
        // attitude.values.yaw increases clockwise from north (BF convention)
        // ENU: X=East, Y=North.  Rotation to body frame:
        const float angle = DECIDEGREES_TO_RADIANS(attitude.values.yaw - 900);
        vector2_t pidBodyFrame;
        vector2Rotate(&pidBodyFrame, &pidSumEF, angle);
            anglesBF.v[AI_ROLL]  = -pidBodyFrame.y;
            anglesBF.v[AI_PITCH] = -pidBodyFrame.x;

        // Limit angle vector
        const float mag = vector2Norm(&anglesBF);
        if (mag > ap.maxAngle && mag > 0.0f) {
            vector2Scale(&anglesBF, &anglesBF, ap.maxAngle / mag);
        }
    }

    ap.pidSumBF = anglesBF;

    // PT3 upsampling to 100Hz for the PID loop
    for (unsigned i = 0; i < RP_AXIS_COUNT; i++) {
        autopilotAngle[i] = pt3FilterApply(&ap.upsampleLpfBF[i], ap.pidSumBF.v[i]);
    }

    DEBUG_SET(DEBUG_AUTOPILOT_PID, 6, lrintf(autopilotAngle[X] * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, lrintf(autopilotAngle[Y] * 100));

    if (navActive) {
        const positionNavCommand_t *navCmd = positionNavGetActiveCommand();
        const float dxM = est->position.x * 0.01f - navCmd->targetPosEfM.x;
        const float dyM = est->position.y * 0.01f - navCmd->targetPosEfM.y;
        const float dzM = est->position.z * 0.01f - navCmd->targetPosEfM.z;
        const float navDistCm = (navCmd->includeAltitude
            ? sqrtf(dxM * dxM + dyM * dyM + dzM * dzM)
            : sqrtf(dxM * dxM + dyM * dyM)) * 100.0f;
        const vector3_t tgtVel = positionNavGetTargetVelocityCmS();

        DEBUG_SET(DEBUG_POSITION_NAV, 0, lrintf(navDistCm));
        DEBUG_SET(DEBUG_POSITION_NAV, 1, lrintf(tgtVel.x));
        DEBUG_SET(DEBUG_POSITION_NAV, 2, lrintf(tgtVel.y));
        DEBUG_SET(DEBUG_POSITION_NAV, 3, lrintf(velEast));
        DEBUG_SET(DEBUG_POSITION_NAV, 4, lrintf(velNorth));
        DEBUG_SET(DEBUG_POSITION_NAV, 5, lrintf(navCmd->targetPosEfM.x * 100.0f));
        DEBUG_SET(DEBUG_POSITION_NAV, 6, lrintf(navCmd->targetPosEfM.y * 100.0f));
        DEBUG_SET(DEBUG_POSITION_NAV, 7, lrintf(tgtVel.z));

        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(navDistCm));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(tgtVel.x));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(pidSumEF.v[0] * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[0] * 10));
    } else {
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceCm));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(errorEast));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(pidSumEF.v[0] * 10));
        DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[0] * 10));
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

#endif // !USE_WING

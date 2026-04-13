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

#ifndef POSHOLD_TASK_RATE_HZ
#define POSHOLD_TASK_RATE_HZ 100
#endif

#define ALTITUDE_D_SCALE       0.005f
#define ALTITUDE_F_SCALE       0.005f

// Cascaded alt hold: outer loop altitude (P+I) -> vz cmd; inner loop velocity (P+I) -> throttle.
// CLI altitudeP/I/D/F map to outer P, outer+inner I split, inner P (velocity), and feedforward gain on vz pilot input.
#define ALTITUDE_OUTER_P_SCALE       0.04f     // (cm/s) / (cm) per altitudeP unit
#define ALTITUDE_OUTER_I_SCALE       0.0008f // scale on altitudeI for outer integral (cm/s bias / accumulated from pos err)
#define ALTITUDE_INNER_I_SCALE       0.00012f // scale on altitudeI for inner integral (throttle, similar order to legacy ALTITUDE_I_SCALE)
#define ALTITUDE_VEL_CMD_MAX_DEFAULT_CM_S  1500.0f
#define ALTITUDE_OUTER_IWINDUP_CM_S      150.0f
#define ALTITUDE_INNER_IWINDUP_THROTTLE  200.0f
#define ALTITUDE_FF_KF_REF               (30.0f * ALTITUDE_F_SCALE) // "100%" feedforward when altitudeF CLI = 30

// Using optical flow PID scales as the unified set
#define POSITION_P_SCALE       0.0033f
#define POSITION_I_SCALE       0.0007f
#define POSITION_II_SCALE      (0.12f * POSITION_I_SCALE)
#define POSITION_D_SCALE       0.00011f

#define POSITION_IWINDUP_LIMIT 250.0f
// Horizontal speed below this (cm/s) ends braking and snaps hold point; fusion noise may need 5–20.
#define SETTLING_VELOCITY_THRESHOLD 5.0f

#define UPSAMPLING_CUTOFF_HZ   5.0f

static pidCoefficient_t positionPidCoeffs;

static float altitudeOuterKp;
static float altitudeOuterKi;
static float altitudeInnerKp;
static float altitudeInnerKi;
static float altitudeFfKfNorm; // scales pilot / rescue vz feedforward vs CLI altitudeF (1.0 at F=30)

// When autopilot hoverThrottle PG is 0, altitude hold captures rcCommand[THROTTLE] on mode entry.
#define AP_HOVER_THROTTLE_CAPTURE_MIN 1100U
#define AP_HOVER_THROTTLE_CAPTURE_MAX 1700U
static uint16_t altHoldCapturedHoverPwm;

static float altitudePosIntegral = 0.0f; // outer I: integral of altitude error -> vz bias (cm/s)
static float altitudeVelIntegral = 0.0f; // inner I: integral of velocity error -> throttle (us offset scale)
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
    isPositionHeld = true;
}

void autopilotInit(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    ap.sticksActive = false;
    ap.maxAngle = cfg->maxAngle;

    altitudeOuterKp = cfg->altitudeP * ALTITUDE_OUTER_P_SCALE;
    altitudeOuterKi = cfg->altitudeI * ALTITUDE_OUTER_I_SCALE;
    altitudeInnerKp = cfg->altitudeD * ALTITUDE_D_SCALE;
    altitudeInnerKi = cfg->altitudeI * ALTITUDE_INNER_I_SCALE;
    if (cfg->altitudeF == 0) {
        altitudeFfKfNorm = 1.0f; // treat as full-rate feedforward from stick / rescue vz
    } else {
        altitudeFfKfNorm = (cfg->altitudeF * ALTITUDE_F_SCALE) / ALTITUDE_FF_KF_REF;
    }

    positionPidCoeffs.Kp  = cfg->positionP  * POSITION_P_SCALE;
    positionPidCoeffs.Ki  = cfg->positionI  * POSITION_I_SCALE;
    positionPidCoeffs.Kii = cfg->positionII * POSITION_II_SCALE;
    positionPidCoeffs.Kd  = cfg->positionD  * POSITION_D_SCALE;

    ap.upsampleLpfGain = pt3FilterGain(UPSAMPLING_CUTOFF_HZ, 0.01f);
    resetUpsampleFilters();
    initPositionAccelLpf();
}

void resetAltitudeControl(void)
{
    altitudePosIntegral = 0.0f;
    altitudeVelIntegral = 0.0f;
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
    const float vz = getAltitudeDerivativeControl();
    const float altitudeErrorCm = targetAltitudeCm - getAltitudeCmControl();

    // increase inner-loop P when |vz| is high (same shaping as legacy D boost)
    float dBoost = 1.0f;
    {
        const float startValue = 500.0f;
        const float altDeriv = fabsf(vz);
        if (altDeriv > startValue) {
            const float ratio = altDeriv / startValue;
            dBoost = (3.0f * ratio - 2.0f) / ratio;
        }
    }
    const float innerKp = altitudeInnerKp * dBoost;

    // Outer loop: PI on altitude -> vertical velocity setpoint (cm/s)
    const float itermRelax = (fabsf(altitudeErrorCm) < 200.0f) ? 1.0f : 0.1f;
    altitudePosIntegral += altitudeErrorCm * altitudeOuterKi * itermRelax * taskIntervalS;
    altitudePosIntegral = constrainf(altitudePosIntegral, -ALTITUDE_OUTER_IWINDUP_CM_S, ALTITUDE_OUTER_IWINDUP_CM_S);

    const float velMax = (velLimitCmS > 1.0f) ? velLimitCmS : ALTITUDE_VEL_CMD_MAX_DEFAULT_CM_S;
    float velCmd = targetAltitudeVelCmS * altitudeFfKfNorm;
    velCmd += altitudeErrorCm * altitudeOuterKp + altitudePosIntegral;
    velCmd = constrainf(velCmd, -velMax, velMax);

    // Inner loop: PI on velocity -> throttle offset (legacy D gain scale: throttle per cm/s)
    const float velErr = velCmd - vz;
    altitudeVelIntegral += velErr * altitudeInnerKi * taskIntervalS;
    altitudeVelIntegral = constrainf(altitudeVelIntegral, -ALTITUDE_INNER_IWINDUP_THROTTLE, ALTITUDE_INNER_IWINDUP_THROTTLE);

    const float innerP = velErr * innerKp;
    const float hoverOffset = (float)autopilotGetEffectiveHoverThrottlePwm() - PWM_RANGE_MIN;
    float throttleOffset = innerP + altitudeVelIntegral + hoverOffset;

    const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
    throttleOffset *= tiltMultiplier;

    float newThrottle = PWM_RANGE_MIN + throttleOffset;
    newThrottle = constrainf(newThrottle, autopilotConfig()->throttleMin, autopilotConfig()->throttleMax);
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 0, lrintf(newThrottle));

    newThrottle = scaleRangef(newThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    throttleOut = constrainf(newThrottle, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(tiltMultiplier * 100));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 3, lrintf(targetAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 4, lrintf(velCmd));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 5, lrintf(altitudePosIntegral));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 6, lrintf(innerP));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, lrintf(altitudeVelIntegral));
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

    // Position error in ENU earth frame (cm)
    const float errorEast  = est->position.x - targetPosition.x;
    const float errorNorth = est->position.y - targetPosition.y;
    const float distanceCm = sqrtf(errorEast * errorEast + errorNorth * errorNorth);

    // Sanity check: detect flyaway
    if (distanceCm > ap.sanityCheckDistance) {
        return false;
    }

    // Velocity from KF (already filtered, earth frame, cm/s)
    const float velEast  = est->velocity.x;
    const float velNorth = est->velocity.y;
    const float speedXY  = sqrtf(velEast * velEast + velNorth * velNorth);

    // Run PID for each earth-frame axis
    const float errors[EF_AXIS_COUNT]     = { errorEast, errorNorth };
    const float velocities[EF_AXIS_COUNT] = { velEast,   velNorth };
    vector2_t pidSumEF = {{ 0, 0 }};

    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
        const float velocity = velocities[axis];

        // P - acts on velocity (damping)
        const float pidP = velocity * positionPidCoeffs.Kp;

        // I - position error spring (disabled during braking; enabled again after hold point capture)
        float pidI = 0.0f;
        // II - slow integral of position error (disabled during braking; enabled again after hold point capture)
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

        // D - acts on acceleration (change in velocity), low-passed to avoid differentiator noise
        const float accelRaw = (velocity - previousVelocity[axis]) / dt;
        previousVelocity[axis] = velocity;
        const float accel = pt1FilterApply(&posAccelLpf[axis], accelRaw);
        const float pidD = -accel * positionPidCoeffs.Kd;

        pidSumEF.v[axis] = pidP + pidI + pidII + pidD;

        DEBUG_SET(DEBUG_AUTOPILOT_PID, 0 + axis, lrintf(pidP * 100));
        DEBUG_SET(DEBUG_AUTOPILOT_PID, 2 + axis, lrintf(pidI * 100));
        DEBUG_SET(DEBUG_AUTOPILOT_PID, 4 + axis, lrintf(pidII * 100));
    }

    // Handle sticks and stopping logic
    vector2_t anglesBF;

    if (ap.sticksActive) {
        anglesBF = (vector2_t){{0, 0}};
        isPositionHeld = false;
        ap.sanityCheckDistance = sanityCheckDistance(speedXY);
    } else {
        // Braking: hold target fixed until horizontal speed drops, then capture hold point.
        if (!isPositionHeld) {
            if (speedXY < SETTLING_VELOCITY_THRESHOLD) {
                isPositionHeld = true;
                targetPosition.x = est->position.x;
                targetPosition.y = est->position.y;
                ap.sanityCheckDistance = sanityCheckDistance(1000.0f);
            }
        }
        // Do not move targetPosition while braking: hold point stays at stick-release until capture.

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
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 0, lrintf(distanceCm));
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 1, lrintf(errorEast));
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 2, lrintf(pidSumEF.v[0] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_POSITION, 3, lrintf(autopilotAngle[0] * 10));

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

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
#include "fc/rc_controls.h"
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
// 4 - D term * 10 // damping on measured velocity
// 5 - A term * 10 // velocity derivative factor (acceleration in distance terms)
// 6 - F term * 10 // target-velocity feedforward (the stick push / nav target)
// 7 - Status - encodes navActive+ 10, velocityMode +20, SticksActive +5, PositionHeld +3, +1 when braking,
// In velocity mode slots 2-5 carry the velocity-loop terms: P/I on velocity error,
// D damping, A the drag feedforward; slot 6 (F) reads ~0. See also DEBUG_POSITION_NAV.

// DEBUG_POSITION_NAV, axis set by gyro_filter_debug_axis
// 0 - target velocity cm/s
// 1 - filtered measured velocity cm/s
// 2 - velocity error cm/s
// 3 - P term * 10 (post buildup clamp)
// 4 - I term * 10
// 5 - D term * 10 (damping)
// 6 - drag feedforward * 10
// 7 - velocityMode * 10, +1 while the buildup clamp engages

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
#define ALTITUDE_F_KF_REF     30.0f
#define ALTITUDE_F_SCALE       0.1f / ALTITUDE_F_KF_REF // full feedforward scale value when altitudeF CLI = 30
#define ALTITUDE_VEL_CMD_MAX_DEFAULT_CM_S  1500.0f
#define ALTITUDE_I_LIMIT      150.0f

#define AP_YAW_P_SCALE         0.01f
#define AP_YAW_D_SCALE         0.01f
#define AP_YAW_RAMP_TIME_S     1.0f

#define XY_DISTANCE_SCALE    0.0015f   // distance P / velocity I
#define XY_DISTANCE_I_SCALE  0.00015f  // distance I
#define XY_VELOCITY_SCALE    0.002f    // distance D / velocity P (opposes velocity)
#define XY_ACCEL_SCALE       0.0005f   // distance Acceleration / velocity D
#define XY_F_SCALE           0.005f / POSHOLD_TASK_RATE_HZ   //  velocity target delta scale factor
#define XY_DRAG_SCALE        0.0001f  //  velocity based drag correction factor


#define POSHOLD_VELOCITY_REVERSAL_THRESHOLD 50.0f // dotcross reversal greater than this will terminate braking mode
#define SANITY_CHECK_DISTANCE 2000.0f        // 20m, increased when stopping from speeds above 10m/s
#define ERROR_DISTANCE_LIMIT  2000.0f        // TO DO: test set to a useful value, this is 20m
#define POSITION_I_LIMIT      2000.0f        // TO DO: test and set to a useful value, this is 20m
#define XY_VELOCITY_I_RELAX_CMS   250.0f     // in Nav mode only, integrate only near the target speed, so the
                                             // integral cannot wind up during the accel phase
static pidCoefficient_t xyPid;
static float xyKDrag = 0.0f;

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
static vector2_t targetVelocity;
static vector2_t previousTargetVelocity; // for feedforward when target velocity changes
static vector2_t posHoldStartPosition;
static vector2_t distanceError;          // deviation from intended position
static vector2_t distanceErrorIntegral;  // integral of position error
static vector2_t previousVelocity;       // for acceleration

static pt2Filter_t posAccelLpf[EF_AXIS_COUNT];
static pt2Filter_t posPidSumLpf[EF_AXIS_COUNT];

static bool isPositionHeld;
static bool wasPositionHeld = false;
static bool wasNavActive = false;
static bool abortNavRequested = false;
static bool forcePitchForward = false;
static bool forceLevelPark = false;
static bool wasAngleSaturated = false;

static float apYawRateDps = 0.0f;
static bool apYawActive = false;
static float apYawAttenuator = 0.0f;
static float apYawRateLimitDps = 0.0f;
static bool apYawCourseValid = false;
static bool apNavHeadingOverrideValid = false;   // mission pre-turn: nose commanded onto the next leg
static float apNavHeadingOverrideDeg = 0.0f;

static void disableYawControl(void);

typedef struct autopilotState_s {
    float sanityCheckDistance;
    bool sticksActive;
    bool wasSticksActive;
    bool navActive;
    float maxAngle;
    float speedXY;              // horizontal ground speed this loop, cm/s
    bool isPosHoldBraking;      // decelerating toward a captured hold point
    unsigned debugAxis;
} autopilotState_t;

float autopilotAngle[RP_AXIS_COUNT];

static autopilotState_t ap = {
    .sanityCheckDistance = SANITY_CHECK_DISTANCE,
    .sticksActive = false,
    .wasSticksActive = false,
};

static void initPidLpfs(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    const float cutoffHz = fmaxf(cfg->positionCutoff * 0.1f, 0.1f); // default of 30 is 3Hz, range 1 to 25Hz
    const float k = pt2FilterGain(cutoffHz, HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ));
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        pt2FilterInit(&posAccelLpf[i], k);
        pt2FilterInit(&posPidSumLpf[i], k);
    }
}

void autopilotInit(void)
{
    const autopilotConfig_t *cfg = autopilotConfig();
    initPidLpfs();

    ap.maxAngle = cfg->maxAngle;
    ap.debugAxis = (gyroConfig()->gyro_filter_debug_axis == FD_PITCH) ? 1 : 0; // 1 for Pitch / North, 0 for Roll / East

    altitudeKp = cfg->altitudeP * ALTITUDE_P_SCALE;
    altitudeKi = cfg->altitudeI * ALTITUDE_I_SCALE;
    altitudeKd = cfg->altitudeD * ALTITUDE_D_SCALE;
    altitudeKf = cfg->altitudeF * ALTITUDE_F_SCALE;

    xyPid.Kp = cfg->positionP  * XY_DISTANCE_SCALE;
    xyPid.Ki = cfg->positionI  * XY_DISTANCE_I_SCALE;
    xyPid.Kd = cfg->positionD  * XY_VELOCITY_SCALE;
    xyPid.Ka = cfg->positionA  * XY_ACCEL_SCALE;
    xyPid.Kf = cfg->positionF  * XY_F_SCALE;
    xyKDrag =  cfg->velocityDragCoeff * XY_DRAG_SCALE;

    ap.sticksActive = false;
    ap.wasSticksActive = false;
    ap.speedXY = 0.0f;
    ap.isPosHoldBraking = false;
    abortNavRequested = false;
    forcePitchForward = false;
    forceLevelPark = false;
    apNavHeadingOverrideValid = false;
    disableYawControl();
    apYawRateLimitDps = 0.0f;
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
    // PID controller on altitude error
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

void moveTargetLocation(const vector2_t *stepEF, unsigned taskRateHz, bool forceAbortRequest)
{
    if (forceAbortRequest) {
        abortNavRequested = true;
    } else {
        // Force the flag back to false when a normal tracking pass runs
        abortNavRequested = false;

        if (stepEF != NULL) {
            targetPosition.v[EF_EAST]  += stepEF->v[EF_EAST];
            targetPosition.v[EF_NORTH] += stepEF->v[EF_NORTH];
            targetVelocity.v[EF_EAST]  = stepEF->v[EF_EAST] * taskRateHz;
            targetVelocity.v[EF_NORTH] = stepEF->v[EF_NORTH] * taskRateHz;
            posHoldStartPosition = targetPosition; // update start point to new target to prevent poshold sanity failure
        }
    }
}

void pitchForwardOverride(bool request)
{
    forcePitchForward = request;
}

void autopilotForceLevelPark(bool request)
{
    forceLevelPark = request;
}

void autopilotSetNavHeadingOverride(bool valid, float headingDeg)
{
    apNavHeadingOverrideValid = valid;
    apNavHeadingOverrideDeg = headingDeg;
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

void initPositionHold(void)
{
    updatePositionHoldTarget();
    resetDistanceError();
    targetVelocity.v[EF_EAST]  = 0.0f;
    targetVelocity.v[EF_NORTH] = 0.0f;
    previousTargetVelocity.v[EF_EAST]  = 0.0f;
    previousTargetVelocity.v[EF_NORTH] = 0.0f;

    ap.isPosHoldBraking = true; // arrest any entry speed before capturing the hold point
    // nb: we do not reset the distanceError integral, to hold its opposition to wind between quick stick inputs
}

static void initNavMode(void)
{
    initPidLpfs();
    resetDistanceError();
    resetDistanceErrorIntegral();
    resetDistanceError();
    previousTargetVelocity.v[EF_EAST]  = 0.0f;
    previousTargetVelocity.v[EF_NORTH] = 0.0f;
    ap.isPosHoldBraking = false;
}

void resetPositionControl(unsigned taskRateHz)
{
    UNUSED(taskRateHz);
    abortNavRequested = false;
    forcePitchForward = false;
    forceLevelPark = false;
    apNavHeadingOverrideValid = false;
    ap.sticksActive = false;
    ap.wasSticksActive = false;
    disableYawControl();
    apYawCourseValid = false;
    wasAngleSaturated = false;
    // Initialise the nav system
    positionEstimatorEnableXY(true);
    positionNavReset();
    wasNavActive = false; // will be enabled as required
    ap.sanityCheckDistance = calculateSanityCheckDistance(); // Set an initial sanity check distance
    initPositionHold(); // sets target location, resets distance error, enables start mode
    previousVelocity = *(const vector2_t *)&positionEstimatorGetEstimate()->velocity.v; // for smooth A in any mode
    resetDistanceErrorIntegral();
    resetDistanceError();
}

void handlepositionControlFailure(void)
{
    resetDistanceError();
    resetDistanceErrorIntegral();
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, 100);
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 6, 100);
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 7, 100);

}

void sticksMoveTarget(void)
{
    float stickPitch = getSetpointRate(PITCH);
    float stickRoll  = getSetpointRate(ROLL);

    const float headingRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    const float cosYaw = cosf(headingRad);
    const float sinYaw = sinf(headingRad);

    targetVelocity.v[EF_NORTH] = (stickPitch * cosYaw) - (stickRoll * sinYaw);
    targetVelocity.v[EF_EAST]  = (stickPitch * sinYaw) + (stickRoll * cosYaw);

    const float dt = HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);
    targetPosition.v[EF_NORTH] += targetVelocity.v[EF_NORTH] * dt;
    targetPosition.v[EF_EAST]  += targetVelocity.v[EF_EAST] * dt;

    posHoldStartPosition = targetPosition;
}

void autopilotSetYawRateLimit(float rateLimitDps)
{
    apYawRateLimitDps = rateLimitDps;
}

float autopilotGetYawRate(void)
{
    return apYawRateDps;
}

bool autopilotYawControlActive(void)
{
    return apYawActive;
}

static void disableYawControl(void)
{
    apYawActive = false;
    apYawAttenuator = 0.0f;
    apYawRateDps = 0.0f;
}

static bool courseHeadingDeg(const positionEstimate3d_t *est, float *headingDeg)
{
    const vector2_t *velocity = (const vector2_t *)&est->velocity.v;
    // Hysteresis so speed noise around the gate doesn't repeatedly drop the
    // controller (and restart its engage ramp): release at 75% of engage.
    const float engageCmS = (float)autopilotConfig()->minForwardVelocity;
    const float gateCmS = apYawCourseValid ? engageCmS * 0.75f : engageCmS;
    apYawCourseValid = vector2Norm(velocity) >= gateCmS;
    if (!apYawCourseValid) {
        return false;
    }
    *headingDeg = RADIANS_TO_DEGREES(atan2_approx(velocity->v[EF_EAST], velocity->v[EF_NORTH]));
    return true;
}

static bool bearingToTargetDeg(const positionEstimate3d_t *est, float *headingDeg)
{
    const positionNavCommand_t *cmd = positionNavGetActiveCommand();
    if (cmd == NULL || !cmd->active) {
        return false;
    }
    const float deltaEastCm  = cmd->targetPosEfM.v[ENU_E] * 100.0f - est->position.v[ENU_E];
    const float deltaNorthCm = cmd->targetPosEfM.v[ENU_N] * 100.0f - est->position.v[ENU_N];
    // Inside the acceptance radius the bearing degenerates; stop steering.
    if (sqrtf(sq(deltaEastCm) + sq(deltaNorthCm)) <= cmd->acceptanceRadiusM * 100.0f) {
        return false;
    }
    *headingDeg = RADIANS_TO_DEGREES(atan2_approx(deltaEastCm, deltaNorthCm));
    return true;
}

// Mission yaw: while navigation is flying a leg, steer the nose to the ground
// course (VELOCITY), the bearing to the active target (BEARING), or course
// falling back to bearing when too slow for a reliable course (HYBRID).
// P on wrapped heading error with a short engage ramp and gyro damping,
// clamped to ap_max_yaw_rate and any YAW_RATE mission cap; rc.c injects the
// resulting rate as the yaw setpoint, exactly as GPS rescue injects its own.
static void updateYawControl(float dt, const positionEstimate3d_t *est)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    if (!FLIGHT_MODE(AUTOPILOT_MODE) || !ap.navActive) {
        disableYawControl();
        return;
    }

    float desiredHeadingDeg = 0.0f;
    bool haveDesiredHeading = false;
    if (apNavHeadingOverrideValid) {
        // Mission pre-turn blend: point the nose onto the next leg regardless of
        // the configured yaw mode, so it is already there as the gate is crossed.
        desiredHeadingDeg = apNavHeadingOverrideDeg;
        haveDesiredHeading = true;
    } else {
        switch (cfg->yawMode) {
        case YAW_MODE_VELOCITY:
            haveDesiredHeading = courseHeadingDeg(est, &desiredHeadingDeg);
            break;
        case YAW_MODE_BEARING:
            haveDesiredHeading = bearingToTargetDeg(est, &desiredHeadingDeg);
            break;
        case YAW_MODE_HYBRID:
            haveDesiredHeading = courseHeadingDeg(est, &desiredHeadingDeg)
                || bearingToTargetDeg(est, &desiredHeadingDeg);
            break;
        default: // YAW_MODE_FIXED, YAW_MODE_DAMPENER (wing only)
            break;
        }
    }

    if (!haveDesiredHeading) {
        disableYawControl();
        return;
    }

    apYawAttenuator = fminf(apYawAttenuator + dt / AP_YAW_RAMP_TIME_S, 1.0f);

    // The yaw rate setpoint (and gyro) is CCW-positive while compass headings
    // are CW-positive, so the heading error enters the setpoint frame negated:
    // desired ahead of heading (a right turn) demands a negative rate.
    float errorDeg = attitude.values.yaw * 0.1f - desiredHeadingDeg;
    errorDeg = fmodf(errorDeg + 540.0f, 360.0f) - 180.0f;

    float yawRateDps = errorDeg * cfg->yawP * AP_YAW_P_SCALE
                     - gyro.gyroADCf[FD_YAW] * cfg->yawD * AP_YAW_D_SCALE;
    yawRateDps *= apYawAttenuator;

    float maxRateDps = (float)cfg->maxYawRate;
    if (apYawRateLimitDps > 0.0f) {
        maxRateDps = fminf(maxRateDps, apYawRateLimitDps);
    }
    yawRateDps = constrainf(yawRateDps, -maxRateDps, maxRateDps);

    apYawRateDps = yawRateDps * GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
    apYawActive = true;
}

bool positionControl(void)
{

    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    const timeDelta_t posholdDtUs = getTaskDeltaTimeUs(TASK_SELF);
    const float dt = (posholdDtUs > 0) ? (posholdDtUs * 1e-6f) : HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);
    static int brakingTimer = 0;
    
    if (!est->isValidXY) {
        disableYawControl();
        return false;
    }
    if (abortNavRequested) {
        disableYawControl();
        handlepositionControlFailure();
        return false; // Return failure and show pos hold fail message in OSD
    }
    if (forceLevelPark) {
        // Heading/mag fault: position hold would lean on the suspect heading and
        // fly sideways, so drop to angle-mode self-level (altitude hold, a
        // separate mode, keeps holding height) until a mode-switch cycle clears it.
        disableYawControl();
        handlepositionControlFailure();
        return false;
    }
    if (forcePitchForward) {
        disableYawControl();
        autopilotAngle[AI_ROLL]  = 0.0f;
        autopilotAngle[AI_PITCH] = 35.0f;
        DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, 200);
        DEBUG_SET(DEBUG_AUTOPILOT_STOP, 6, 200);
        DEBUG_SET(DEBUG_AUTOPILOT_STOP, 7, 200);
        return true;
    }
    const vector2_t currentPosition = *(const vector2_t *)&est->position.v;
    const vector2_t velocity = *(const vector2_t *)&est->velocity.v;
    vector2_t pidSumVectorEF     = { { 0 } };
    vector2_t velocityError      = { { 0 } };
    // PIDs are distance based, so D is a Velocity-proportional factor, A is acceleration-proportional, etc
    vector2_t pidP               = { { 0 } };
    vector2_t pidI               = { { 0 } };
    vector2_t pidD               = { { 0 } };
    vector2_t pidA               = { { 0 } };
    vector2_t pidF               = { { 0 } };
    // Update navigation status
    positionNavUpdate(dt, est);
    ap.navActive = positionNavHasActiveTarget() && !positionNavTargetReached();

    ap.speedXY = vector2Norm(&velocity);
    if (ap.navActive) {
        isPositionHeld = false;
        if (!wasNavActive) {
            initNavMode();
        }
        const vector3_t tgtVel = positionNavGetTargetVelocityCmS();
        targetVelocity = *(const vector2_t *)&tgtVel.v;
        ap.isPosHoldBraking = false; // nav sequences its own speed
    } else {
        // Control mode should be position hold
        if (!isPositionHeld) {
            initPositionHold();
            brakingTimer = 0;
            ap.sanityCheckDistance = calculateSanityCheckDistance();
            isPositionHeld = true;
        }
        if (ap.sticksActive) {
            if (!ap.wasSticksActive) {
                // initialise sticksActive
                updatePositionHoldTarget();
                ap.sanityCheckDistance = calculateSanityCheckDistance();
                ap.isPosHoldBraking = false;
            }
            sticksMoveTarget(); //move target position and generate associated velocityTargets
        } else {
            // No stick input
            targetVelocity.v[EF_EAST]  = 0.0f;
            targetVelocity.v[EF_NORTH] = 0.0f;
            if (ap.wasSticksActive) {
                // Sticks were active and have just been released: capture the current point and begin braking.
                updatePositionHoldTarget();
                ap.sanityCheckDistance = calculateSanityCheckDistance();
                ap.isPosHoldBraking = true;
                brakingTimer = 0;
            }
            if (ap.isPosHoldBraking) {
                targetPosition = currentPosition; // Drag target location every frame while braking to avoid P and I climb
                brakingTimer += 1; //
                if (brakingTimer > POSHOLD_TASK_RATE_HZ) { // one second, might be better at 1.5 or 2.0s but this is pretty good
                    brakingTimer = POSHOLD_TASK_RATE_HZ;
                }
                // Braking mode; ends when velocity is below the stop threshold or when the dominant velocity vector reverses sign
                const float vectorDotProduct = (velocity.v[EF_NORTH] * previousVelocity.v[EF_NORTH]) + 
                                               (velocity.v[EF_EAST]  * previousVelocity.v[EF_EAST]);
                const bool velocityCrossingZero = vectorDotProduct < -POSHOLD_VELOCITY_REVERSAL_THRESHOLD; // some deadband
                const bool stopped = (ap.speedXY < (float)autopilotConfig()->stopThreshold);
                const bool timedOut = brakingTimer >= POSHOLD_TASK_RATE_HZ;
                if (stopped || timedOut || velocityCrossingZero) {
                    updatePositionHoldTarget(); // capture the current location as the final hold  target
                    brakingTimer = 0;
                    ap.isPosHoldBraking = false;
                }
            } else {
                // Settled hold: guard against a position-estimate flyaway.
                vector2_t deltaPosV;
                vector2Sub(&deltaPosV, &posHoldStartPosition, &currentPosition);
                if (vector2Norm(&deltaPosV) > ap.sanityCheckDistance) {
                    disableYawControl();
                    autopilotAngle[AI_ROLL]  = 0.0f; // Level out
                    autopilotAngle[AI_PITCH] = 0.0f;
                    handlepositionControlFailure();
                    return false; // Return failure, allow angle mode control, and show pos hold fail message in OSD
                }
            }
        }
    }
    updateYawControl(dt, est);

    wasPositionHeld = isPositionHeld;
    wasNavActive = ap.navActive;
    ap.wasSticksActive = ap.sticksActive; // Main frame-to-frame history update
    const bool velocityMode = ap.navActive && autopilotConfig()->velocityControlEnable;

    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
        float dTermBrakingBoost = 1.0f;
        const float tgtVelAcceleration = (targetVelocity.v[axis] - previousTargetVelocity.v[axis]) * POSHOLD_TASK_RATE_HZ; //cm/s per second
        velocityError.v[axis] = targetVelocity.v[axis] - velocity.v[axis];
        const float accelerationRaw = (previousVelocity.v[axis] -  velocity.v[axis]) * POSHOLD_TASK_RATE_HZ;
        previousVelocity.v[axis] =  velocity.v[axis];
        const float acceleration = pt2FilterApply(&posAccelLpf[axis], accelerationRaw);

        bool shouldIntegrateDistanceError = true;

        if (velocityMode) {
            // Nav velocity loop: track the commanded ground velocity directly.
            distanceErrorIntegral.v[axis] = 0.0f; // force distance error integral to zero
            shouldIntegrateDistanceError = false; // do not allow second integral accumulation
            if (fabsf(velocityError.v[axis]) < XY_VELOCITY_I_RELAX_CMS
                && (!wasAngleSaturated || (velocityError.v[axis] * distanceError.v[axis]) < 0.0f)) {
                distanceError.v[axis] += velocityError.v[axis] * dt; // make a virtual distance error from integral of velocityError
            }
        } else {
            // in Position Hold Mode
            distanceError.v[axis] = targetPosition.v[axis] - currentPosition.v[axis]; 

            if (ap.isPosHoldBraking) {
                dTermBrakingBoost = 1.0f + fabsf(velocity.v[axis]) * 0.001f; // stronger D when stopping from high velocity
            } else if (ap.sticksActive) {
                // both velocity target and position target are actively changed according to setpoint
                shouldIntegrateDistanceError = false; // avoid iTerm windup, but retain current value 
            }
        }
        //these things happen in all modes
        distanceError.v[axis] = constrainf(distanceError.v[axis], -ERROR_DISTANCE_LIMIT, ERROR_DISTANCE_LIMIT);
        if (shouldIntegrateDistanceError) {
            distanceErrorIntegral.v[axis] += distanceError.v[axis] * dt;
            distanceErrorIntegral.v[axis] = constrainf(distanceErrorIntegral.v[axis], -POSITION_I_LIMIT, POSITION_I_LIMIT);
        }

        pidP.v[axis] = distanceError.v[axis] * xyPid.Kp; // P factor from distance error, real or virtual
        pidI.v[axis] = distanceErrorIntegral.v[axis] * xyPid.Ki; // integral of distance error, forced to zero when there is no hard position source
        pidD.v[axis] = -velocity.v[axis] * xyPid.Kd * dTermBrakingBoost; // damping on measured velocity
        pidD.v[axis] += velocity.v[axis] *xyKDrag; // drag compensation reduces damping
        pidA.v[axis] = acceleration * xyPid.Ka;
        pidF.v[axis] = targetVelocity.v[axis] * xyPid.Kd + tgtVelAcceleration * xyPid.Kf;
        // in F, we use Kd on the constant target velocity component to balance D when craft is at target velocity; using D on the delta provides tunable acceleration deceleration responsiveness
        pidSumVectorEF.v[axis] = pidP.v[axis] + pidI.v[axis] + pidD.v[axis] + pidA.v[axis] + pidF.v[axis];
        pidSumVectorEF.v[axis] = pt2FilterApply(&posPidSumLpf[axis], pidSumVectorEF.v[axis]);

    } // End for loop
    previousTargetVelocity = targetVelocity;

    bool buildupClamped = false;
    if (velocityMode) {
        // velocityBuildupMaxPitch bounds the velocity-proportional drive vector (now pidD)
        // so the pitch bias while building up to speed is limited.
        const float buildupMaxDeg = autopilotConfig()->velocityBuildupMaxPitch;
        const float dMag = vector2Norm(&pidD);
        if (dMag > buildupMaxDeg) {
            buildupClamped = true;
            const float scale = (dMag > 0.001f) ? buildupMaxDeg / dMag : 0.0f;
            vector2Scale(&pidD, &pidD, scale);
            // Re-sum the vectors with the scaled D component
            for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
                pidSumVectorEF.v[axis] = pidP.v[axis] + pidI.v[axis] + pidD.v[axis] + pidA.v[axis] + pidF.v[axis];
            }
        }
    }

    // Rotation from Earth Frame to Body Frame
    const float headingRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    vector2_t headingV;
    vector2_t angleV;

    headingV.v[EF_EAST]  = sinf(headingRad);
    headingV.v[EF_NORTH] = cosf(headingRad);
    angleV.v[AI_PITCH] = vector2Dot(&headingV, &pidSumVectorEF);
    angleV.v[AI_ROLL]  = vector2Cross(&headingV, &pidSumVectorEF);

    const float mag = vector2Norm(&angleV);
    wasAngleSaturated = (mag > ap.maxAngle);
    if (mag > ap.maxAngle && mag > 0.001f) {
        const float scale = ap.maxAngle / mag;
        vector2Scale(&angleV, &angleV, scale);
    }

    autopilotAngle[AI_ROLL]  = -angleV.v[AI_ROLL];
    autopilotAngle[AI_PITCH] =  angleV.v[AI_PITCH];

    int statusValue = 0;
    if (ap.navActive)       statusValue += 10;
    if (velocityMode)       statusValue += 20;
    if (abortNavRequested)  statusValue += 100;
    if (isPositionHeld)     statusValue += 3; // plus 1, ie 4,  if stopping
    if (ap.sticksActive)    statusValue += 5;
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 0, lrintf(velocity.v[ap.debugAxis])); // velocity cm/s
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 1, lrintf(distanceError.v[ap.debugAxis])); // distance error
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 2, lrintf(pidP.v[ap.debugAxis] * 10));   
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 3, lrintf(pidI.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 4, lrintf(pidD.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 5, lrintf(pidA.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 6, lrintf(pidF.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, statusValue + (ap.isPosHoldBraking ? 1 : 0));

    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 0, lrintf(velocityError.v[EF_EAST]));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 1, lrintf(velocityError.v[EF_NORTH]));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 2, lrintf(pidSumVectorEF.v[EF_EAST] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 3, lrintf(pidSumVectorEF.v[EF_NORTH] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 4, lrintf(autopilotAngle[AI_ROLL] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 5, lrintf(autopilotAngle[AI_PITCH] * 10));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 6, statusValue + (ap.isPosHoldBraking ? 1 : 0));
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 7, statusValue + (ap.isPosHoldBraking ? 1 : 0));

    DEBUG_SET(DEBUG_POSITION_NAV, 0, lrintf(targetVelocity.v[ap.debugAxis]));
    DEBUG_SET(DEBUG_POSITION_NAV, 1, lrintf(velocity.v[ap.debugAxis]));
    DEBUG_SET(DEBUG_POSITION_NAV, 2, lrintf(velocityError.v[ap.debugAxis]));
    DEBUG_SET(DEBUG_POSITION_NAV, 3, lrintf(pidP.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_POSITION_NAV, 4, lrintf(pidI.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_POSITION_NAV, 5, lrintf(pidD.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_POSITION_NAV, 6, lrintf(pidA.v[ap.debugAxis] * 10));
    DEBUG_SET(DEBUG_POSITION_NAV, 7, (velocityMode ? 10 : 0) + (buildupClamped ? 1 : 0));

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

#endif // !USE_WING

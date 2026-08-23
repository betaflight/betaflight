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
// 0 - Velocity cm/s
// 1 - DistanceError cm
// 2 - P term * 10 // based on distance from intended position
// 3 - I term * 10 // integral of distance error over time
// 4 - D term * 10 // damping on measured velocity, opposing velocity
// 5 - A term * 10 // opposes the Kalman acceleration estimate (acceleration in distance terms)
// 6 - F term * 10 // target-velocity feedforward (the stick push / nav target)
// 7 - Status - encodes navActive +10, anchorOff +20, abortNavRequested +100,
//     SticksActive +5, PositionHeld +3, +1 when braking. Also forced to 100 on a
//     position-control failure and 200 while pitch-forward is being forced.
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
// 7 - anchorOff * 10, +1 while the buildup clamp engages

// DEBUG_AUTOPILOT_STOP
// 0 - VelocityError East
// 1 - VelocityError North
// 2 - PIDSum East * 10
// 3 - PIDSum North * 10
// 4 - Roll angle command * 10
// 5 - Pitch angle command * 10
// 6 - Status, same encoding and value as slot 7
// 7 - Status - encodes navActive +10, anchorOff +20, abortNavRequested +100,
//     SticksActive +5, PositionHeld +3, +1 when braking. Both 6 and 7 are forced
//     to 100 on a position-control failure and 200 while pitch-forward is forced.

#ifndef POSHOLD_TASK_RATE_HZ
#define POSHOLD_TASK_RATE_HZ 100
#endif

#define ALTITUDE_P_SCALE       0.018f
#define ALTITUDE_I_SCALE       0.005f
#define ALTITUDE_D_SCALE       0.03f
#define ALTITUDE_A_SCALE       0.006f
#define ALTITUDE_F_SCALE       0.02f // vertical velocity target scale (on the target itself, not its delta)
#define ALTITUDE_VEL_CMD_MAX_DEFAULT_CM_S  1500.0f
#define ALTITUDE_I_LIMIT      150.0f

// Unified XY control: a single distance-based PIDAF law serves position hold,
// nav missions and GPS rescue. D is a velocity-proportional factor, A is
// acceleration-proportional, F the target-velocity driver. The former separate
// nav velocity loop is gone; its gains fold onto these (velocity_P -> position_D,
// velocity_I -> position_P, velocity_D -> position_A).
#define XY_DISTANCE_SCALE      0.004f   // distance P  / velocity I
#define XY_DISTANCE_I_SCALE    0.00017f  // distance I
#define XY_VELOCITY_SCALE      0.01f    // distance D  / velocity P (opposes measured velocity)
#define XY_ACCEL_SCALE         0.0015f   // distance A  / velocity D
#define XY_F_SCALE             0.0001f  // degrees per cm/s^2 of target-velocity rate of change, per gain unit
#define XY_DRAG_SCALE          0.0002f   // velocity-based drag correction, must stay below the D scale
#define BRAKING_EXIT_SPEED_MIN 1.0f // exit braking mode below this
#define BRAKING_TIMEOUT_S                      1.0f // braking gives up and captures the point after this long

#define SANITY_CHECK_DISTANCE 2000.0f //20m, increased when stopping from speeds above 10m/s
// The settled-hold flyaway fence is graded, not instant: an excursion must
// persist this long before it counts as a failure. Field logs show single-fix
// multipath excursions of 12-51 m while parked in a clean hover; instant
// tripping turned each one into a POSHOLD FAIL with a level-out step.
#define SANITY_VIOLATION_LATCH_S 1.0f
// One automatic re-anchor is allowed per healthy stretch; this much clean
// settled time earns it back. A genuine flyaway trips again immediately after
// its retry and stays failed.
#define SANITY_RETRY_REPLENISH_S 10.0f
#define ERROR_DISTANCE_LIMIT  2000.0f // TO DO: test set to a useful value, this is 20m
// Nav anchors position to positionNav's carrot, whose lead grows with speed; a
// tighter error bound stops that speed-proportional lead from driving P into a
// positive-feedback overspeed, leaving the velocity feedforward to set cruise.
#define NAV_ERROR_DISTANCE_LIMIT 500.0f // 5m
#define POSITION_I_LIMIT      2000.0f // TO DO: test and set to a useful value, this is 20m

#define AP_YAW_P_SCALE         0.01f
#define AP_YAW_D_SCALE         0.01f
#define AP_YAW_RAMP_TIME_S     1.0f

static pidCoefficient_t xyPid;
static float xyKDrag;

static float altitudeKp;
static float altitudeKi;
static float altitudeKd;
static float altitudeKa;
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
static vector2_t previousTargetVelocity; // for the target-velocity-delta feedforward
static vector2_t targetAcceleration;     // stick feedforward driver, earth frame
static vector2_t posHoldStartPosition;
static vector2_t distanceError;          // deviation from intended position (real or virtual)
static vector2_t distanceErrorIntegral;  // integral of position error
static vector2_t previousVelocity;       // for reversal of velocity detection

static pt3Filter_t posNoisyPidsLpf[EF_AXIS_COUNT]; // smooths F, the noisiest term

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

typedef enum {
    ANCHOR_OFF = 0,   // virtual distance error, integrated from the velocity error
    ANCHOR_HOLD,      // real distance error: targetPosition - currentPosition
} xyAnchorMode_e;

typedef enum {
    I_ACCUMULATE = 0, // integrate the distance error (settled hold, holds against wind)
    I_FREEZE,         // retain the current integral, do not accumulate (sticks active)
    I_ZERO,           // force the integral to zero (pure velocity tracking: nav / rescue)
} xyIntegralPolicy_e;

typedef struct autopilotState_s {
    float sanityCheckDistance;
    float sanityViolationS;     // time the settled hold has spent beyond the fence
    float violationFreeS;       // clean settled time since the last violation; replenishes the retry
    bool sanityRetryUsed;       // one automatic re-anchor per healthy stretch
    bool sticksActive;
    bool wasSticksActive;
    bool navActive;
    float maxAngle;
    float speedXY;              // horizontal ground speed this loop, cm/s
    float speedTrendCmS;        // ~0.5 s lowpass of speedXY: reference for "is the craft slowing?"
    bool speedSlowing;          // speed is meaningfully below its own trend
    bool isPosHoldBraking;      // decelerating toward a captured hold point
    float brakingTimeS;         // seconds spent in the current braking phase
    vector2_t brakingEntryDirection; // velocity direction when entering stopping phase
    float brakingExitSpeed;
    xyAnchorMode_e anchor;      // position-anchor selection for this loop
    xyIntegralPolicy_e iPolicy; // integral policy for this loop
    unsigned debugAxis;
} autopilotState_t;

float autopilotAngle[RP_AXIS_COUNT];

static autopilotState_t ap = {
    .sanityCheckDistance = SANITY_CHECK_DISTANCE,
    .sticksActive = false,
    .wasSticksActive = false,
};

static float posLpfDtS = 0.0f;  // interval the F filter gains are currently set for

static float posPidLpfGain(float dtS)
{
    const float cutoffHz = fmaxf(autopilotConfig()->positionCutoff * 0.1f, 0.1f); // default of 30 is 3Hz, range 1 to 5Hz
    return pt3FilterGain(cutoffHz, dtS);
}

static void initPidLpfs(void)
{
    posLpfDtS = HZ_TO_INTERVAL(POSHOLD_TASK_RATE_HZ);  // nominal until the loop measures itself
    const float k = posPidLpfGain(posLpfDtS);
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        pt3FilterInit(&posNoisyPidsLpf[i], k);
    }
}

// The F filter steps once per loop, so its gain follows the real task interval:
// TASK_POSHOLD is rescheduled to the flow sensor's rate, which need not be
// POSHOLD_TASK_RATE_HZ. Retuned in place, without resetting the filter states,
// whenever the measured interval moves.
static void updatePidLpfGains(float dtS)
{
    if (dtS <= 0.0f || fabsf(dtS - posLpfDtS) < 0.05f * posLpfDtS) {
        return;
    }
    posLpfDtS = dtS;
    const float k = posPidLpfGain(dtS);
    for (unsigned i = 0; i < EF_AXIS_COUNT; i++) {
        pt3FilterUpdateCutoff(&posNoisyPidsLpf[i], k);
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
    altitudeKa = cfg->altitudeA * ALTITUDE_A_SCALE;
    altitudeKf = cfg->altitudeF * ALTITUDE_F_SCALE;

    xyPid.Kp = cfg->positionP * XY_DISTANCE_SCALE;
    xyPid.Ki = cfg->positionI * XY_DISTANCE_I_SCALE;
    xyPid.Kd = cfg->positionD * XY_VELOCITY_SCALE;
    xyPid.Ka = cfg->positionA * XY_ACCEL_SCALE;
    xyPid.Kf = cfg->positionF * XY_F_SCALE;
    xyKDrag  = fminf(cfg->velocityDragCoeff * XY_DRAG_SCALE, 0.5f * xyPid.Kd); // keep drag a fraction of D so it can never reverse net damping

    ap.sticksActive = false;
    ap.wasSticksActive = false;
    ap.speedXY = 0.0f;
    ap.speedTrendCmS = 0.0f;
    ap.speedSlowing = false;
    ap.brakingTimeS = 0.0f;
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
    const float verticalAcceleration = getAltitudeAccelerationControl();
    const float verticalVelocity = getAltitudeDerivativeControl();
    const float altitudeErrorCm = targetAltitudeCm - currentAltitudeCm;
    const float itermRelax = (fabsf(altitudeErrorCm) < 200.0f) ? 1.0f : 0.1f; // don't accumulate too much iTerm with transient but large overshoots (>2m error )
    const float altitudeP = altitudeErrorCm * altitudeKp;
    altitudeI += altitudeErrorCm * altitudeKi * itermRelax * taskIntervalS;
    altitudeI = constrainf(altitudeI, -ALTITUDE_I_LIMIT, ALTITUDE_I_LIMIT);
    const float velMax = (velLimitCmS > 1.0f) ? velLimitCmS : ALTITUDE_VEL_CMD_MAX_DEFAULT_CM_S;
    const float targetVerticalVelocity = constrainf(targetAltitudeVelCmS, -velMax, velMax);
    float dBoost = 1.0f;
    const float boostThreshold = 500.0f; // 5m/s
    const float absVerticalVelocity = fabsf(verticalVelocity);
    if (absVerticalVelocity > boostThreshold) {
        const float ratio = absVerticalVelocity / boostThreshold;
        dBoost = (3.0f * ratio - 2.0f) / ratio; // 1 at 5m/s, 2 at 10m/s...
    }
    const float altitudeD = -verticalVelocity * altitudeKd * dBoost;
    const float altitudeA = -verticalAcceleration *  altitudeKa;
    const float altitudeF = targetVerticalVelocity * altitudeKf;

    const float hoverOffset = (float)autopilotGetEffectiveHoverThrottlePwm() - PWM_RANGE_MIN;


    float throttleOffset = altitudeP
                         + altitudeI
                         + altitudeD
                         + altitudeA
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
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 1, lrintf(targetAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 2, lrintf(currentAltitudeCm));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 3, lrintf(altitudeP));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 4, lrintf(altitudeI));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 5, lrintf(altitudeD));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 6, lrintf(altitudeA));
    DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, lrintf(altitudeF));
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

static void setBrakingMode(void)
{
    const vector2_t *velocity =
        (const vector2_t *)&positionEstimatorGetEstimate()->velocity.v;

    const float speedXY = vector2Norm(velocity);

    ap.isPosHoldBraking = true;
    ap.brakingTimeS = 0.0f;

    ap.brakingExitSpeed = MAX(
        BRAKING_EXIT_SPEED_MIN,
        speedXY * (float)autopilotConfig()->stopThreshold * 0.01f);

    if (speedXY > BRAKING_EXIT_SPEED_MIN) {
        ap.brakingEntryDirection.v[EF_NORTH] = velocity->v[EF_NORTH] / speedXY;
        ap.brakingEntryDirection.v[EF_EAST]  = velocity->v[EF_EAST]  / speedXY;
    } else {
        vector2Zero(&ap.brakingEntryDirection);
    }
}

void initPositionHold(void)
{
    updatePositionHoldTarget();
    resetDistanceError();
    setBrakingMode(); // arrest entry speed only when starting fast
    vector2Zero(&targetVelocity);
    vector2Zero(&targetAcceleration);
    vector2Zero(&previousTargetVelocity);
    // nb: we do not reset the distanceError integral, to hold its opposition to wind between quick stick inputs
}

// Re-anchor the hold at the craft's current position — what a pilot cycling
// the mode switch achieves, callable from recovery paths (sensor dropout
// recovery, the one-shot sanity retry). Enters through the normal braking
// capture so any speed the craft picked up meanwhile is arrested first, and
// the fence is re-sized to the current ground speed.

void positionControlReanchor(void)
{
    initPositionHold();
    ap.sanityCheckDistance = calculateSanityCheckDistance();
    ap.sanityViolationS = 0.0f;
    ap.violationFreeS = 0.0f;
}

static void initNavMode(void)
{
    initPidLpfs();
    resetDistanceError();
    resetDistanceErrorIntegral();
    vector2Zero(&previousTargetVelocity);
    vector2Zero(&targetAcceleration);
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
    ap.sanityViolationS = 0.0f;
    ap.violationFreeS = 0.0f;
    ap.sanityRetryUsed = false;
    initPositionHold(); // sets target location, resets distance error, enables braking mode
    previousVelocity = *(const vector2_t *)&positionEstimatorGetEstimate()->velocity.v; // for velocity reversal detection
    resetDistanceErrorIntegral();
}

void handlepositionControlFailure(void)
{
    resetDistanceError();
    resetDistanceErrorIntegral();
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 7, 100);
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 6, 100);
    DEBUG_SET(DEBUG_AUTOPILOT_STOP, 7, 100);

}

// A sanity-fence violation has outlived the persistence window: spend the
// one-shot retry if it is available (re-anchor and carry on), otherwise level
// out and fail the hold.
static bool sanityViolationExpired(void)
{
    if (!ap.sanityRetryUsed) {
        ap.sanityRetryUsed = true;
        positionControlReanchor();
        return true;
    }
    disableYawControl();
    autopilotAngle[AI_ROLL]  = 0.0f; // Level out
    autopilotAngle[AI_PITCH] = 0.0f;
    handlepositionControlFailure();
    return false; // Return failure, allow angle mode control, and show pos hold fail message in OSD
}

// Stick input becomes a target velocity (full stick -> maxVelocity), rotated
// from body to earth frame. Stick feedforward becomes the target-acceleration
// driver for the F term. The anchor-off virtual-distance path turns this
// target velocity into P/I, so there is no targetPosition integration here.
void sticksSetTargetVelocity(void)
{
    // full stick maps to maxVelocity; the rates can change between flights, so scale per loop
    const float velocityGainPitch = autopilotConfig()->maxVelocity / getMaxRcRate(PITCH);
    const float velocityGainRoll  = autopilotConfig()->maxVelocity / getMaxRcRate(ROLL);

    const float stickPitch = getSetpointRate(PITCH) * velocityGainPitch;
    const float stickRoll  = getSetpointRate(ROLL)  * velocityGainRoll;

    const float headingRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);
    const float cosYaw = cosf(headingRad);
    const float sinYaw = sinf(headingRad);

    targetVelocity.v[EF_NORTH] = (stickPitch * cosYaw) - (stickRoll * sinYaw);
    targetVelocity.v[EF_EAST]  = (stickPitch * sinYaw) + (stickRoll * cosYaw);

#ifdef USE_FEEDFORWARD
    const float ffPitch = getFeedforward(PITCH) * velocityGainPitch;
    const float ffRoll  = getFeedforward(ROLL)  * velocityGainRoll;
    targetAcceleration.v[EF_NORTH] = (ffPitch * cosYaw) - (ffRoll * sinYaw);
    targetAcceleration.v[EF_EAST]  = (ffPitch * sinYaw) + (ffRoll * cosYaw);
#endif
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
    updatePidLpfGains(dt);

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
    vector2_t pidP               = { { 0 } };
    vector2_t pidI               = { { 0 } };
    vector2_t pidD               = { { 0 } };
    vector2_t pidA               = { { 0 } };
    vector2_t pidF               = { { 0 } };
    // Update navigation status
    positionNavUpdate(dt, est);
    ap.navActive = positionNavHasActiveTarget() && !positionNavTargetReached();

    // Horizontal ground speed and its ~0.5 s trend, used by the braking stop
    // detector and the braking-phase sanity fence. "Slowing" is speed below its
    // own average (judged before the average absorbs the new sample): brake
    // physics (distance grows while speed falls) versus a flyaway (speed held).
    ap.speedXY = vector2Norm(&velocity);
    ap.speedSlowing = ap.speedXY < ap.speedTrendCmS - 20.0f;
    ap.speedTrendCmS += (dt / (0.5f + dt)) * (ap.speedXY - ap.speedTrendCmS);

    // Default control policy for the loop; each feeder overrides as needed.
    ap.anchor = ANCHOR_HOLD;
    ap.iPolicy = I_ACCUMULATE;

    if (ap.navActive) {
        isPositionHeld = false;
        if (!wasNavActive) {
            initNavMode();
        }
        const vector3_t tgtVel = positionNavGetTargetVelocityCmS();
        targetVelocity = *(const vector2_t *)&tgtVel.v;
        ap.isPosHoldBraking = false; // nav sequences its own speed
        ap.iPolicy = I_ZERO;         // position feedback carries the trim; no second integral
        const positionNavCommand_t *navCmd = positionNavGetActiveCommand();
        if (navCmd != NULL && navCmd->active) {
            // Anchor to the (moving) carrot: real position feedback keeps straight
            // and curved legs from drifting, with the commanded velocity as the
            // feedforward. The carrot's lead distance produces the cruise tilt via P.
            targetPosition.v[EF_EAST]  = navCmd->targetPosEfM.v[ENU_E] * 100.0f;
            targetPosition.v[EF_NORTH] = navCmd->targetPosEfM.v[ENU_N] * 100.0f;
            ap.anchor = ANCHOR_HOLD;
        } else {
            ap.anchor = ANCHOR_OFF;  // no active command target: track velocity only
        }
    } else {
        // Control mode should be position hold
        if (!isPositionHeld) {
            initPositionHold(); // set target position, activate braking mode if moving fast
            ap.sanityCheckDistance = calculateSanityCheckDistance();
            isPositionHeld = true;
        }
        if (ap.sticksActive) {
            if (!ap.wasSticksActive) {
                resetDistanceError();
                ap.sanityCheckDistance = calculateSanityCheckDistance();
                ap.isPosHoldBraking = false; // pilot is commanding, don't brake
            }
            sticksSetTargetVelocity();
            posHoldStartPosition = currentPosition; // pilot may fly far; keep the fence with the craft
            ap.anchor = ANCHOR_OFF;  // fly the commanded velocity via the virtual distance error
            ap.iPolicy = I_FREEZE;   // retain the integral, do not wind it up while manoeuvring
        } else {
            // No stick input
#ifdef USE_GPS_RESCUE
            if (!FLIGHT_MODE(GPS_RESCUE_MODE)){
#endif
                // If in GPS Rescue, we have both velocity and acceleration
                // otherwise hold position at zero velocity and acceleration
                vector2Zero(&targetVelocity);
                vector2Zero(&targetAcceleration);
#ifdef USE_GPS_RESCUE
            }
#endif
            if (ap.wasSticksActive) {
                // Sticks just released: capture the current point and decide
                // whether to brake, based on the speed being carried.
                updatePositionHoldTarget();
                ap.sanityCheckDistance = calculateSanityCheckDistance();
                setBrakingMode();
            }
            if (ap.isPosHoldBraking) {
                // Braking: hold anchor with the target dragged to the craft, the
                // integral frozen, D boosted (in the loop). End on stop, a 1 s
                // timeout, or a velocity-vector reversal (overshoot past capture).
                ap.brakingTimeS = MIN(ap.brakingTimeS + dt, BRAKING_TIMEOUT_S);
                ap.anchor = ANCHOR_HOLD;
                ap.iPolicy = I_FREEZE;
                targetPosition = currentPosition;
                const float velocityAlongEntryDirection =
                    (velocity.v[EF_NORTH] * ap.brakingEntryDirection.v[EF_NORTH]) +
                    (velocity.v[EF_EAST]  * ap.brakingEntryDirection.v[EF_EAST]);
                const bool reversed = velocityAlongEntryDirection < 0.0f;
                const bool stopped = ap.speedXY < ap.brakingExitSpeed;
                const bool timedOut = ap.brakingTimeS >= BRAKING_TIMEOUT_S;
                if (stopped || timedOut || reversed) {
                    updatePositionHoldTarget(); // capture the stopped point as the hold target
                    ap.isPosHoldBraking = false;
                    ap.brakingTimeS = 0.0f;
                } else {
                    // The fence watches the brake too. Braking suppresses the
                    // settled check below, and a genuine flyaway (a bad-mag
                    // toilet bowl accelerates, so it never meets the stop or
                    // stall conditions) would otherwise ride the moving target
                    // indefinitely — including straight after the one-shot
                    // retry, which re-enters through this braking capture.
                    // While the craft is actually slowing, growing distance is
                    // brake physics and the fence rides just ahead of it (so a
                    // fast entry whose stopping distance beats 2 s of entry
                    // speed cannot false-trip); beyond the fence and NOT
                    // slowing runs the same violation clock as the settled
                    // hold.
                    vector2_t brakeDeltaV;
                    vector2Sub(&brakeDeltaV, &posHoldStartPosition, &currentPosition);
                    const float brakeDistance = vector2Norm(&brakeDeltaV);
                    if (brakeDistance > ap.sanityCheckDistance) {
                        if (ap.speedSlowing) {
                            ap.sanityCheckDistance = brakeDistance + 2.0f * ap.speedXY;
                            ap.sanityViolationS = 0.0f;
                        } else {
                            ap.violationFreeS = 0.0f;
                            ap.sanityViolationS += dt;
                            if (ap.sanityViolationS > SANITY_VIOLATION_LATCH_S) {
                                return sanityViolationExpired();
                            }
                        }
                    } else {
                        ap.sanityViolationS = 0.0f;
                    }
                }
            } else {
                // Settled hold: real position lock, integrate against wind.
                ap.anchor = ANCHOR_HOLD;
                ap.iPolicy = I_ACCUMULATE;
                // Guard against a position-estimate flyaway.
                // Graded, not instant: a single bad fix (multipath excursions
                // of 12-51 m appear in field logs during a clean hover) must
                // not fail the hold — fed to the PIDs it would also slam P
                // into the angle clamp, so the previous output is held while
                // a brief excursion passes. Only a persistent one fails, and
                // the first sustained trip earns one automatic re-anchor at
                // the current spot (what a pilot cycling the switch does):
                // an isolated mid-flight glitch self-heals, while a genuine
                // flyaway trips again immediately and stays failed.
                vector2_t deltaPosV;
                vector2Sub(&deltaPosV, &posHoldStartPosition, &currentPosition);
                if (vector2Norm(&deltaPosV) > ap.sanityCheckDistance) {
                    ap.violationFreeS = 0.0f;
                    ap.sanityViolationS += dt;
                    if (ap.sanityViolationS > SANITY_VIOLATION_LATCH_S) {
                        return sanityViolationExpired();
                    }
                    return true; // brief excursion: hold the previous command
                } else {
                    ap.sanityViolationS = 0.0f;
                    ap.violationFreeS += dt;
                    if (ap.violationFreeS > SANITY_RETRY_REPLENISH_S) {
                        ap.sanityRetryUsed = false;
                    }
                }
            }
        }
    }

    updateYawControl(dt, est);

    wasPositionHeld = isPositionHeld;
    wasNavActive = ap.navActive;
    ap.wasSticksActive = ap.sticksActive; // Main frame-to-frame history update

    const bool anchorOff = (ap.anchor == ANCHOR_OFF);

    // One unified distance-based PIDAF law. The mode differences are already
    // encoded in ap.anchor / ap.iPolicy / ap.isPosHoldBraking (set above); the
    // maths below is identical for position hold, nav and rescue.

    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
        velocityError.v[axis] = targetVelocity.v[axis] - velocity.v[axis];
        const float acceleration = -est->acceleration.v[axis]; // from Kalman acceleration estimate from accelerometer and derivative of other velocity sources)
        previousVelocity.v[axis] = velocity.v[axis];

        // Distance error: real (position anchor) or virtual (integral of the
        // velocity error). The virtual integral carries the steady cruise tilt,
        // so it accumulates continuously, gated only by saturation anti-windup
        // (a maxAngle-clamped output stops it growing but may still unwind).
        if (anchorOff) {
            const bool windupOk = !wasAngleSaturated || (velocityError.v[axis] * distanceError.v[axis]) < 0.0f;
            if (windupOk) {
                distanceError.v[axis] += velocityError.v[axis] * dt;
            }
        } else {
            distanceError.v[axis] = targetPosition.v[axis] - currentPosition.v[axis];
        }
        const float errLimit = ap.navActive ? NAV_ERROR_DISTANCE_LIMIT : ERROR_DISTANCE_LIMIT;
        distanceError.v[axis] = constrainf(distanceError.v[axis], -errLimit, errLimit);

        // Integral of the distance error, governed by the integral policy.
        switch (ap.iPolicy) {
        case I_ACCUMULATE:
            distanceErrorIntegral.v[axis] += distanceError.v[axis] * dt;
            break;
        case I_ZERO:
            distanceErrorIntegral.v[axis] = 0.0f;
            break;
        case I_FREEZE:
        default:
            break;
        }
        distanceErrorIntegral.v[axis] = constrainf(distanceErrorIntegral.v[axis], -POSITION_I_LIMIT, POSITION_I_LIMIT);

        // Feedforward driver: the rate of change of the target velocity, or the
        // stick setpoint feedforward when the pilot is commanding.
        // Per-loop change divided by the real loop interval, so the same physical
        // acceleration gives the same feedforward at any task rate.
        float targetVelDelta = (targetVelocity.v[axis] - previousTargetVelocity.v[axis]) / dt;
        previousTargetVelocity.v[axis] = targetVelocity.v[axis];
#ifdef USE_FEEDFORWARD
        if (ap.sticksActive) {
            targetVelDelta = targetAcceleration.v[axis];
        }
#endif

        const float brakeBoost = ap.isPosHoldBraking ? (1.0f + fabsf(velocity.v[axis]) * 0.0005f) : 1.0f; // ~2x at 20 m/s

        pidP.v[axis] = distanceError.v[axis] * xyPid.Kp;
        pidI.v[axis] = distanceErrorIntegral.v[axis] * xyPid.Ki;
        pidD.v[axis] = -velocity.v[axis] * xyPid.Kd * brakeBoost + velocity.v[axis] * xyKDrag; // damping, minus drag at speed
        pidA.v[axis] = acceleration * xyPid.Ka;
        // F: Kd on the steady target velocity balances D so the pair cancels at
        // the target speed (reconstructing D-from-error); Kf on the delta is the
        // tunable acceleration feedforward.
        pidF.v[axis] = targetVelocity.v[axis] * xyPid.Kd + targetVelDelta * xyPid.Kf;
    } // End for loop

    // Buildup clamp: only in the anchor-off fallback, where the velocity-error
    // drive (D + F = Kd*velocityError plus the accel feedforward) itself carries
    // the cruise tilt and would otherwise slam the pitch while accelerating. When
    // anchored to a position (nav carrot or hold), P carries the tilt and D + F
    // must stay free to track and brake velocity, so the clamp is skipped.
    bool buildupClamped = false;
    if (ap.navActive && ap.anchor == ANCHOR_OFF) {
        const float buildupMaxDeg = autopilotConfig()->velocityBuildupMaxPitch;
        vector2_t drive = { { pidD.v[EF_EAST] + pidF.v[EF_EAST], pidD.v[EF_NORTH] + pidF.v[EF_NORTH] } };
        const float driveMag = vector2Norm(&drive);
        if (driveMag > buildupMaxDeg && driveMag > 0.001f) {
            buildupClamped = true;
            const float scale = buildupMaxDeg / driveMag;
            vector2Scale(&pidD, &pidD, scale);
            vector2Scale(&pidF, &pidF, scale);
        }
    }

    // Combine: only F is smoothed, being the noisiest term; P, I, D and A all
    // ride outside the filter. NOTE: D is on raw measured velocity and A on the
    // raw Kalman acceleration — filter placement is an open tuning item.
    for (unsigned axis = 0; axis < EF_AXIS_COUNT; axis++) {
        const float smoothedF = pt3FilterApply(&posNoisyPidsLpf[axis], pidF.v[axis]);
        pidSumVectorEF.v[axis] = pidP.v[axis] + pidI.v[axis] + pidD.v[axis] + pidA.v[axis] + smoothedF;
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
    if (anchorOff)          statusValue += 20;
    if (abortNavRequested)  statusValue += 100;
    if (isPositionHeld)     statusValue += 3; // plus 1, ie 4,  if stopping
    if (ap.sticksActive)    statusValue += 5;
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 0, lrintf(velocity.v[ap.debugAxis]));
    DEBUG_SET(DEBUG_AUTOPILOT_PID, 1, lrintf(distanceError.v[ap.debugAxis]));
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
    DEBUG_SET(DEBUG_POSITION_NAV, 7, (anchorOff ? 10 : 0) + (buildupClamped ? 1 : 0));

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

/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"

uint32_t targetPidLooptime;
static bool pidStabilisationEnabled;

float axisPIDf[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

static float previousGyroIf[3];

static float dT;

void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    dT = (float)targetPidLooptime * 0.000001f;
}

void pidResetErrorGyroState(void)
{
    for (int axis = 0; axis < 3; axis++) {
        previousGyroIf[axis] = 0.0f;
    }
}

static float itermAccelerator = 1.0f;

void pidSetItermAccelerator(float newItermAccelerator) {
    itermAccelerator = newItermAccelerator;
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static filterApplyFnPtr dtermNotchFilterApplyFn;
static void *dtermFilterNotch[2];
static filterApplyFnPtr dtermLpfApplyFn;
static void *dtermFilterLpf[2];
static filterApplyFnPtr ptermYawFilterApplyFn;
static void *ptermYawFilter;

void pidInitFilters(const pidProfile_t *pidProfile)
{
    static biquadFilter_t biquadFilterNotch[2];
    static pt1Filter_t pt1Filter[2];
    static biquadFilter_t biquadFilter[2];
    static firFilterDenoise_t denoisingFilter[2];
    static pt1Filter_t pt1FilterYaw;

    BUILD_BUG_ON(FD_YAW != 2); // only setting up Dterm filters on roll and pitch axes, so ensure yaw axis is 2

    if (pidProfile->dterm_notch_hz == 0) {
        dtermNotchFilterApplyFn = nullFilterApply;
    } else {
        dtermNotchFilterApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(pidProfile->dterm_notch_hz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
            dtermFilterNotch[axis] = &biquadFilterNotch[axis];
            biquadFilterInit(dtermFilterNotch[axis], pidProfile->dterm_notch_hz, targetPidLooptime, notchQ, FILTER_NOTCH);
        }
    }

    if (pidProfile->dterm_lpf_hz == 0) {
        dtermLpfApplyFn = nullFilterApply;
    } else {
        switch (pidProfile->dterm_filter_type) {
        default:
            dtermLpfApplyFn = nullFilterApply;
            break;
        case FILTER_PT1:
            dtermLpfApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
                dtermFilterLpf[axis] = &pt1Filter[axis];
                pt1FilterInit(dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, dT);
            }
            break;
        case FILTER_BIQUAD:
            dtermLpfApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
                dtermFilterLpf[axis] = &biquadFilter[axis];
                biquadFilterInitLPF(dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
            }
            break;
        case FILTER_FIR:
            dtermLpfApplyFn = (filterApplyFnPtr)firFilterDenoiseUpdate;
            for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
                dtermFilterLpf[axis] = &denoisingFilter[axis];
                firFilterDenoiseInit(dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
            }
            break;
        }
    }

    if (pidProfile->yaw_lpf_hz == 0) {
        ptermYawFilterApplyFn = nullFilterApply;
    } else {
        ptermYawFilterApplyFn = (filterApplyFnPtr)pt1FilterApply;
        ptermYawFilter = &pt1FilterYaw;
        pt1FilterInit(ptermYawFilter, pidProfile->yaw_lpf_hz, dT);
    }
}

static float Kp[3], Ki[3], Kd[3], c[3], levelGain, horizonGain, horizonTransition, maxVelocity[3], relaxFactor[3];

void pidInitConfig(const pidProfile_t *pidProfile) {
    for(int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        Kp[axis] = PTERM_SCALE * pidProfile->P8[axis];
        Ki[axis] = ITERM_SCALE * pidProfile->I8[axis];
        Kd[axis] = DTERM_SCALE * pidProfile->D8[axis];
        c[axis] = pidProfile->dtermSetpointWeight / 100.0f;
        relaxFactor[axis] = 1.0f - (pidProfile->setpointRelaxRatio / 100.0f);
    }
    levelGain = pidProfile->P8[PIDLEVEL] / 10.0f;
    horizonGain = pidProfile->I8[PIDLEVEL] / 10.0f;
    horizonTransition = 100.0f / pidProfile->D8[PIDLEVEL];
    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 1000 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 1000 * dT;
}

static float calcHorizonLevelStrength(void) {
    float horizonLevelStrength = 0.0f;
    if (horizonTransition > 0.0f) {
        const float mostDeflectedPos = MAX(getRcDeflectionAbs(FD_ROLL), getRcDeflectionAbs(FD_PITCH));
        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = constrainf(1 - mostDeflectedPos * horizonTransition, 0, 1);
    }
    return horizonLevelStrength;
}

static float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) {
    // calculate error angle and limit the angle to the max inclination
    float errorAngle = pidProfile->levelSensitivity * getRcDeflection(axis);
#ifdef GPS
    errorAngle += GPS_angle[axis];
#endif
    errorAngle = constrainf(errorAngle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    errorAngle = (errorAngle - ((attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f));
    if(FLIGHT_MODE(ANGLE_MODE)) {
        // ANGLE mode - control is angle based, so control loop is needed
        currentPidSetpoint = errorAngle * levelGain;
    } else {
        // HORIZON mode - direct sticks control is applied to rate PID
        // mix up angle error to desired AngleRate to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = currentPidSetpoint + (errorAngle * horizonGain * horizonLevelStrength);
    }
    return currentPidSetpoint;
}

static float accelerationLimit(int axis, float currentPidSetpoint) {
    static float previousSetpoint[3];
    const float currentVelocity = currentPidSetpoint- previousSetpoint[axis];

    if(ABS(currentVelocity) > maxVelocity[axis])
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void pidController(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float tpaFactor)
{
    static float previousRateError[2];
    static float previousSetpoint[3];

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        float currentPidSetpoint = getSetpointRate(axis);

        if(maxVelocity[axis])
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);

        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && axis != YAW) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }

        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        //  ---------- 2-DOF PID controller with optional filter on derivative term. b = 1 and only c can be tuned (amount derivative on measurement or error).  ----------

        // -----calculate error rate
        const float errorRate = currentPidSetpoint - gyroRate;       // r - y

        // -----calculate P component and add Dynamic Part based on stick input
        float PTerm = Kp[axis] * errorRate * tpaFactor;
        if (axis == FD_YAW) {
            PTerm = ptermYawFilterApplyFn(ptermYawFilter, PTerm);
        }

        // -----calculate I component
        // Reduce strong Iterm accumulation during higher stick inputs
        const float accumulationThreshold = (axis == FD_YAW) ? pidProfile->yawItermIgnoreRate : pidProfile->rollPitchItermIgnoreRate;
        const float setpointRateScaler = constrainf(1.0f - (ABS(currentPidSetpoint) / accumulationThreshold), 0.0f, 1.0f);

        float ITerm = previousGyroIf[axis];
        ITerm += Ki[axis] * errorRate * dT * setpointRateScaler * itermAccelerator;
        // limit maximum integrator value to prevent WindUp
        ITerm = constrainf(ITerm, -250.0f, 250.0f);
        previousGyroIf[axis] = ITerm;

        // -----calculate D component (Yaw D not yet supported)
        float DTerm = 0.0;
        if (axis != FD_YAW) {
            float dynC = c[axis];
            if (pidProfile->setpointRelaxRatio < 100) {
                const float rcDeflection = getRcDeflectionAbs(axis);
                dynC = c[axis];
                if (currentPidSetpoint > 0) {
                    if ((currentPidSetpoint - previousSetpoint[axis]) < previousSetpoint[axis])
                        dynC = dynC * sq(rcDeflection) * relaxFactor[axis] + dynC * (1-relaxFactor[axis]);
                } else if (currentPidSetpoint < 0) {
                    if ((currentPidSetpoint - previousSetpoint[axis]) > previousSetpoint[axis])
                        dynC = dynC * sq(rcDeflection) * relaxFactor[axis] + dynC * (1-relaxFactor[axis]);
                }
            }
            const float rD = dynC * currentPidSetpoint - gyroRate;    // cr - y
            // Divide rate change by dT to get differential (ie dr/dt)
            const float delta = (rD - previousRateError[axis]) / dT;
            previousRateError[axis] = rD;

            DTerm = Kd[axis] * delta * tpaFactor;
            DEBUG_SET(DEBUG_DTERM_FILTER, axis, DTerm);

            // apply filters
            DTerm = dtermNotchFilterApplyFn(dtermFilterNotch[axis], DTerm);
            DTerm = dtermLpfApplyFn(dtermFilterLpf[axis], DTerm);

        }
        previousSetpoint[axis] = currentPidSetpoint;

        // -----calculate total PID output
        axisPIDf[axis] = PTerm + ITerm + DTerm;
        // Disable PID control at zero throttle
        if (!pidStabilisationEnabled) axisPIDf[axis] = 0;

#ifdef BLACKBOX
        axisPID_P[axis] = PTerm;
        axisPID_I[axis] = ITerm;
        axisPID_D[axis] = DTerm;
#endif
    }
}

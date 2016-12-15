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

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"

extern float rcInput[3];
extern float setpointRate[3];

uint32_t targetPidLooptime;
static bool pidStabilisationEnabled;

float axisPIDf[3];

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t PIDweight[3];

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

static float Kp[3], Ki[3], Kd[3], c[3];
static float rollPitchMaxVelocity, yawMaxVelocity, relaxFactor[3];

void pidInitConfig(const pidProfile_t *pidProfile) {
    for(int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        Kp[axis] = PTERM_SCALE * pidProfile->P8[axis];
        Ki[axis] = ITERM_SCALE * pidProfile->I8[axis];
        Kd[axis] = DTERM_SCALE * pidProfile->D8[axis];
        c[axis] = pidProfile->dtermSetpointWeight / 100.0f;
        relaxFactor[axis] = 1.0f - (pidProfile->setpointRelaxRatio / 100.0f);
    }
    yawMaxVelocity = pidProfile->yawRateAccelLimit * 1000 * dT;
    rollPitchMaxVelocity = pidProfile->rateAccelLimit * 1000 * dT;
}

// calculates strength of horizon leveling; 0 = none, 100 = most leveling
static int calcHorizonLevelStrength(uint16_t rxConfigMidrc, int horizonTiltEffect,
        uint8_t horizonTiltMode, int horizonSensitivity)
{
    // get raw stick positions (-500 to 500):
    const int32_t stickPosAil = getRcStickDeflection(FD_ROLL, rxConfigMidrc);
    const int32_t stickPosEle = getRcStickDeflection(FD_PITCH, rxConfigMidrc);

    // 0 at center stick, 500 at max stick deflection:
    const int32_t mostDeflectedPos = MAX(ABS(stickPosAil), ABS(stickPosEle));

    // start with 100 at center stick, 0 at max stick deflection:
    int horizonLevelStrength = (500 - mostDeflectedPos) / 5;

    // 0 at level, 900 at vertical, 1800 at inverted (degrees * 10)
    const int currentInclination = MAX(ABS(attitude.values.roll),
                                                ABS(attitude.values.pitch));

    // horizonTiltMode:  SAFE = leveling always active when sticks centered,
    // EXPERT = leveling can be totally off when inverted
    if (horizonTiltMode == HORIZON_TILT_MODE_EXPERT) {
        if (horizonTiltEffect < 175) {
            // horizonTiltEffect 0 to 125 => 2700 to 900
            //  (represents where leveling goes to zero):
            const int cutoffDeciDegrees = (175-horizonTiltEffect) * 18;
            // inclinationLevelRatio (0 to 100) is smaller (less leveling)
            //  for larger inclinations; 0 at cutoffDeciDegrees value:
            const int inclinationLevelRatio = constrain(
                              (((cutoffDeciDegrees-currentInclination)*10) /
                                           (cutoffDeciDegrees/10)), 0, 100);
            // apply configured horizon sensitivity:
            if (horizonSensitivity <= 0) {       // zero means no leveling
                horizonLevelStrength = 0;
            } else {
                // when stick is near center (horizonLevelStrength ~= 100)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0)
                //  H_sensitivity value has more effect:
                horizonLevelStrength = (horizonLevelStrength - 100) *
                        100 / horizonSensitivity + 100;
            }
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength = horizonLevelStrength * inclinationLevelRatio / 100;
        }
        else
          horizonLevelStrength = 0;
    } else {  // horizon_tilt_mode = SAFE (leveling always active when sticks centered)
        int sensitFact;
        if (horizonTiltEffect > 0) {
            // ratio of 100 to 0 (larger means more leveling):
            const int factorRatio = 100 - horizonTiltEffect;
            // inclinationLevelRatio (0 to 100) is smaller (less leveling)
            //  for larger inclinations, goes to 100 at inclination level:
            int inclinationLevelRatio =
                  ((1800-currentInclination)/18 * (100-factorRatio)) / 100 + factorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = horizonSensitivity * inclinationLevelRatio / 100;
        }
        else   // horizonTiltEffect=0 for "old" functionality
            sensitFact = horizonSensitivity;

        if (sensitFact <= 0) {           // zero means no leveling
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 100)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0)
            //  sensitFact value has more effect:
            horizonLevelStrength =  (horizonLevelStrength - 100) * 100 / sensitFact + 100;
        }
    }
    return constrain(horizonLevelStrength, 0, 100);
}

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void pidController(const pidProfile_t *pidProfile, uint16_t max_angle_inclination,
         const rollAndPitchTrims_t *angleTrim, uint16_t midrc)
{
    static float previousRateError[2];
    static float previousSetpoint[3];

    float horizonLevelStrength = 1;
    if (FLIGHT_MODE(HORIZON_MODE)) {
        // (convert 0-100 range to 0.0-1.0 range)
        horizonLevelStrength = (float)calcHorizonLevelStrength(midrc, pidProfile->horizon_tilt_effect,
                pidProfile->horizon_tilt_mode, pidProfile->D8[PIDLEVEL]) / 100.0f;
    }

    // Yet Highly experimental and under test and development
    // Throttle coupled to Igain like inverted TPA // 50hz calculation (should cover all rx protocols)
    static float kiThrottleGain = 1.0f;
    if (pidProfile->itermThrottleGain) {
        const uint16_t maxLoopCount = 20000 / targetPidLooptime;
        const float throttleItermGain = (float)pidProfile->itermThrottleGain * 0.001f;
        static int16_t previousThrottle;
        static uint16_t loopIncrement;

        if (loopIncrement >= maxLoopCount) {
            kiThrottleGain = 1.0f + constrainf((float)(ABS(rcCommand[THROTTLE] - previousThrottle)) * throttleItermGain, 0.0f, 5.0f); // Limit to factor 5
            previousThrottle = rcCommand[THROTTLE];
            loopIncrement = 0;
        } else {
            loopIncrement++;
        }
    }

    // ----------PID controller----------
    const float tpaFactor = PIDweight[0] / 100.0f; // tpa is now float
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        // Limit abrupt yaw inputs / stops
        const float maxVelocity = (axis == FD_YAW) ? yawMaxVelocity : rollPitchMaxVelocity;
        if (maxVelocity) {
            const float currentVelocity = setpointRate[axis] - previousSetpoint[axis];
            if (ABS(currentVelocity) > maxVelocity) {
                setpointRate[axis] = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity : previousSetpoint[axis] - maxVelocity;
            }
        }

        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && axis != YAW) {
            // calculate error angle and limit the angle to the max inclination
#ifdef GPS
                const float errorAngle = (constrainf(pidProfile->levelSensitivity * rcCommand[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f; // 16 bits is ok here
#else
                const float errorAngle = (constrainf(pidProfile->levelSensitivity * rcCommand[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f; // 16 bits is ok here
#endif
            if (FLIGHT_MODE(ANGLE_MODE)) {
                // ANGLE mode - control is angle based, so control loop is needed
                setpointRate[axis] = errorAngle * pidProfile->P8[PIDLEVEL] / 10.0f;
            } else {
                // HORIZON mode - direct sticks control is applied to rate PID
                // mix up angle error to desired AngleRate to add a little auto-level feel
                setpointRate[axis] = setpointRate[axis] + (errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 10.0f);
            }
        }

        const float PVRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        //  ---------- 2-DOF PID controller with optional filter on derivative term. b = 1 and only c can be tuned (amount derivative on measurement or error).  ----------

        // -----calculate error rate
        const float errorRate = setpointRate[axis] - PVRate;       // r - y

        // -----calculate P component and add Dynamic Part based on stick input
        float PTerm = Kp[axis] * errorRate * tpaFactor;

        // -----calculate I component
        // Reduce strong Iterm accumulation during higher stick inputs
        const float accumulationThreshold = (axis == FD_YAW) ? pidProfile->yawItermIgnoreRate : pidProfile->rollPitchItermIgnoreRate;
        const float setpointRateScaler = constrainf(1.0f - (ABS(setpointRate[axis]) / accumulationThreshold), 0.0f, 1.0f);
        const float itermScaler = setpointRateScaler * kiThrottleGain;

        float ITerm = previousGyroIf[axis];
        ITerm += Ki[axis] * errorRate * dT * itermScaler;;
        // limit maximum integrator value to prevent WindUp
        ITerm = constrainf(ITerm, -250.0f, 250.0f);
        previousGyroIf[axis] = ITerm;

        // -----calculate D component (Yaw D not yet supported)
        float DTerm = 0.0;
        if (axis != FD_YAW) {
            float dynC = c[axis];
            if (pidProfile->setpointRelaxRatio < 100) {
                dynC = c[axis];
                if (setpointRate[axis] > 0) {
                    if ((setpointRate[axis] - previousSetpoint[axis]) < previousSetpoint[axis])
                        dynC = dynC * powerf(rcInput[axis], 2) * relaxFactor[axis] + dynC * (1-relaxFactor[axis]);
                } else if (setpointRate[axis] < 0) {
                    if ((setpointRate[axis] - previousSetpoint[axis]) > previousSetpoint[axis])
                        dynC = dynC * powerf(rcInput[axis], 2) * relaxFactor[axis] + dynC * (1-relaxFactor[axis]);
                }
            }
            const float rD = dynC * setpointRate[axis] - PVRate;    // cr - y
            // Divide rate change by dT to get differential (ie dr/dt)
            const float delta = (rD - previousRateError[axis]) / dT;
            previousRateError[axis] = rD;

            DTerm = Kd[axis] * delta * tpaFactor;
            DEBUG_SET(DEBUG_DTERM_FILTER, axis, DTerm);

            // apply filters
            DTerm = dtermNotchFilterApplyFn(dtermFilterNotch[axis], DTerm);
            DTerm = dtermLpfApplyFn(dtermFilterLpf[axis], DTerm);

        } else {
            PTerm = ptermYawFilterApplyFn(ptermYawFilter, PTerm);
        }
        previousSetpoint[axis] = setpointRate[axis];

        // -----calculate total PID output
        axisPIDf[axis] = PTerm + ITerm + DTerm;
        // Disable PID control at zero throttle
        if (!pidStabilisationEnabled) axisPIDf[axis] = 0;

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif

#ifdef BLACKBOX
        axisPID_P[axis] = PTerm;
        axisPID_I[axis] = ITerm;
        axisPID_D[axis] = DTerm;
#endif
    }
}

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

static float errorGyroIf[3];

void setTargetPidLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
}

void pidResetErrorGyroState(void)
{
    for (int axis = 0; axis < 3; axis++) {
        errorGyroIf[axis] = 0.0f;
    }
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

pt1Filter_t deltaFilter[3];
pt1Filter_t yawFilter;
biquadFilter_t dtermFilterLpf[3];
biquadFilter_t dtermFilterNotch[3];
bool dtermNotchInitialised;
bool dtermBiquadLpfInitialised;
firFilterDenoise_t dtermDenoisingState[3];

static void pidInitFilters(const pidProfile_t *pidProfile)
{
    static uint8_t lowpassFilterType;

    if (pidProfile->dterm_notch_hz && !dtermNotchInitialised) {
        float notchQ = filterGetNotchQ(pidProfile->dterm_notch_hz, pidProfile->dterm_notch_cutoff);
        for (int axis = 0; axis < 3; axis++) biquadFilterInit(&dtermFilterNotch[axis], pidProfile->dterm_notch_hz, targetPidLooptime, notchQ, FILTER_NOTCH);
        dtermNotchInitialised = true;
    }

    if ((pidProfile->dterm_filter_type != lowpassFilterType) && pidProfile->dterm_lpf_hz) {
        if (pidProfile->dterm_filter_type == FILTER_BIQUAD) {
            for (int axis = 0; axis < 3; axis++) biquadFilterInitLPF(&dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
        }

        if (pidProfile->dterm_filter_type == FILTER_FIR) {
            for (int axis = 0; axis < 3; axis++) firFilterDenoiseInit(&dtermDenoisingState[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
        }
        lowpassFilterType = pidProfile->dterm_filter_type;
    }
}

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void pidController(const pidProfile_t *pidProfile, uint16_t max_angle_inclination,
         const rollAndPitchTrims_t *angleTrim, uint16_t midrc)
{
    static float lastRateError[2];
    static float Kp[3], Ki[3], Kd[3], c[3];
    static float rollPitchMaxVelocity, yawMaxVelocity;
    static float previousSetpoint[3], relaxFactor[3];
    static float dT;

    if (!dT) {
        dT = (float)targetPidLooptime * 0.000001f;
    }

    pidInitFilters(pidProfile);

    float horizonLevelStrength = 1;
    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the raw stick positions
        const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL, midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH, midrc));
        const int32_t mostDeflectedPos = MAX(stickPosAil, stickPosEle);
        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
        if(pidProfile->D8[PIDLEVEL] == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / pidProfile->D8[PIDLEVEL])) + 1, 0, 1);
        }
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
    for (int axis = 0; axis < 3; axis++) {

        static uint8_t configP[3], configI[3], configD[3];

        // Prevent unnecessary computing and check for changed PIDs. No need for individual checks. Only pids is fine for now
        // Prepare all parameters for PID controller
        if ((pidProfile->P8[axis] != configP[axis]) || (pidProfile->I8[axis] != configI[axis]) || (pidProfile->D8[axis] != configD[axis])) {
            Kp[axis] = PTERM_SCALE * pidProfile->P8[axis];
            Ki[axis] = ITERM_SCALE * pidProfile->I8[axis];
            Kd[axis] = DTERM_SCALE * pidProfile->D8[axis];
            c[axis] = pidProfile->dtermSetpointWeight / 100.0f;
            relaxFactor[axis] = 1.0f - (pidProfile->setpointRelaxRatio / 100.0f);
            yawMaxVelocity = pidProfile->yawRateAccelLimit * 1000 * dT;
            rollPitchMaxVelocity = pidProfile->rateAccelLimit * 1000 * dT;

            configP[axis] = pidProfile->P8[axis];
            configI[axis] = pidProfile->I8[axis];
            configD[axis] = pidProfile->D8[axis];
        }

        // Limit abrupt yaw inputs / stops
        float maxVelocity = (axis == YAW) ? yawMaxVelocity : rollPitchMaxVelocity;
        if (maxVelocity) {
            float currentVelocity = setpointRate[axis] - previousSetpoint[axis];
            if (ABS(currentVelocity) > maxVelocity) {
                setpointRate[axis] = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity : previousSetpoint[axis] - maxVelocity;
            }
            previousSetpoint[axis] = setpointRate[axis];
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

        const float PVRate = gyroADCf[axis] / 16.4f; // Process variable from gyro output in deg/sec

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        //  ---------- 2-DOF PID controller with optional filter on derivative term. b = 1 and only c can be tuned (amount derivative on measurement or error).  ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // ----- calculate error / angle rates  ----------
        const float errorRate = setpointRate[axis] - PVRate;       // r - y

        // -----calculate P component and add Dynamic Part based on stick input
        float PTerm = Kp[axis] * errorRate * tpaFactor;

        // -----calculate I component.
        // Reduce strong Iterm accumulation during higher stick inputs
        float accumulationThreshold = (axis == YAW) ? pidProfile->yawItermIgnoreRate : pidProfile->rollPitchItermIgnoreRate;
        float setpointRateScaler = constrainf(1.0f - (ABS(setpointRate[axis]) / accumulationThreshold), 0.0f, 1.0f);

        // Handle All windup Scenarios
        // limit maximum integrator value to prevent WindUp
        float itermScaler = setpointRateScaler * kiThrottleGain;

        errorGyroIf[axis] = constrainf(errorGyroIf[axis] + Ki[axis] * errorRate * dT * itermScaler, -250.0f, 250.0f);

        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        const float ITerm = errorGyroIf[axis];

        //-----calculate D-term (Yaw D not yet supported)
        float DTerm;
        if (axis != YAW) {
            static float previousSetpoint[3];
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
            previousSetpoint[axis] = setpointRate[axis];
            const float rD = dynC * setpointRate[axis] - PVRate;    // cr - y
            float delta = rD - lastRateError[axis];
            lastRateError[axis] = rD;

            // Divide delta by targetLooptime to get differential (ie dr/dt)
            delta *= (1.0f / dT);

            if (debugMode == DEBUG_DTERM_FILTER) debug[axis] = Kd[axis] * delta * tpaFactor;

            // Filter delta
            if (dtermNotchInitialised) delta = biquadFilterApply(&dtermFilterNotch[axis], delta);

            if (pidProfile->dterm_lpf_hz) {
                if (pidProfile->dterm_filter_type == FILTER_BIQUAD)
                    delta = biquadFilterApply(&dtermFilterLpf[axis], delta);
                else if (pidProfile->dterm_filter_type == FILTER_PT1)
                    delta = pt1FilterApply4(&deltaFilter[axis], delta, pidProfile->dterm_lpf_hz, dT);
                else
                    delta = firFilterDenoiseUpdate(&dtermDenoisingState[axis], delta);
            }

            DTerm = Kd[axis] * delta * tpaFactor;

            // -----calculate total PID output
            axisPIDf[axis] = PTerm + ITerm + DTerm;
        } else {
            if (pidProfile->yaw_lpf_hz) PTerm = pt1FilterApply4(&yawFilter, PTerm, pidProfile->yaw_lpf_hz, dT);

            axisPIDf[axis] = PTerm + ITerm;

            DTerm = 0.0f; // needed for blackbox
        }

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

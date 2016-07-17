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

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"

#include "drivers/accgyro.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"

#include "config/runtime_config.h"

extern uint8_t motorCount;
uint32_t targetPidLooptime;
extern float errorLimiter;
extern float angleRate[3], angleRateSmooth[3];

int16_t axisPID[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t PIDweight[3];

static int32_t errorGyroI[3];
static float errorGyroIf[3];

static void pidBetaflight(const pidProfile_t *pidProfile, uint16_t max_angle_inclination,
        const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig);

pidControllerFuncPtr pid_controller = pidBetaflight; // which pid controller are we using

void setTargetPidLooptime(uint8_t pidProcessDenom)
{
    targetPidLooptime = gyro.targetLooptime * pidProcessDenom;
}

void pidResetErrorGyroState(void)
{
    int axis;

    for (axis = 0; axis < 3; axis++) {
        errorGyroI[axis] = 0;
        errorGyroIf[axis] = 0.0f;
    }
}

float getdT (void)
{
    static float dT;
    if (!dT) dT = (float)targetPidLooptime * 0.000001f;

    return dT;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static pt1Filter_t deltaFilter[3];
static pt1Filter_t yawFilter;

// Experimental betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage
static void pidBetaflight(const pidProfile_t *pidProfile, uint16_t max_angle_inclination,
         const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    float RateError = 0, gyroRate = 0, RateErrorSmooth = 0;
    float ITerm,PTerm,DTerm;
    static float lastRateError[2];
    float delta;
    int axis;
    float horizonLevelStrength = 1;

    float tpaFactor = PIDweight[0] / 100.0f; // tpa is now float

    // Scaling factors for Pids for better tunable range in configurator
    static const float PTermScale = 0.032029f;
    static const float ITermScale = 0.244381f;
    static const float DTermScale = 0.000529f;

    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the raw stick positions
        const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL, rxConfig->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH, rxConfig->midrc));
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
    for (axis = 0; axis < 3; axis++) {

        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && axis != YAW) {
            // calculate error angle and limit the angle to the max inclination
#ifdef GPS
            const float errorAngle = (constrain(2 * rcCommandSmooth[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]); // 16 bits is ok here
#else
            const float errorAngle = (constrain(2 * rcCommandSmooth[axis], -((int) max_angle_inclination),
                +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]); // 16 bits is ok here
#endif
            if (FLIGHT_MODE(ANGLE_MODE)) {
                // ANGLE mode - control is angle based, so control loop is needed
                angleRate[axis] = errorAngle * pidProfile->P8[PIDLEVEL] / 16.0f;
            } else {
                // HORIZON mode - direct sticks control is applied to rate PID
                // mix up angle error to desired AngleRate to add a little auto-level feel
                angleRate[axis] = angleRateSmooth[axis] + (errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 16.0f);
            }
        }

        gyroRate = gyroADCf[axis] / 16.4f; // gyro output deg/sec

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = angleRate[axis] - gyroRate;

        float dynReduction = tpaFactor;
        // Reduce Hunting effect and jittering near setpoint. Limit multiple zero crossing within deadband and lower PID affect during low error amount
        if (pidProfile->toleranceBand) {
            const float minReduction = (float)pidProfile->toleranceBandReduction / 100.0f;
            static uint8_t zeroCrossCount[3];
            static uint8_t currentErrorPolarity[3];
            if (ABS(RateError) < pidProfile->toleranceBand) {
                if (zeroCrossCount[axis]) {
                    if (currentErrorPolarity[axis] == POSITIVE_ERROR) {
                        if (RateError < 0 ) {
                            zeroCrossCount[axis]--;
                            currentErrorPolarity[axis] = NEGATIVE_ERROR;
                        }
                    } else {
                        if (RateError > 0 ) {
                            zeroCrossCount[axis]--;
                            currentErrorPolarity[axis] = POSITIVE_ERROR;
                        }
                    }
                } else {
                    dynReduction *= constrainf(ABS(RateError) / pidProfile->toleranceBand, minReduction, 1.0f);
                }
            } else {
                zeroCrossCount[axis] =  pidProfile->zeroCrossAllowanceCount;
                currentErrorPolarity[axis] = (RateError > 0) ? POSITIVE_ERROR : NEGATIVE_ERROR;
            }
        }

        if (pidProfile->deltaMethod == DELTA_FROM_ERROR) {
            // Smoothed Error for Derivative when delta from error selected
            if (flightModeFlags && axis != YAW)
                RateErrorSmooth = RateError;
            else
                RateErrorSmooth = angleRateSmooth[axis] - gyroRate;
        }

        // -----calculate P component
        PTerm = PTermScale * RateError * pidProfile->P8[axis] * dynReduction;

        // Constrain YAW by yaw_p_limit value if not servo driven in that case servolimits apply
        if((motorCount >= 4 && pidProfile->yaw_p_limit) && axis == YAW) {
            PTerm = constrainf(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
        }

        // -----calculate I component.
        // Prevent strong Iterm accumulation during stick inputs
        float accumulationThreshold = (axis == YAW) ? pidProfile->yawItermIgnoreRate : pidProfile->rollPitchItermIgnoreRate;
        float antiWindupScaler = constrainf(1.0f - (1.5f * (ABS(angleRate[axis]) / accumulationThreshold)), 0.0f, 1.0f);

        errorGyroIf[axis] = constrainf(errorGyroIf[axis] + ITermScale * RateError * getdT() * pidProfile->I8[axis] * antiWindupScaler * kiThrottleGain, -250.0f, 250.0f);

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        ITerm = errorGyroIf[axis];

        //-----calculate D-term
        if (axis == YAW) {
            if (pidProfile->yaw_lpf_hz) PTerm = pt1FilterApply4(&yawFilter, PTerm, pidProfile->yaw_lpf_hz, getdT());

            axisPID[axis] = lrintf(PTerm + ITerm);

            if (motorCount >= 4) {
                int16_t yaw_jump_prevention_limit = constrain(YAW_JUMP_PREVENTION_LIMIT_HIGH - (pidProfile->D8[axis] << 3), YAW_JUMP_PREVENTION_LIMIT_LOW, YAW_JUMP_PREVENTION_LIMIT_HIGH);

                // prevent "yaw jump" during yaw correction
                axisPID[YAW] = constrain(axisPID[YAW], -yaw_jump_prevention_limit - ABS(rcCommand[YAW]), yaw_jump_prevention_limit + ABS(rcCommand[YAW]));
            }

            DTerm = 0.0f; // needed for blackbox
        } else {
            if (pidProfile->deltaMethod == DELTA_FROM_ERROR) {
                delta = RateErrorSmooth - lastRateError[axis];
                lastRateError[axis] = RateErrorSmooth;
            } else {
                delta = -(gyroRate - lastRateError[axis]);
                lastRateError[axis] = gyroRate;
            }

            // Divide delta by targetLooptime to get differential (ie dr/dt)
            delta *= (1.0f / getdT());

            // Filter delta
            if (pidProfile->dterm_lpf_hz) delta = pt1FilterApply4(&deltaFilter[axis], delta, pidProfile->dterm_lpf_hz, getdT());

            DTerm = constrainf(DTermScale * delta * (float)pidProfile->D8[axis] * dynReduction, -300.0f, 300.0f);

            // -----calculate total PID output
            axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm), -1000, 1000);
        }

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

// Legacy pid controller betaflight evolved pid rewrite based on 2.9 releae. Good for fastest cycletimes for those who believe in that. Don't expect much development in the future
static void pidLegacy(const pidProfile_t *pidProfile, uint16_t max_angle_inclination,
        const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    int axis;
    int32_t PTerm, ITerm, DTerm, delta;
    static int32_t lastRateError[3];
    int32_t AngleRateTmp = 0, AngleRateTmpSmooth = 0, RateError = 0, gyroRate = 0, RateErrorSmooth = 0;

    int8_t horizonLevelStrength = 100;

    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the raw stick positions
        const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL, rxConfig->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH, rxConfig->midrc));
        const int32_t mostDeflectedPos = MAX(stickPosAil, stickPosEle);
        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (500 - mostDeflectedPos) / 5;  // 100 at centre stick, 0 = max stick deflection
        // Using Level D as a Sensitivity for Horizon. 0 more level to 255 more rate. Default value of 100 seems to work fine.
        // For more rate mode increase D and slower flips and rolls will be possible
        horizonLevelStrength = constrain((10 * (horizonLevelStrength - 100) * (10 * pidProfile->D8[PIDLEVEL] / 80) / 100) + 100, 0, 100);
    }

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {

        // -----Get the desired angle rate depending on flight mode
        AngleRateTmp = (int32_t)angleRate[axis];
        if (axis != YAW) AngleRateTmpSmooth = (int32_t)angleRateSmooth[axis];

        if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && axis != YAW) {
            // calculate error angle and limit the angle to max configured inclination
#ifdef GPS
            const int32_t errorAngle = constrain(2 * rcCommandSmooth[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis];
#else
            const int32_t errorAngle = constrain(2 * rcCommandSmooth[axis], -((int) max_angle_inclination),
                +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis];
#endif
            if (FLIGHT_MODE(ANGLE_MODE)) {
                // ANGLE mode - control is angle based, so control loop is needed
                AngleRateTmp = (errorAngle * pidProfile->P8[PIDLEVEL]) >> 4;
            } else {
                // HORIZON mode - mix up angle error to desired AngleRateTmp to add a little auto-level feel,
                // horizonLevelStrength is scaled to the stick input
                AngleRateTmp =  AngleRateTmpSmooth + ((errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 100) >> 4);
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        gyroRate = gyroADC[axis] / 4;

        RateError = AngleRateTmp - gyroRate;

        if (pidProfile->deltaMethod == DELTA_FROM_ERROR) {
            // Smoothed Error for Derivative when delta from error selected
            if (flightModeFlags && axis != YAW)
                RateErrorSmooth = RateError;
            else
                RateErrorSmooth = AngleRateTmpSmooth - gyroRate;
        }

        // -----calculate P component
        PTerm = (RateError * pidProfile->P8[axis] * PIDweight[axis] / 100) >> 7;

        // Constrain YAW by yaw_p_limit value if not servo driven in that case servolimits apply
        if((motorCount >= 4 && pidProfile->yaw_p_limit) && axis == YAW) {
            PTerm = constrain(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
        }

        // -----calculate I component
        // there should be no division before accumulating the error to integrator, because the precision would be reduced.
        // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        // Time correction (to avoid different I scaling for different builds based on average cycle time)
        // is normalized to cycle time = 2048.
        // Prevent Accumulation
        uint16_t resetRate = (axis == YAW) ? pidProfile->yawItermIgnoreRate : pidProfile->rollPitchItermIgnoreRate;
        uint16_t dynamicFactor = (1 << 8) - constrain(((ABS(AngleRateTmp) << 6) / resetRate), 0, 1 << 8);
        uint16_t dynamicKi = (pidProfile->I8[axis] * dynamicFactor) >> 8;

        errorGyroI[axis] = errorGyroI[axis] + ((RateError * (uint16_t)targetPidLooptime) >> 11) * dynamicKi;

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t) - GYRO_I_MAX << 13, (int32_t) + GYRO_I_MAX << 13);

        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        if (axis == YAW) {
            if (pidProfile->yaw_lpf_hz) PTerm = pt1FilterApply4(&yawFilter, PTerm, pidProfile->yaw_lpf_hz, getdT());

            axisPID[axis] = PTerm + ITerm;

            if (motorCount >= 4) {
                int16_t yaw_jump_prevention_limit = constrain(YAW_JUMP_PREVENTION_LIMIT_HIGH - (pidProfile->D8[axis] << 3), YAW_JUMP_PREVENTION_LIMIT_LOW, YAW_JUMP_PREVENTION_LIMIT_HIGH);

                // prevent "yaw jump" during yaw correction
                axisPID[YAW] = constrain(axisPID[YAW], -yaw_jump_prevention_limit - ABS(rcCommand[YAW]), yaw_jump_prevention_limit + ABS(rcCommand[YAW]));
            }

            DTerm = 0; // needed for blackbox
        } else {
            if (pidProfile->deltaMethod == DELTA_FROM_ERROR) {
                delta = RateErrorSmooth - lastRateError[axis];
                lastRateError[axis] = RateErrorSmooth;
            } else {
                delta = -(gyroRate - lastRateError[axis]);
                lastRateError[axis] = gyroRate;
            }

            // Divide delta by targetLooptime to get differential (ie dr/dt)
            delta = (delta * ((uint16_t) 0xFFFF / ((uint16_t)targetPidLooptime >> 4))) >> 5;

            // Filter delta
            if (pidProfile->dterm_lpf_hz) delta = pt1FilterApply4(&deltaFilter[axis], (float)delta, pidProfile->dterm_lpf_hz, getdT());

            DTerm = (delta * pidProfile->D8[axis] * PIDweight[axis] / 100) >> 8;

            // -----calculate total PID output
            axisPID[axis] = PTerm + ITerm + DTerm;
        }


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

void pidSetController(pidControllerType_e type)
{
    switch (type) {
        default:
        case PID_CONTROLLER_LEGACY:
            pid_controller = pidLegacy;
            break;
        case PID_CONTROLLER_BETAFLIGHT:
            pid_controller = pidBetaflight;
    }
}


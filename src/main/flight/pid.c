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
#include "drivers/gyro_sync.h"

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

extern float dT;
extern bool motorLimitReached;

#define PREVENT_WINDUP(x,y) { if (ABS(x) > ABS(y)) { if (x < 0) { x = -ABS(y); }  else { x = ABS(y); } } }

int16_t axisPID[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

#define DELTA_TOTAL_SAMPLES 3

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t PIDweight[3];

static int32_t errorGyroI[3], previousErrorGyroI[3];
static float errorGyroIf[3], previousErrorGyroIf[3];

static void pidRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig);

typedef void (*pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig);            // pid controller function prototype

pidControllerFuncPtr pid_controller = pidRewrite; // which pid controller are we using, defaultMultiWii

void pidResetErrorGyro(rxConfig_t *rxConfig)
{
    int axis;

    for (axis = 0; axis < 3; axis++) {
        if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
            int rollPitchStatus = calculateRollPitchCenterStatus(rxConfig);
            if (rollPitchStatus == CENTERED) {
                PREVENT_WINDUP(errorGyroI[axis], previousErrorGyroI[axis]);
                PREVENT_WINDUP(errorGyroIf[axis], previousErrorGyroIf[axis]);
            }
        } else {
            errorGyroI[axis] = 0;
            errorGyroIf[axis] = 0.0f;
        }
    }
}

float scaleItermToRcInput(int axis) {
    float rcCommandReflection = (float)rcCommand[axis] / 500.0f;
    static float iTermScaler[3] = {1.0f, 1.0f, 1.0f};

    if (ABS(rcCommandReflection) > 0.7f && (!flightModeFlags)) {   /* scaling should not happen in level modes */
        /* Reset Iterm on high stick inputs. No scaling necessary here */
        iTermScaler[axis] = 0.0f;
    } else {
        /* Prevent rapid windup during acro recoveries. Slowly enable Iterm activity. Perhaps more scaling to looptime needed for consistency */
        if (iTermScaler[axis] < 1) {
            iTermScaler[axis] = constrainf(iTermScaler[axis] + 0.001f, 0.0f, 1.0f);
        } else {
            iTermScaler[axis] = 1;
        }
    }
    return iTermScaler[axis];
}

void acroPlusApply(acroPlus_t *axisState, int axis, pidProfile_t *pidProfile) {
    float rcCommandReflection = (float)rcCommand[axis] / 500.0f;
    axisState->wowFactor = 1;
    axisState->factor = 0;

    /* acro plus factor handling */
    if (axis != YAW && pidProfile->airModeInsaneAcrobilityFactor && (!flightModeFlags)) {
        axisState->wowFactor = ABS(rcCommandReflection) * ((float)pidProfile->airModeInsaneAcrobilityFactor / 100.0f); //0-1f
        axisState->factor = axisState->wowFactor * rcCommandReflection * 1000;
        axisState->wowFactor = 1.0f - axisState->wowFactor;
    }
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static acroPlus_t acroPlusState[3];
static biquad_t deltaBiQuadState[3];
static bool deltaStateIsSet;

static void pidLuxFloat(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig)
{
    float RateError, AngleRate, gyroRate;
    float ITerm,PTerm,DTerm;
    static float lastError[3], lastGyroRate[3];
    static float previousDelta[3][DELTA_TOTAL_SAMPLES];
    float delta, deltaSum;
    int axis, deltaCount;
    float horizonLevelStrength = 1;

    if (!deltaStateIsSet && pidProfile->dterm_lpf_hz) {
        for (axis = 0; axis < 3; axis++) BiQuadNewLpf(pidProfile->dterm_lpf_hz, &deltaBiQuadState[axis], targetLooptime);
        deltaStateIsSet = true;
    }

    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the raw stick positions
        const int32_t stickPosAil = ABS(getRcStickDeflection(FD_ROLL, rxConfig->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(FD_PITCH, rxConfig->midrc));
        const int32_t mostDeflectedPos = MAX(stickPosAil, stickPosEle);
        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
        if(pidProfile->H_sensitivity == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / pidProfile->H_sensitivity)) + 1, 0, 1);
        }
    }

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        uint8_t rate = 30;
        // -----Get the desired angle rate depending on flight mode
        if (axis == YAW || !pidProfile->airModeInsaneAcrobilityFactor || !IS_RC_MODE_ACTIVE(BOXACROPLUS)) {
            rate = controlRateConfig->rates[axis];
        }

        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            AngleRate = (float)((rate + 10) * rcCommand[YAW]) / 50.0f;
         } else {
             // ACRO mode, control is GYRO based, direct sticks control is applied to rate PID
             AngleRate = (float)((rate + 20) * rcCommand[axis]) / 50.0f; // 200dps to 1200dps max roll/pitch rate
             if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
                // calculate error angle and limit the angle to the max inclination
#ifdef GPS
                const float errorAngle = (constrain(rcCommand[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f; // 16 bits is ok here
#else
                const float errorAngle = (constrain(rcCommand[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f; // 16 bits is ok here
#endif
                if (FLIGHT_MODE(ANGLE_MODE)) {
                    // ANGLE mode - control is angle based, so control loop is needed
                    AngleRate = errorAngle * pidProfile->A_level;
                } else {
                    // HORIZON mode - direct sticks control is applied to rate PID
                    // mix up angle error to desired AngleRate to add a little auto-level feel
                    AngleRate += errorAngle * pidProfile->H_level * horizonLevelStrength;
                }
            }
        }

        gyroRate = gyroADC[axis] * gyro.scale; // gyro output scaled to dps

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRate - gyroRate;

        // -----calculate P component
        PTerm = RateError * pidProfile->P_f[axis] * PIDweight[axis] / 100;

        // -----calculate I component.
        errorGyroIf[axis] = constrainf(errorGyroIf[axis] + RateError * dT * pidProfile->I_f[axis] * 10, -250.0f, 250.0f);

        if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
            errorGyroIf[axis] *= scaleItermToRcInput(axis);
            if (motorLimitReached) {
                PREVENT_WINDUP(errorGyroIf[axis], previousErrorGyroIf[axis]);
            } else {
                previousErrorGyroIf[axis] = errorGyroIf[axis];
            }
        }

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        ITerm = errorGyroIf[axis];

        //-----calculate D-term
        if (!pidProfile->deltaFromGyro) {
            delta = RateError - lastError[axis];
            lastError[axis] = RateError;
        } else {
            delta = -(gyroRate - lastGyroRate[axis]);  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
            lastGyroRate[axis] = gyroRate;
        }

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta *= (1.0f / dT);

        if (deltaStateIsSet) {
            delta = applyBiQuadFilter(delta, &deltaBiQuadState[axis]);
        } else {
            // Apply moving average
            deltaSum = 0;
            for (deltaCount = DELTA_TOTAL_SAMPLES-1; deltaCount > 0; deltaCount--) previousDelta[axis][deltaCount] = previousDelta[axis][deltaCount-1];
            previousDelta[axis][0] = delta;
            for (deltaCount = 0; deltaCount < DELTA_TOTAL_SAMPLES; deltaCount++) deltaSum += previousDelta[axis][deltaCount];
            delta = (deltaSum / DELTA_TOTAL_SAMPLES);
        }

        DTerm = constrainf(delta * pidProfile->D_f[axis] * PIDweight[axis] / 100, -300.0f, 300.0f);

        // -----calculate total PID output
        axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm), -1000, 1000);

        if (IS_RC_MODE_ACTIVE(BOXACROPLUS)) {
            acroPlusApply(&acroPlusState[axis], axis, pidProfile);
            axisPID[axis] = acroPlusState[axis].factor + acroPlusState[axis].wowFactor * axisPID[axis];
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

static void pidRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination,
        rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);

    int axis, deltaCount;
    int32_t PTerm, ITerm, DTerm, delta, deltaSum;
    static int32_t lastError[3] = { 0, 0, 0 };
    static int32_t previousDelta[3][DELTA_TOTAL_SAMPLES];
    static int32_t lastGyroRate[3];
    int32_t AngleRateTmp, RateError, gyroRate;

    int8_t horizonLevelStrength = 100;

    if (!deltaStateIsSet && pidProfile->dterm_lpf_hz) {
        for (axis = 0; axis < 3; axis++) BiQuadNewLpf(pidProfile->dterm_lpf_hz, &deltaBiQuadState[axis], targetLooptime);
        deltaStateIsSet = true;
    }

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
        uint8_t rate = 30;
        // -----Get the desired angle rate depending on flight mode
        if (axis == YAW || !pidProfile->airModeInsaneAcrobilityFactor || !IS_RC_MODE_ACTIVE(BOXACROPLUS)) {
            rate = controlRateConfig->rates[axis];
        }

        // -----Get the desired angle rate depending on flight mode
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            AngleRateTmp = ((int32_t)(rate + 27) * rcCommand[YAW]) >> 5;
        } else {
            AngleRateTmp = ((int32_t)(rate + 27) * rcCommand[axis]) >> 4;
            if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
                // calculate error angle and limit the angle to max configured inclination
#ifdef GPS
                const int32_t errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis];
#else
                const int32_t errorAngle = constrain(2 * rcCommand[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis];
#endif
                if (FLIGHT_MODE(ANGLE_MODE)) {
                    // ANGLE mode - control is angle based, so control loop is needed
                    AngleRateTmp = (errorAngle * pidProfile->P8[PIDLEVEL]) >> 4;
                } else {
                    // HORIZON mode - mix up angle error to desired AngleRateTmp to add a little auto-level feel,
                    // horizonLevelStrength is scaled to the stick input
                    AngleRateTmp += (errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 100) >> 4;
                }
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        gyroRate = gyroADC[axis] / 4;
        RateError = AngleRateTmp - gyroRate;

        // -----calculate P component
        PTerm = (RateError * pidProfile->P8[axis] * PIDweight[axis] / 100) >> 7;

        // -----calculate I component
        // there should be no division before accumulating the error to integrator, because the precision would be reduced.
        // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        // Time correction (to avoid different I scaling for different builds based on average cycle time)
        // is normalized to cycle time = 2048.
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * (uint16_t)targetLooptime) >> 11) * pidProfile->I8[axis];

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t) - GYRO_I_MAX << 13, (int32_t) + GYRO_I_MAX << 13);

        if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
            errorGyroI[axis] = (int32_t) (errorGyroI[axis] * scaleItermToRcInput(axis));
            if (motorLimitReached) {
                PREVENT_WINDUP(errorGyroIf[axis], previousErrorGyroIf[axis]);
            } else {
                previousErrorGyroI[axis] = errorGyroI[axis];
            }
        }

        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        if (!pidProfile->deltaFromGyro) {   // quick and dirty solution for testing
            delta = RateError - lastError[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
            lastError[axis] = RateError;
        } else {
            delta = -(gyroRate - lastGyroRate[axis]);  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
            lastGyroRate[axis] = gyroRate;
        }
        
        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = (delta * ((uint16_t) 0xFFFF / ((uint16_t)targetLooptime >> 4))) >> 6;

        if (deltaStateIsSet) {
            delta = lrintf(applyBiQuadFilter((float) delta, &deltaBiQuadState[axis])) * 3;  // Keep same scaling as unfiltered delta
        } else {
            // Apply moving average
            deltaSum = 0;
            for (deltaCount = DELTA_TOTAL_SAMPLES -1; deltaCount > 0; deltaCount--) previousDelta[axis][deltaCount] = previousDelta[axis][deltaCount-1];
            previousDelta[axis][0] = delta;
            for (deltaCount = 0; deltaCount < DELTA_TOTAL_SAMPLES; deltaCount++) deltaSum += previousDelta[axis][deltaCount];
            delta = deltaSum;
        }

        DTerm = (delta * pidProfile->D8[axis] * PIDweight[axis] / 100) >> 8;

        // -----calculate total PID output
        axisPID[axis] = PTerm + ITerm + DTerm;

        if (IS_RC_MODE_ACTIVE(BOXACROPLUS)) {
            acroPlusApply(&acroPlusState[axis], axis, pidProfile);
            axisPID[axis] = lrintf(acroPlusState[axis].factor + acroPlusState[axis].wowFactor * axisPID[axis]);
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
        case PID_CONTROLLER_MWREWRITE:
            pid_controller = pidRewrite;
            break;
        case PID_CONTROLLER_LUX_FLOAT:
            pid_controller = pidLuxFloat;
    }
}


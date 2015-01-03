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

#include "common/axis.h"
#include "common/maths.h"

#include "config/runtime_config.h"

#include "rx/rx.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"

#include "io/rc_controls.h"
#include "flight/flight.h"
#include "flight/navigation.h"
#include "flight/autotune.h"
#include "io/gps.h"

extern uint16_t cycleTime;

int16_t heading, magHold;
int16_t axisPID[3];
uint8_t dynP8[3], dynI8[3], dynD8[3];

static int32_t errorGyroI[3] = { 0, 0, 0 };
static float errorGyroIf[3] = { 0.0f, 0.0f, 0.0f };
static int32_t errorAngleI[2] = { 0, 0 };

static void pidMultiWii(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim);

typedef void (*pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim);            // pid controller function prototype

pidControllerFuncPtr pid_controller = pidMultiWii; // which pid controller are we using, defaultMultiWii

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    rollAndPitchTrims->values.roll = 0;
    rollAndPitchTrims->values.pitch = 0;
}

void resetErrorAngle(void)
{
    errorAngleI[ROLL] = 0;
    errorAngleI[PITCH] = 0;
}

void resetErrorGyro(void)
{
    errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
    errorGyroI[YAW] = 0;

    errorGyroIf[ROLL] = 0;
    errorGyroIf[PITCH] = 0;
    errorGyroIf[YAW] = 0;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

#ifdef AUTOTUNE
bool shouldAutotune(void)
{
    return ARMING_FLAG(ARMED) && FLIGHT_MODE(AUTOTUNE_MODE);
}
#endif

static void pidBaseflight(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim)
{
    float RateError, errorAngle, AngleRate, gyroRate;
    float ITerm,PTerm,DTerm;
    static float lastGyroRate[3];
    static float delta1[3], delta2[3];
    float delta, deltaSum;
    float dT;
    int axis;

    dT = (float)cycleTime * 0.000001f;

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            AngleRate = (float)((controlRateConfig->yawRate + 10) * rcCommand[YAW]) / 50.0f;
         } else {
            // calculate error and limit the angle to the max inclination
#ifdef GPS
            errorAngle = (constrain(rcCommand[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - inclination.raw[axis] + angleTrim->raw[axis]) / 10.0f; // 16 bits is ok here
#else
            errorAngle = (constrain(rcCommand[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - inclination.raw[axis] + angleTrim->raw[axis]) / 10.0f; // 16 bits is ok here
#endif

#ifdef AUTOTUNE
            if (shouldAutotune()) {
                errorAngle = autotune(rcAliasToAngleIndexMap[axis], &inclination, errorAngle);
            }
#endif

            if (FLIGHT_MODE(ANGLE_MODE)) {
                // it's the ANGLE mode - control is angle based, so control loop is needed
                AngleRate = errorAngle * pidProfile->A_level;
            } else {
                //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                AngleRate = (float)((controlRateConfig->rollPitchRate + 20) * rcCommand[axis]) / 50.0f; // 200dps to 1200dps max yaw rate
                if (FLIGHT_MODE(HORIZON_MODE)) {
                    // mix up angle error to desired AngleRate to add a little auto-level feel
                    AngleRate += errorAngle * pidProfile->H_level;
                }
            }
        }

        gyroRate = gyroData[axis] * gyro.scale; // gyro output scaled to dps

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRate - gyroRate;

        // -----calculate P component
        PTerm = RateError * pidProfile->P_f[axis];
        // -----calculate I component
        errorGyroIf[axis] = constrainf(errorGyroIf[axis] + RateError * dT * pidProfile->I_f[axis], -250.0f, 250.0f);

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        ITerm = errorGyroIf[axis];

        //-----calculate D-term
        delta = gyroRate - lastGyroRate[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastGyroRate[axis] = gyroRate;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta *= (1.0f / dT);
        // add moving average here to reduce noise
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = constrainf((deltaSum / 3.0f) * pidProfile->D_f[axis], -300.0f, 300.0f);

        // -----calculate total PID output
        axisPID[axis] = constrain(lrintf(PTerm + ITerm - DTerm), -1000, 1000);
    }

}

static void pidMultiWii(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim)
{
    int axis, prop;
    int32_t error, errorAngle;
    int32_t PTerm, ITerm, PTermACC = 0, ITermACC = 0, PTermGYRO = 0, ITermGYRO = 0, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int32_t delta1[3], delta2[3];
    int32_t deltaSum;
    int32_t delta;

    UNUSED(controlRateConfig);

    // **** PITCH & ROLL & YAW PID ****
    prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]

    for (axis = 0; axis < 3; axis++) {
        if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && (axis == FD_ROLL || axis == FD_PITCH)) { // MODE relying on ACC
            // observe max inclination
#ifdef GPS
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - inclination.raw[axis] + angleTrim->raw[axis];
#else
            errorAngle = constrain(2 * rcCommand[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - inclination.raw[axis] + angleTrim->raw[axis];
#endif

#ifdef AUTOTUNE
            if (shouldAutotune()) {
                errorAngle = DEGREES_TO_DECIDEGREES(autotune(rcAliasToAngleIndexMap[axis], &inclination, DECIDEGREES_TO_DEGREES(errorAngle)));
            }
#endif

            PTermACC = errorAngle * pidProfile->P8[PIDLEVEL] / 100; // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
            PTermACC = constrain(PTermACC, -pidProfile->D8[PIDLEVEL] * 5, +pidProfile->D8[PIDLEVEL] * 5);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
            ITermACC = (errorAngleI[axis] * pidProfile->I8[PIDLEVEL]) >> 12;
        }
        if (!FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || axis == FD_YAW) { // MODE relying on GYRO or YAW axis
            error = (int32_t) rcCommand[axis] * 10 * 8 / pidProfile->P8[axis];
            error -= gyroData[axis] / 4;

            PTermGYRO = rcCommand[axis];

            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
            if ((abs(gyroData[axis]) > (640 * 4)) || (axis == FD_YAW && abs(rcCommand[axis]) > 100))
                errorGyroI[axis] = 0;

            ITermGYRO = (errorGyroI[axis] / 125 * pidProfile->I8[axis]) / 64;
        }
        if (FLIGHT_MODE(HORIZON_MODE) && (axis == FD_ROLL || axis == FD_PITCH)) {
            PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500;
            ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500;
        } else {
            if (FLIGHT_MODE(ANGLE_MODE) && (axis == FD_ROLL || axis == FD_PITCH)) {
                PTerm = PTermACC;
                ITerm = ITermACC;
            } else {
                PTerm = PTermGYRO;
                ITerm = ITermGYRO;
            }
        }

        PTerm -= ((int32_t)gyroData[axis] / 4) * dynP8[axis] / 10 / 8; // 32 bits is needed for calculation
        delta = (gyroData[axis] - lastGyro[axis]) / 4;
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * dynD8[axis]) / 32;
        axisPID[axis] = PTerm + ITerm - DTerm;
    }
}

#define GYRO_I_MAX 256

static void pidRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination,
        rollAndPitchTrims_t *angleTrim)
{
    int32_t errorAngle;
    int axis;
    int32_t delta, deltaSum;
    static int32_t delta1[3], delta2[3];
    int32_t PTerm, ITerm, DTerm;
    static int32_t lastError[3] = { 0, 0, 0 };
    int32_t AngleRateTmp, RateError;

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        if (axis == FD_YAW) { // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            AngleRateTmp = (((int32_t)(controlRateConfig->yawRate + 27) * rcCommand[YAW]) >> 5);
        } else {
            // calculate error and limit the angle to max configured inclination
#ifdef GPS
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - inclination.raw[axis] + angleTrim->raw[axis]; // 16 bits is ok here
#else
            errorAngle = constrain(2 * rcCommand[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - inclination.raw[axis] + angleTrim->raw[axis]; // 16 bits is ok here
#endif

#ifdef AUTOTUNE
            if (shouldAutotune()) {
                errorAngle = DEGREES_TO_DECIDEGREES(autotune(rcAliasToAngleIndexMap[axis], &inclination, DECIDEGREES_TO_DEGREES(errorAngle)));
            }
#endif

            if (!FLIGHT_MODE(ANGLE_MODE)) { //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                AngleRateTmp = ((int32_t)(controlRateConfig->rollPitchRate + 27) * rcCommand[axis]) >> 4;
                if (FLIGHT_MODE(HORIZON_MODE)) {
                    // mix up angle error to desired AngleRateTmp to add a little auto-level feel
                    AngleRateTmp += (errorAngle * pidProfile->I8[PIDLEVEL]) >> 8;
                }
            } else { // it's the ANGLE mode - control is angle based, so control loop is needed
                AngleRateTmp = (errorAngle * pidProfile->P8[PIDLEVEL]) >> 4;
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRateTmp - (gyroData[axis] / 4);

        // -----calculate P component
        PTerm = (RateError * pidProfile->P8[axis]) >> 7;
        // -----calculate I component
        // there should be no division before accumulating the error to integrator, because the precision would be reduced.
        // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        // Time correction (to avoid different I scaling for different builds based on average cycle time)
        // is normalized to cycle time = 2048.
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * cycleTime) >> 11) * pidProfile->I8[axis];

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t) - GYRO_I_MAX << 13, (int32_t) + GYRO_I_MAX << 13);
        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        delta = RateError - lastError[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = (delta * ((uint16_t) 0xFFFF / (cycleTime >> 4))) >> 6;
        // add moving average here to reduce noise
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * pidProfile->D8[axis]) >> 8;

        // -----calculate total PID output
        axisPID[axis] = PTerm + ITerm + DTerm;
    }
}

void setPIDController(int type)
{
    switch (type) {
        case 0:
        default:
            pid_controller = pidMultiWii;
            break;
        case 1:
            pid_controller = pidRewrite;
            break;
        case 2:
            pid_controller = pidBaseflight;
    }
}


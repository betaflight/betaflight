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

#define SRC_MAIN_FLIGHT_PID_LUXFLOAT_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/runtime_config.h"
#include "config/config_unittest.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/rate_profile.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"
#include "flight/mixer.h"

extern float dT;
extern uint8_t PIDweight[3];
extern float lastITermf[3], ITermLimitf[3];

extern biquad_t deltaFilterState[3];

#ifdef BLACKBOX
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

STATIC_UNIT_TESTED int16_t pidLuxFloatCore(int axis, const pidProfile_t *pidProfile, float gyroRate, float AngleRate)
{
    static float lastErrorForDelta[3];
    static float delta1[3], delta2[3];

    SET_PID_LUX_FLOAT_CORE_LOCALS(axis);

    const float RateError = AngleRate - gyroRate;

    // -----calculate P component
    const float PTerm = RateError * pidProfile->P8[axis] * PIDweight[axis] / 100;

    // -----calculate I component
    float ITerm = lastITermf[axis] + RateError * dT * pidProfile->I8[axis] * 10;
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    ITerm = constrainf(ITerm, -PID_LUX_FLOAT_MAX_I, PID_LUX_FLOAT_MAX_I);
    // Anti windup protection
    if (IS_RC_MODE_ACTIVE(BOXAIRMODE)) {
        ITerm = ITerm * pidScaleItermToRcInput(axis);
        if (STATE(ANTI_WINDUP) || motorLimitReached) {
            ITerm = constrainf(ITerm, -ITermLimitf[axis], ITermLimitf[axis]);
        } else {
            ITermLimitf[axis] = ABS(ITerm);
        }
    }
    lastITermf[axis] = ITerm;

    // -----calculate D component
    float delta;
    if (pidProfile->deltaMethod == DELTA_FROM_ERROR) {
        delta = RateError - lastErrorForDelta[axis];
        lastErrorForDelta[axis] = RateError;
    } else {
        // Delta from measurement
        delta = -(gyroRate - lastErrorForDelta[axis]);
        lastErrorForDelta[axis] = gyroRate;
    }
    // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
    // would be scaled by different dt each time. Division by dT fixes that.
    delta *= (1.0f / dT);
    float deltaSum;
    if (pidProfile->dterm_cut_hz) {
        // Dterm delta low pass
        delta = applyBiQuadFilter(delta, &deltaFilterState[axis]);
    } else {
        // When dterm filter disabled apply moving average to reduce noise
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        delta = deltaSum / 3.0f;
    }
    const float DTerm = constrainf(delta * pidProfile->D8[axis] * PIDweight[axis] / 100, -PID_LUX_FLOAT_MAX_D, PID_LUX_FLOAT_MAX_D);

#ifdef BLACKBOX
    axisPID_P[axis] = PTerm;
    axisPID_I[axis] = ITerm;
    axisPID_D[axis] = DTerm;
#endif
    GET_PID_LUX_FLOAT_CORE_LOCALS(axis);
    // -----calculate total PID output
    return constrain(lrintf(PTerm + ITerm + DTerm), -PID_LUX_FLOAT_MAX_PID, PID_LUX_FLOAT_MAX_PID);
}

void pidLuxFloat(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    float horizonLevelStrength = 1;

    pidFilterIsSetCheck(pidProfile);

    if (FLIGHT_MODE(HORIZON_MODE)) {

        // Figure out the raw stick positions
        const int32_t stickPosAil = ABS(getRcStickDeflection(ROLL, rxConfig->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(PITCH, rxConfig->midrc));
        const int32_t mostDeflectedPos =  MAX(stickPosAil, stickPosEle);

        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
        if(pidProfile->D8[PIDLEVEL] == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / pidProfile->D8[PIDLEVEL])) + 1, 0, 1);
        }
    }

    // ----------PID controller----------
    for (int axis = 0; axis < 3; axis++) {
        const uint8_t rate = controlRateConfig->rates[axis];

        float AngleRate;
        // -----Get the desired angle rate depending on flight mode
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            AngleRate = (float)((rate + 10) * rcCommand[YAW]) / 50.0f;
        } else {
            // control is GYRO based (ACRO and HORIZON) - direct sticks control is applied to rate PID
            AngleRate = (float)((rate + 20) * rcCommand[axis]) / 50.0f; // 200dps to 1200dps max roll/pitch rate
            if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
               // calculate error angle and limit the angle to the max inclination
#ifdef GPS
                const float errorAngle = (constrain(rcCommand[axis] + GPS_angle[axis], -((int)max_angle_inclination), max_angle_inclination)
                        - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f;
#else
                const float errorAngle = (constrain(rcCommand[axis], -((int)max_angle_inclination), max_angle_inclination)
                        - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f;
#endif
                if (FLIGHT_MODE(ANGLE_MODE)) {
                    // ANGLE mode - control is angle based, so control loop is needed
                    AngleRate = errorAngle * pidProfile->P8[PIDLEVEL];
                } else {
                    // HORIZON mode - direct sticks control is applied to rate PID
                    // mix up angle error to desired AngleRate to add a little auto-level feel
                    AngleRate += errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength;
                }
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        const float gyroRate = gyroADC[axis] * gyro.scale; // gyro output scaled to dps
        axisPID[axis] = pidLuxFloatCore(axis, pidProfile, gyroRate, AngleRate);

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif
    }
}


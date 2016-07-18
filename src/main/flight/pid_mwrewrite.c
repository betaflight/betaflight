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

#define SRC_MAIN_FLIGHT_PID_MWREWRITE_C_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "fc/rc_controls.h"
#include "fc/rate_profile.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "config/config_unittest.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"
#include "flight/mixer.h"


extern float dT;
extern uint8_t PIDweight[3];
extern int32_t lastITerm[3], ITermLimit[3];

extern pt1Filter_t deltaFilter[3];
extern pt1Filter_t yawFilter;

extern uint8_t motorCount;

#ifdef BLACKBOX
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif


STATIC_UNIT_TESTED int16_t pidMultiWiiRewriteCore(int axis, const pidProfile_t *pidProfile, int32_t gyroRate, int32_t angleRate)
{
    static int32_t lastRateForDelta[3];

    SET_PID_MULTI_WII_REWRITE_CORE_LOCALS(axis);

    const int32_t rateError = angleRate - gyroRate;

    // -----calculate P component
    int32_t PTerm = (rateError * pidProfile->P8[axis] * PIDweight[axis] / 100) >> 7;
    // Constrain YAW by yaw_p_limit value if not servo driven, in that case servolimits apply
    if (axis == YAW) {
        if (pidProfile->yaw_lpf) {
            PTerm = pt1FilterApply4(&yawFilter, PTerm, pidProfile->yaw_lpf, dT);
        }
        if (pidProfile->yaw_p_limit && motorCount >= 4) {
            PTerm = constrain(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
        }
    }

    // -----calculate I component
    // There should be no division before accumulating the error to integrator, because the precision would be reduced.
    // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator (Q19.13 format) is used.
    // Time correction (to avoid different I scaling for different builds based on average cycle time)
    // is normalized to cycle time = 2048 (2^11).
    int32_t ITerm = lastITerm[axis] + ((rateError * (uint16_t)targetLooptime) >> 11) * pidProfile->I8[axis];
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    ITerm = constrain(ITerm, (int32_t)(-PID_MAX_I << 13), (int32_t)(PID_MAX_I << 13));
    // Anti windup protection
    if (rcModeIsActive(BOXAIRMODE)) {
        if (STATE(ANTI_WINDUP) || motorLimitReached) {
            ITerm = constrain(ITerm, -ITermLimit[axis], ITermLimit[axis]);
        } else {
            ITermLimit[axis] = ABS(ITerm);
        }
    }
    lastITerm[axis] = ITerm;
    ITerm = ITerm >> 13; // take integer part of Q19.13 value

    // -----calculate D component
    int32_t DTerm;
    if (pidProfile->D8[axis] == 0) {
        // optimisation for when D8 is zero, often used by YAW axis
        DTerm = 0;
    } else {
        int32_t delta;
        if (pidProfile->deltaMethod == PID_DELTA_FROM_MEASUREMENT) {
            delta = -(gyroRate - lastRateForDelta[axis]);
            lastRateForDelta[axis] = gyroRate;
        } else {
            delta = rateError - lastRateForDelta[axis];
            lastRateForDelta[axis] = rateError;
        }
        // Divide delta by targetLooptime to get differential (ie dr/dt)
        delta = (delta * ((uint16_t)0xFFFF / ((uint16_t)targetLooptime >> 4))) >> 5;
        if (pidProfile->dterm_lpf) {
            // DTerm delta low pass filter
            delta = lrintf(pt1FilterApply4(&deltaFilter[axis], (float)delta, pidProfile->dterm_lpf, dT));
        }
        DTerm = (delta * pidProfile->D8[axis] * PIDweight[axis] / 100) >> 8;
        DTerm = constrain(DTerm, -PID_MAX_D, PID_MAX_D);
    }

#ifdef BLACKBOX
    axisPID_P[axis] = PTerm;
    axisPID_I[axis] = ITerm;
    axisPID_D[axis] = DTerm;
#endif
    GET_PID_MULTI_WII_REWRITE_CORE_LOCALS(axis);
    // -----calculate total PID output
    return PTerm + ITerm + DTerm;
}

void pidMultiWiiRewrite(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    int8_t horizonLevelStrength = 0;
    if (FLIGHT_MODE(HORIZON_MODE)) {
        // Figure out the most deflected stick position
        const int32_t stickPosAil = ABS(getRcStickDeflection(ROLL, rxConfig->midrc));
        const int32_t stickPosEle = ABS(getRcStickDeflection(PITCH, rxConfig->midrc));
        const int32_t mostDeflectedPos =  MAX(stickPosAil, stickPosEle);

        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (500 - mostDeflectedPos) / 5;  // 100 at centre stick, 0 = max stick deflection

        // Using D8[PIDLEVEL] as a Sensitivity for Horizon.
        // 0 more level to 255 more rate. Default value of 100 seems to work fine.
        // For more rate mode increase D and slower flips and rolls will be possible
        horizonLevelStrength = constrain((10 * (horizonLevelStrength - 100) * (10 * pidProfile->D8[PIDLEVEL] / 80) / 100) + 100, 0, 100);
    }

    // ----------PID controller----------
    for (int axis = 0; axis < 3; axis++) {
        const uint8_t rate = controlRateConfig->rates[axis];

        // -----Get the desired angle rate depending on flight mode
        int32_t angleRate;
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            angleRate = (((int32_t)(rate + 27) * rcCommand[YAW]) >> 5);
        } else {
            // control is GYRO based for ACRO and HORIZON - direct sticks control is applied to rate PID
            angleRate = ((int32_t)(rate + 27) * rcCommand[axis]) >> 4;
            if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
                // calculate error angle and limit the angle to the max inclination
                // multiplication of rcCommand corresponds to changing the sticks scaling here
#ifdef GPS
                const int32_t errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int)max_angle_inclination), max_angle_inclination)
                        - attitude.raw[axis] + angleTrim->raw[axis];
#else
                const int32_t errorAngle = constrain(2 * rcCommand[axis], -((int)max_angle_inclination), max_angle_inclination)
                        - attitude.raw[axis] + angleTrim->raw[axis];
#endif
                if (FLIGHT_MODE(ANGLE_MODE)) {
                    // ANGLE mode
                    angleRate = (errorAngle * pidProfile->P8[PIDLEVEL]) >> 4;
                } else {
                    // HORIZON mode
                    // mix in errorAngle to desired angleRate to add a little auto-level feel.
                    // horizonLevelStrength has been scaled to the stick input
                    angleRate += (errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 100) >> 4;
                }
            }
        }

        // --------low-level gyro-based PID. ----------
        const int32_t gyroRate = gyroADC[axis] / 4;
        axisPID[axis] = pidMultiWiiRewriteCore(axis, pidProfile, gyroRate, angleRate);

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
             calculate_Gtune(axis);
        }
#endif
    }
}


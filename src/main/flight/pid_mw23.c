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
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"

#ifndef SKIP_PID_MW23

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/config_unittest.h"

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
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"
#include "flight/mixer.h"

static int32_t ITermAngle[2];

uint8_t dynP8[3], dynI8[3], dynD8[3];

extern float dT;
extern uint8_t motorCount;

#ifdef BLACKBOX
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif
extern int32_t lastITerm[3], ITermLimit[3];

extern pt1Filter_t deltaFilter[3];


void pidResetITermAngle(void)
{
    ITermAngle[AI_ROLL] = 0;
    ITermAngle[AI_PITCH] = 0;
}

void pidMultiWii23(const pidProfile_t *pidProfile, const controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, const rollAndPitchTrims_t *angleTrim, const rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);

    int axis, prop = 0;
    int32_t rc, error, errorAngle, delta, gyroError;
    int32_t PTerm, ITerm, PTermACC, ITermACC, DTerm;
    static int16_t lastErrorForDelta[2];
    static int32_t delta1[2], delta2[2];

    if (FLIGHT_MODE(HORIZON_MODE)) {
        prop = MIN(MAX(ABS(rcCommand[PITCH]), ABS(rcCommand[ROLL])), 512);
    }

    // PITCH & ROLL
    for (axis = 0; axis < 2; axis++) {

        rc = rcCommand[axis] << 1;

        gyroError = gyroADC[axis] / 4;

        error = rc - gyroError;
        lastITerm[axis]  = constrain(lastITerm[axis] + error, -16000, +16000);   // WindUp   16 bits is ok here

        if (ABS(gyroADC[axis]) > (640 * 4)) {
            lastITerm[axis] = 0;
        }

        // Anti windup protection
        if (rcModeIsActive(BOXAIRMODE)) {
            if (STATE(ANTI_WINDUP) || motorLimitReached) {
                lastITerm[axis] = constrain(lastITerm[axis], -ITermLimit[axis], ITermLimit[axis]);
            } else {
                ITermLimit[axis] = ABS(lastITerm[axis]);
            }
        }

        ITerm = (lastITerm[axis] >> 7) * pidProfile->I8[axis] >> 6;   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000

        PTerm = (int32_t)rc * pidProfile->P8[axis] >> 6;

        if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {   // axis relying on ACC
            // 50 degrees max inclination
#ifdef GPS
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis];
#else
            errorAngle = constrain(2 * rcCommand[axis], -((int) max_angle_inclination),
                +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis];
#endif

            ITermAngle[axis]  = constrain(ITermAngle[axis] + errorAngle, -10000, +10000);                                                // WindUp     //16 bits is ok here

            PTermACC = ((int32_t)errorAngle * pidProfile->P8[PIDLEVEL]) >> 7;   // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result

            int16_t limit = pidProfile->D8[PIDLEVEL] * 5;
            PTermACC = constrain(PTermACC, -limit, +limit);

            ITermACC = ((int32_t)ITermAngle[axis] * pidProfile->I8[PIDLEVEL]) >> 12;  // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

            ITerm = ITermACC + ((ITerm - ITermACC) * prop >> 9);
            PTerm = PTermACC + ((PTerm - PTermACC) * prop >> 9);
        }

        PTerm -= ((int32_t)gyroError * dynP8[axis]) >> 6;   // 32 bits is needed for calculation

        //-----calculate D-term based on the configured approach (delta from measurement or deltafromError)
        // Delta from measurement
        delta = -(gyroError - lastErrorForDelta[axis]);
        lastErrorForDelta[axis] = gyroError;
        if (pidProfile->dterm_lpf) {
            // Dterm delta low pass
            DTerm = delta;
            DTerm = lrintf(pt1FilterApply4(&deltaFilter[axis], (float)DTerm, pidProfile->dterm_lpf, dT)) * 3;  // Keep same scaling as unfiltered DTerm
        } else {
            // When dterm filter disabled apply moving average to reduce noise
            DTerm  = delta1[axis] + delta2[axis] + delta;
            delta2[axis] = delta1[axis];
            delta1[axis] = delta;
        }
        DTerm = ((int32_t)DTerm * dynD8[axis]) >> 5;   // 32 bits is needed for calculation

        axisPID[axis] = PTerm + ITerm + DTerm;

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

    //YAW
    rc = (int32_t)rcCommand[YAW] * (2 * controlRateConfig->rates[YAW] + 30)  >> 5;
#ifdef ALIENWFLIGHT
    error = rc - gyroADC[FD_YAW];
#else
    error = rc - (gyroADC[FD_YAW] / 4);
#endif
    lastITerm[FD_YAW]  += (int32_t)error * pidProfile->I8[FD_YAW];
    lastITerm[FD_YAW]  = constrain(lastITerm[FD_YAW], 2 - ((int32_t)1 << 28), -2 + ((int32_t)1 << 28));
    if (ABS(rc) > 50) lastITerm[FD_YAW] = 0;

    PTerm = (int32_t)error * pidProfile->P8[FD_YAW] >> 6; // TODO: Bitwise shift on a signed integer is not recommended

    // Constrain YAW by D value if not servo driven in that case servolimits apply
    if(motorCount >= 4 && pidProfile->yaw_p_limit < YAW_P_LIMIT_MAX) {
        PTerm = constrain(PTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
    }

    ITerm = constrain((int16_t)(lastITerm[FD_YAW] >> 13), -GYRO_I_MAX, +GYRO_I_MAX);

    axisPID[FD_YAW] =  PTerm + ITerm;

#ifdef GTUNE
    if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
        calculate_Gtune(FD_YAW);
    }
#endif

#ifdef BLACKBOX
    axisPID_P[FD_YAW] = PTerm;
    axisPID_I[FD_YAW] = ITerm;
    axisPID_D[FD_YAW] = 0;
#endif
}

#endif


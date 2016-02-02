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
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"

#include "config/runtime_config.h"

extern uint16_t cycleTime;
extern uint8_t motorCount;
extern bool motorLimitReached;
extern float dT;

int16_t axisPID[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t dynP8[3], dynI8[3], dynD8[3], PIDweight[3];

static float errorGyroIf[3] = { 0.0f, 0.0f, 0.0f };
static float errorGyroIfLimit[3] = { 0.0f, 0.0f, 0.0f };

void pidResetErrorGyro(void)
{
    errorGyroIf[ROLL] = 0.0f;
    errorGyroIf[PITCH] = 0.0f;
    errorGyroIf[YAW] = 0.0f;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static biquad_t deltaBiQuadState[3];
static bool deltaFilterInit = false;

float pidRcCommandToAngle(int16_t stick)
{
    return stick * 2.0f;
}

int16_t pidAngleToRcCommand(float angleDeciDegrees)
{
    return angleDeciDegrees / 2.0f;
}

float pidRcCommandToRate(int16_t stick, uint8_t rate)
{
    // Map stick position from 200dps to 1200dps
    return (float)((rate + 20) * stick) / 50.0f;
}

#define FP_PID_RATE_P_MULTIPLIER    40.0f
#define FP_PID_RATE_I_MULTIPLIER    40.0f
#define FP_PID_RATE_D_MULTIPLIER    4000.0f
#define FP_PID_LEVEL_P_MULTIPLIER   40.0f

void pidController(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);

    float horizonLevelStrength = 1;
    static float rateErrorBuf[3][5] = { { 0 } };
    int axis, n;

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
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100.0f / pidProfile->D8[PIDLEVEL])) + 1, 0, 1);
        }
    }

    for (axis = 0; axis < 3; axis++) {
        float kP = pidProfile->P8[axis] / FP_PID_RATE_P_MULTIPLIER;
        float kI = pidProfile->I8[axis] / FP_PID_RATE_I_MULTIPLIER;
        float kD = pidProfile->D8[axis] / FP_PID_RATE_D_MULTIPLIER;
        bool useIntegralComponent = (pidProfile->P8[axis] != 0) && (pidProfile->I8[axis] != 0);

        float rateTarget;   // rotation rate target (dps)

        // Rate setpoint for ACRO and HORIZON modes
        rateTarget = pidRcCommandToRate(rcCommand[axis], controlRateConfig->rates[axis]);

        // Outer PID loop (ANGLE/HORIZON)
        if ((axis != FD_YAW) && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
            static filterStatePt1_t angleFilterState[2];    // Only ROLL and PITCH

            float angleTarget = pidRcCommandToAngle(rcCommand[axis]);
            float angleError = (constrain(angleTarget, -pidProfile->max_angle_inclination, +pidProfile->max_angle_inclination) - attitude.raw[axis]) / 10.0f;

            // P[LEVEL] defines self-leveling strength (both for ANGLE and HORIZON modes)
            if (FLIGHT_MODE(HORIZON_MODE)) {
                rateTarget += angleError * (pidProfile->P8[PIDLEVEL] / FP_PID_LEVEL_P_MULTIPLIER) * horizonLevelStrength;
            }
            else {
                rateTarget = angleError * (pidProfile->P8[PIDLEVEL] / FP_PID_LEVEL_P_MULTIPLIER);
            }

            // Apply simple LPF to rateTarget to make response less jerky
            if (pidProfile->I8[PIDLEVEL]) {
                // I8[PIDLEVEL] is filter cutoff frequency (Hz). Practical values of filtering frequency is 5-10 Hz
                rateTarget = filterApplyPt1(rateTarget, &angleFilterState[axis], pidProfile->I8[PIDLEVEL], dT);
            }
        }

        // Limit desired rate to something gyro can measure reliably
        rateTarget = constrainf(rateTarget, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);

        // Inner PID loop - gyro-based control to reach rateTarget
        float gyroRate = gyroADC[axis] * gyro.scale; // gyro output scaled to dps
        float rateError = rateTarget - gyroRate;

        // Calculate new P-term
        float newPTerm = rateError * kP * (PIDweight[axis] / 100);

        if((motorCount >= 4 && pidProfile->yaw_p_limit) && axis == YAW) {
            newPTerm = constrain(newPTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
        }

        // Calculate new D-term
        // Shift old error values
        for (n = 4; n > 0; n--) {
            rateErrorBuf[axis][n] = rateErrorBuf[axis][n-1];
        }

        // Store new error value
        rateErrorBuf[axis][0] = rateError;

        // Calculate derivative using 5-point noise-robust differentiator by Pavel Holoborodko
        float newDTerm = ((2 * (rateErrorBuf[axis][1] - rateErrorBuf[axis][3]) + (rateErrorBuf[axis][0] - rateErrorBuf[axis][4])) / (8 * dT)) * kD * (PIDweight[axis] / 100);

        // Apply additional lowpass
        if (pidProfile->dterm_lpf_hz) {
            if (!deltaFilterInit) {
                for (axis = 0; axis < 3; axis++)
                    filterInitBiQuad(pidProfile->dterm_lpf_hz, &deltaBiQuadState[axis], 0);
                deltaFilterInit = true;
            }

            newDTerm = filterApplyBiQuad(newDTerm, &deltaBiQuadState[axis]);
        }

        // TODO: Get feedback from mixer on available correction range for each axis
        float newOutput = newPTerm + errorGyroIf[axis] + newDTerm;
        float newOutputLimited = constrainf(newOutput, -PID_MAX_OUTPUT, +PID_MAX_OUTPUT);

        // Integrate only if we can do backtracking
        if (useIntegralComponent) {
            float kT = 2.0f / ((kP / kI) + (kD / kP));
            errorGyroIf[axis] += (rateError * kI * dT) + ((newOutputLimited - newOutput) * kT * dT);

            // Don't grow I-term if motors are at their limit
            if (STATE(ANTI_WINDUP) || motorLimitReached) {
                errorGyroIf[axis] = constrainf(errorGyroIf[axis], -errorGyroIfLimit[axis], errorGyroIfLimit[axis]);
            } else {
                errorGyroIfLimit[axis] = ABS(errorGyroIf[axis]);
            }
        } else {
            errorGyroIf[axis] = 0;
        }

        axisPID[axis] = newOutputLimited;

        debug[axis] = newOutputLimited;

#ifdef BLACKBOX
        axisPID_P[axis] = newPTerm;
        axisPID_I[axis] = errorGyroIf[axis];
        axisPID_D[axis] = newDTerm;
#endif
    }
}

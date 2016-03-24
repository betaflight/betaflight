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

typedef struct {
    float kP;
    float kI;
    float kD;
    float kT;

    float rateTarget;
    float rateErrorBuf[5];
    float errorGyroIf;
    float errorGyroIfLimit;

    filterStatePt1_t angleFilterState;

    biquad_t deltaBiQuadState;
    bool deltaFilterInit;
} pidState_t;

extern uint16_t cycleTime;
extern uint8_t motorCount;
extern bool motorLimitReached;
extern float dT;

int16_t axisPID[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3], axisPID_Setpoint[3];
#endif

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 1 = 100% scale means no PID reduction
float PIDweight[3];

static pidState_t pidState[3];

void pidResetErrorGyro(void)
{
    pidState[ROLL].errorGyroIf = 0.0f;
    pidState[PITCH].errorGyroIf = 0.0f;
    pidState[YAW].errorGyroIf = 0.0f;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

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
#define FP_PID_RATE_I_MULTIPLIER    10.0f
#define FP_PID_RATE_D_MULTIPLIER    4000.0f
#define FP_PID_LEVEL_P_MULTIPLIER   40.0f

static void pidOuterLoop(pidProfile_t *pidProfile, rxConfig_t *rxConfig)
{
    int axis;
    float horizonLevelStrength = 1;

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

    // Set rateTarget for axis
    for (axis = 0; axis < 3; axis++) {
        float rateTarget = pidState[axis].rateTarget;

        if (axis == FD_YAW) {
            // Axis lock implementation from OpenPilot
        }
        else {
            // This is ROLL/PITCH, run ANGLE/HORIZON controllers
            if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))) {
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
                // Ideas behind this:
                //  1) Attitude is updated at gyro rate, rateTarget for ANGLE mode is calculated from attitude
                //  2) If this rateTarget is passed directly into gyro-base PID controller this effectively doubles the rateError. D-term that is calculated from error
                //     tend to amplify this even more. Moreover, this tend to respond to every slightest change in attitude making self-leveling jittery
                //  3) Lowering LEVEL P can make the effects of (2) less visible, but this also slows down self-leveling.
                //  4) Human pilot response to attitude change in RATE mode is fairly slow and smooth, human pilot doesn't compensate for each slightest change
                //  5) (2) and (4) lead to a simple idea of adding a low-pass filter on rateTarget for ANGLE mode damping response to rapid attitude changes and smoothing
                //     out self-leveling reaction
                if (pidProfile->I8[PIDLEVEL]) {
                    // I8[PIDLEVEL] is filter cutoff frequency (Hz). Practical values of filtering frequency is 5-10 Hz
                    rateTarget = filterApplyPt1(rateTarget, &pidState[axis].angleFilterState, pidProfile->I8[PIDLEVEL], dT);
                }
            }
        }

        // Limit desired rate to something gyro can measure reliably
        pidState[axis].rateTarget = constrainf(rateTarget, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);
    }
}

static void pidApplyRateController(pidProfile_t *pidProfile, pidState_t *pidState, int axis, float gyroRate)
{
    int n;

    float rateError = pidState->rateTarget - gyroRate;

    // Calculate new P-term
    float newPTerm = rateError * pidState->kP;

    if((motorCount >= 4 && pidProfile->yaw_p_limit) && axis == YAW) {
        newPTerm = constrain(newPTerm, -pidProfile->yaw_p_limit, pidProfile->yaw_p_limit);
    }

    // Calculate new D-term
    // Shift old error values
    for (n = 4; n > 0; n--) {
        pidState->rateErrorBuf[n] = pidState->rateErrorBuf[n-1];
    }

    // Store new error value
    pidState->rateErrorBuf[0] = rateError;

    // Calculate derivative using 5-point noise-robust differentiator by Pavel Holoborodko
    float newDTerm = ((2 * (pidState->rateErrorBuf[1] - pidState->rateErrorBuf[3]) + (pidState->rateErrorBuf[0] - pidState->rateErrorBuf[4])) / (8 * dT)) * pidState->kD;

    // Apply additional lowpass
    if (pidProfile->dterm_lpf_hz) {
        if (!pidState->deltaFilterInit) {
            filterInitBiQuad(pidProfile->dterm_lpf_hz, &pidState->deltaBiQuadState, 0);
            pidState->deltaFilterInit = true;
        }

        newDTerm = filterApplyBiQuad(newDTerm, &pidState->deltaBiQuadState);
    }

    // TODO: Get feedback from mixer on available correction range for each axis
    float newOutput = newPTerm + pidState->errorGyroIf + newDTerm;
    float newOutputLimited = constrainf(newOutput, -PID_MAX_OUTPUT, +PID_MAX_OUTPUT);

    // Integrate only if we can do backtracking
    pidState->errorGyroIf += (rateError * pidState->kI * dT) + ((newOutputLimited - newOutput) * pidState->kT * dT);

    // Don't grow I-term if motors are at their limit
    if (STATE(ANTI_WINDUP) || motorLimitReached) {
        pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidState->errorGyroIfLimit, pidState->errorGyroIfLimit);
    } else {
        pidState->errorGyroIfLimit = ABS(pidState->errorGyroIf);
    }

    axisPID[axis] = newOutputLimited;

#ifdef BLACKBOX
    axisPID_P[axis] = newPTerm;
    axisPID_I[axis] = pidState[axis].errorGyroIf;
    axisPID_D[axis] = newDTerm;
    axisPID_Setpoint[axis] = pidState->rateTarget;
#endif
}

static void pidInnerLoop(pidProfile_t *pidProfile)
{
    int axis;

    for (axis = 0; axis < 3; axis++) {
        /* Calculate PID gains */
        pidState[axis].kP = pidProfile->P8[axis] / FP_PID_RATE_P_MULTIPLIER * PIDweight[axis];
        pidState[axis].kI = pidProfile->I8[axis] / FP_PID_RATE_I_MULTIPLIER;
        pidState[axis].kD = pidProfile->D8[axis] / FP_PID_RATE_D_MULTIPLIER * PIDweight[axis];

        if ((pidProfile->P8[axis] != 0) && (pidProfile->I8[axis] != 0)) {
            pidState[axis].kT = 2.0f / ((pidState[axis].kP / pidState[axis].kI) + (pidState[axis].kD / pidState[axis].kP));
        }
        else {
            pidState[axis].kT = 0;
        }

        /* Apply PID setpoint controller */
        pidApplyRateController(pidProfile,
                               &pidState[axis],
                               axis,
                               gyroADC[axis] * gyro.scale);     // scale gyro rate to DPS
    }
}

/* Read sticks input for each axis */
static void getRateTarget(controlRateConfig_t *controlRateConfig)
{
    uint8_t axis;

    for (axis = 0; axis < 3; axis++) {
        pidState[axis].rateTarget = constrainf(pidRcCommandToRate(rcCommand[axis], controlRateConfig->rates[axis]), -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);
    }
}

void pidController(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, rxConfig_t *rxConfig)
{
    /* Step 1: Read sticks */
    getRateTarget(controlRateConfig);

    /* 
    Step 2: Run outer loop control for ANGLE and HORIZON
    In any other case, it is not needed
    */
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        pidOuterLoop(pidProfile, rxConfig);
    }

    /* Step 2: Run gyro-driven inner loop control */
    pidInnerLoop(pidProfile);
}

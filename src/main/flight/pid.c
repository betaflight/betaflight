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

#include "drivers/sensor.h"

#include "drivers/accgyro.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "fc/rc_controls.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"

#include "fc/runtime_config.h"


uint32_t targetPidLooptime;

bool pidStabilisationEnabled;

uint8_t PIDweight[3];

int16_t axisPID[3];

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t PIDweight[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

int32_t errorGyroI[3];
float errorGyroIf[3];

#ifdef SKIP_PID_FLOAT
pidControllerFuncPtr pid_controller = pidLegacy; // which pid controller are we using
#else
pidControllerFuncPtr pid_controller = pidBetaflight; // which pid controller are we using
#endif

void setTargetPidLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
}

void pidResetErrorGyroState(void)
{
    for (int axis = 0; axis < 3; axis++) {
        errorGyroI[axis] = 0;
        errorGyroIf[axis] = 0.0f;
    }
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

float getdT(void)
{
    static float dT;
    if (!dT) dT = (float)targetPidLooptime * 0.000001f;

    return dT;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

pt1Filter_t deltaFilter[3];
pt1Filter_t yawFilter;
biquadFilter_t dtermFilterLpf[3];
biquadFilter_t dtermFilterNotch[3];
bool dtermNotchInitialised;
bool dtermBiquadLpfInitialised;

void initFilters(const pidProfile_t *pidProfile)
{
    int axis;

    if (pidProfile->dterm_notch_hz && !dtermNotchInitialised) {
        float notchQ = filterGetNotchQ(pidProfile->dterm_notch_hz, pidProfile->dterm_notch_cutoff);
        for (axis = 0; axis < 3; axis++) biquadFilterInit(&dtermFilterNotch[axis], pidProfile->dterm_notch_hz, targetPidLooptime, notchQ, FILTER_NOTCH);
        dtermNotchInitialised = true;
    }

    if (pidProfile->dterm_filter_type == FILTER_BIQUAD) {
        if (pidProfile->dterm_lpf_hz && !dtermBiquadLpfInitialised) {
            for (axis = 0; axis < 3; axis++) biquadFilterInitLPF(&dtermFilterLpf[axis], pidProfile->dterm_lpf_hz, targetPidLooptime);
            dtermBiquadLpfInitialised = true;
        }
    }
}

        } else {
            if (pidProfile->yaw_lpf_hz) PTerm = pt1FilterApply4(&yawFilter, PTerm, pidProfile->yaw_lpf_hz, getdT());

            axisPID[axis] = lrintf(PTerm + ITerm);

            DTerm = 0.0f; // needed for blackbox
        } else {
            if (pidProfile->yaw_lpf_hz) PTerm = pt1FilterApply4(&yawFilter, PTerm, pidProfile->yaw_lpf_hz, getdT());

            axisPID[axis] = PTerm + ITerm;

            if (motorCount >= 4) {
                int16_t yaw_jump_prevention_limit = constrain(YAW_JUMP_PREVENTION_LIMIT_HIGH - (pidProfile->D8[axis] << 3), YAW_JUMP_PREVENTION_LIMIT_LOW, YAW_JUMP_PREVENTION_LIMIT_HIGH);

                // prevent "yaw jump" during yaw correction
                axisPID[YAW] = constrain(axisPID[YAW], -yaw_jump_prevention_limit - ABS(rcCommand[YAW]), yaw_jump_prevention_limit + ABS(rcCommand[YAW]));
            }

            DTerm = 0; // needed for blackbox
void pidSetController(pidControllerType_e type)
{
    switch (type) {
        default:
        case PID_CONTROLLER_LEGACY:
            pid_controller = pidLegacy;
            break;
#ifndef SKIP_PID_FLOAT
        case PID_CONTROLLER_BETAFLIGHT:
            pid_controller = pidBetaflight;
#endif
    }
}

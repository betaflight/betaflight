/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_PSAS

#include <math.h>
#include <string.h>

#include "common/maths.h"

#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/psas_trimming.h"

static psas_trimming_data_t psasTrimmingData;

void psasTrimmingInit(const pidProfile_t *pidProfile)
{
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // update the gain only; keep the filter state
        pt1FilterUpdateCutoff(&psasTrimmingData.stickLowpass[axis], pt1FilterGainFromDelay(pidProfile->psas_trim_lowpass_time * 0.001f, pidRuntime.dT));
    }

    psasTrimmingData.enabledMask = pidProfile->psas_trim_enabled_mask;
    if (psasTrimmingData.enabledMask == 0) {
        psasTrimmingData.state = TRIMMING_OFF;
        memset(psasTrimmingData.output, 0, sizeof(psasTrimmingData.output));
    }
    psasTrimmingData.deadband = pidProfile->psas_trim_deadband * 0.1f;
    psasTrimmingData.rate = pidProfile->psas_trim_rate * 0.01f;
    psasTrimmingData.outputRange = pidProfile->psas_trim_limit;
    psasTrimmingData.tauReset = pidProfile->psas_trim_tau_reset * 0.001f;
}

// Apply trim to bigger stick deflection
static void estimateActiveTrimAxis(void)
{
        float maxStick = -1.0f;
        for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            if ((psasTrimmingData.enabledMask & (1 << axis)) == 0) {
                continue;
            }
            const float absStick = fabsf(psasTrimmingData.stickFiltered[axis]);
            if (absStick > maxStick) {
                maxStick = absStick;
                psasTrimmingData.activeAxis = axis;
            }
        }
}

void setPsasTrimmingState(uint8_t state)
{
    if (psasTrimmingData.enabledMask != 0) {
        psasTrimmingData.state = state;
        // Active channel estimation after TRIMMING_ADJUSTMENT switch
        if (psasTrimmingData.state == TRIMMING_ADJUSTMENT) {
            estimateActiveTrimAxis();
        }
    }
}

void psasTrimmingUpdate(void)
{
    if (psasTrimmingData.enabledMask == 0) {
        return;
    }
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        psasTrimmingData.stickFiltered[axis] = pt1FilterApply(&psasTrimmingData.stickLowpass[axis], getRcDeflection(axis) * 100.0f);
    }

    if (psasTrimmingData.state == TRIMMING_FIX) {
        return;
    } else if (psasTrimmingData.state == TRIMMING_OFF) {
        for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            psasTrimmingData.output[axis] += -psasTrimmingData.output[axis] / psasTrimmingData.tauReset * pidRuntime.dT;
        }
    } else if (psasTrimmingData.state == TRIMMING_ADJUSTMENT) {
        const uint8_t activeAxis = psasTrimmingData.activeAxis;
        const float stickFiltered = psasTrimmingData.stickFiltered[activeAxis];
        if (fabsf(stickFiltered) < psasTrimmingData.deadband) {
            return;
        }
        psasTrimmingData.output[activeAxis] += stickFiltered * psasTrimmingData.rate * pidRuntime.dT;
        psasTrimmingData.output[activeAxis] = constrainf(psasTrimmingData.output[activeAxis], -psasTrimmingData.outputRange, psasTrimmingData.outputRange);
    }
}

float getPsasTrimmingOutput(uint8_t axis)
{
    return psasTrimmingData.output[axis];
}

bool isPsasTrimmingChannelEnabled(uint8_t axis)
{
    return (psasTrimmingData.enabledMask & (1 << axis)) != 0;
}

uint8_t getPsasTrimmingState(void)
{
    return psasTrimmingData.state;
}

uint8_t getPsasTrimmingActiveChannel(void)
{
    return psasTrimmingData.activeAxis;
}

#endif // USE_PSAS

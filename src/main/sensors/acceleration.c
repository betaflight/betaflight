/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/filter.h"
#include "common/utils.h"

#include "config/feature.h"

#include "sensors/acceleration_init.h"
#include "sensors/boardalignment.h"
#include "sensors/gyro.h"

#include "acceleration.h"

FAST_DATA_ZERO_INIT acc_t acc;                       // acc access functions

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC.x -= accelerationTrims->raw[X];
    acc.accADC.y -= accelerationTrims->raw[Y];
    acc.accADC.z -= accelerationTrims->raw[Z];
}

static void postProcessAccelerometer(void)
{
    static vector3_t accAdcPrev;

    for (unsigned axis = 0; axis < XYZ_AXIS_COUNT; axis++) {

        // Apply anti-alias filter for attitude task (if enabled)
        if (accelerationRuntime.accLpfCutHz) {
            acc.accADC.v[axis] = pt2FilterApply(&accelerationRuntime.accFilter[axis], acc.accADC.v[axis]);
        }

        // Calculate derivative of acc (jerk)
        acc.jerk.v[axis] = (acc.accADC.v[axis] - accAdcPrev.v[axis]) * acc.sampleRateHz;
        accAdcPrev.v[axis] = acc.accADC.v[axis];
    }

    acc.accMagnitude = vector3Norm(&acc.accADC) * acc.dev.acc_1G_rec;
    acc.jerkMagnitude = vector3Norm(&acc.jerk) * acc.dev.acc_1G_rec;
}

void accUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const int16_t val =  acc.dev.ADCRaw[axis];
        acc.accADC.v[axis] = val;
    }

    if (acc.dev.accAlign == ALIGN_CUSTOM) {
        alignSensorViaMatrix(&acc.accADC, &acc.dev.rotationMatrix);
    } else {
        alignSensorViaRotation(&acc.accADC, acc.dev.accAlign);
    }

    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    applyAccelerationTrims(accelerationRuntime.accelerationTrims);

    postProcessAccelerometer();

    acc.isAccelUpdatedAtLeastOnce = true;
}

#endif // USE_ACC

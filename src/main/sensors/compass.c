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

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"

#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "fc/runtime_config.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/compass.h"

#ifdef NAZE
#include "hardware_revision.h"
#endif

mag_t mag;                   // mag access functions

#ifdef MAG

static int16_t magADCRaw[XYZ_AXIS_COUNT];
static uint8_t magInit = 0;
static uint8_t magUpdatedAtLeastOnce = 0;

bool compassInit(const compassConfig_t *compassConfig)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    LED1_ON;
    const bool ret = mag.dev.init();
    LED1_OFF;
    if (ret) {
        const int deg = compassConfig->mag_declination / 100;
        const int min = compassConfig->mag_declination   % 100;
        mag.magneticDeclination = (deg + ((float)min * (1.0f / 60.0f))) * 10; // heading is in 0.1deg units
        magInit = 1;
    }
    return ret;
}

bool isCompassHealthy(void)
{
    return (magADC[X] != 0) && (magADC[Y] != 0) && (magADC[Z] != 0);
}

bool isCompassReady(void)
{
    return magUpdatedAtLeastOnce;
}

static sensorCalibrationState_t calState;

void compassUpdate(timeUs_t currentTimeUs, flightDynamicsTrims_t *magZero)
{
    static timeUs_t calStartedAt = 0;
    static int16_t magPrev[XYZ_AXIS_COUNT];

    if (!mag.dev.read(magADCRaw)) {
        mag.magADC[X] = 0;
        mag.magADC[Y] = 0;
        mag.magADC[Z] = 0;
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        mag.magADC[axis] = magADCRaw[axis];  // int32_t copy to work with
    }

    if (STATE(CALIBRATE_MAG)) {
        calStartedAt = currentTimeUs;

        for (int axis = 0; axis < 3; axis++) {
            magZero->raw[axis] = 0;
            magPrev[axis] = 0;
        }

        sensorCalibrationResetState(&calState);
        DISABLE_STATE(CALIBRATE_MAG);
    }

    if (magInit) {              // we apply offset only once mag calibration is done
        mag.magADC[X] -= magZero->raw[X];
        mag.magADC[Y] -= magZero->raw[Y];
        mag.magADC[Z] -= magZero->raw[Z];
    }

    if (calStartedAt != 0) {
        if ((currentTimeUs - calStartedAt) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            LED0_TOGGLE;

            float diffMag = 0;
            float avgMag = 0;

            for (int axis = 0; axis < 3; axis++) {
                diffMag += (mag.magADC[axis] - magPrev[axis]) * (mag.magADC[axis] - magPrev[axis]);
                avgMag += (mag.magADC[axis] + magPrev[axis]) * (mag.magADC[axis] + magPrev[axis]) / 4.0f;
            }

            // sqrtf(diffMag / avgMag) is a rough approximation of tangent of angle between magADC and magPrev. tan(8 deg) = 0.14
            if ((avgMag > 0.01f) && ((diffMag / avgMag) > (0.14f * 0.14f))) {
                sensorCalibrationPushSampleForOffsetCalculation(&calState, mag.magADC);

                for (int axis = 0; axis < 3; axis++) {
                    magPrev[axis] = mag.magADC[axis];
                }
            }
        } else {
            float magZerof[3];
            sensorCalibrationSolveForOffset(&calState, magZerof);

            for (int axis = 0; axis < 3; axis++) {
                magZero->raw[axis] = lrintf(magZerof[axis]);
            }

            calStartedAt = 0;
            persistentFlagSet(FLAG_MAG_CALIBRATION_DONE);
            saveConfigAndNotify();
        }
    }

    alignSensors(mag.magADC, mag.dev.magAlign);

    magUpdatedAtLeastOnce = 1;
}
#endif

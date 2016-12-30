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
#include "drivers/compass_ak8963.h"
#include "drivers/compass_ak8975.h"
#include "drivers/compass_fake.h"
#include "drivers/compass_hmc5883l.h"
#include "drivers/compass_mag3110.h"
#include "drivers/compass_ist8310.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/logging.h"
#include "drivers/system.h"

#include "fc/runtime_config.h"

#include "io/gps.h"

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

bool compassDetect(magDev_t *dev, magSensor_e magHardwareToUse)
{
    magSensor_e magHardware = MAG_NONE;
    requestedSensors[SENSOR_INDEX_MAG] = magHardwareToUse;

#ifdef USE_MAG_HMC5883
    const hmc5883Config_t *hmc5883Config = 0;

#ifdef NAZE // TODO remove this target specific define
    static const hmc5883Config_t nazeHmc5883Config_v1_v4 = {
            .intTag = IO_TAG(PB12) /* perhaps disabled? */
    };
    static const hmc5883Config_t nazeHmc5883Config_v5 = {
            .intTag = IO_TAG(MAG_INT_EXTI)
    };
    if (hardwareRevision < NAZE32_REV5) {
        hmc5883Config = &nazeHmc5883Config_v1_v4;
    } else {
        hmc5883Config = &nazeHmc5883Config_v5;
    }
#endif

#ifdef MAG_INT_EXTI
    static const hmc5883Config_t extiHmc5883Config = {
        .intTag = IO_TAG(MAG_INT_EXTI)
    };

    hmc5883Config = &extiHmc5883Config;
#endif

#endif

    dev->magAlign = ALIGN_DEFAULT;

    switch(magHardwareToUse) {
    case MAG_AUTODETECT:
    case MAG_HMC5883:
#ifdef USE_MAG_HMC5883
        if (hmc5883lDetect(dev, hmc5883Config)) {
#ifdef MAG_HMC5883_ALIGN
            dev->magAlign = MAG_HMC5883_ALIGN;
#endif
            magHardware = MAG_HMC5883;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT) {
            break;
        }

    case MAG_AK8975:
#ifdef USE_MAG_AK8975
        if (ak8975Detect(dev)) {
#ifdef MAG_AK8975_ALIGN
            dev->magAlign = MAG_AK8975_ALIGN;
#endif
            magHardware = MAG_AK8975;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT) {
            break;
        }

    case MAG_AK8963:
#ifdef USE_MAG_AK8963
        if (ak8963Detect(dev)) {
#ifdef MAG_AK8963_ALIGN
            dev->magAlign = MAG_AK8963_ALIGN;
#endif
            magHardware = MAG_AK8963;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT) {
            break;
        }

    case MAG_GPS:
#ifdef GPS
        if (gpsMagDetect(dev)) {
#ifdef MAG_GPS_ALIGN
            dev->magAlign = MAG_GPS_ALIGN;
#endif
            magHardware = MAG_GPS;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT) {
            break;
        }

    case MAG_MAG3110:
#ifdef USE_MAG_MAG3110
        if (mag3110detect(dev)) {
#ifdef MAG_MAG3110_ALIGN
            dev->magAlign = MAG_MAG3110_ALIGN;
#endif
            magHardware = MAG_MAG3110;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT) {
            break;
        }

    case MAG_IST8310:
#ifdef USE_MAG_IST8310
        if (ist8310Detect(dev)) {
#ifdef MAG_IST8310_ALIGN
            dev->magAlign = MAG_IST8310_ALIGN;
#endif
            magHardware = MAG_IST8310;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT) {
            break;
        }

    case MAG_FAKE:
#ifdef USE_FAKE_MAG
        if (fakeMagDetect(dev)) {
            magHardware = MAG_FAKE;
            break;
        }
#endif
        /* If we are asked for a specific sensor - break out, otherwise - fall through and continue */
        if (magHardwareToUse != MAG_AUTODETECT) {
            break;
        }

    case MAG_NONE:
        magHardware = MAG_NONE;
        break;
    }

    addBootlogEvent6(BOOT_EVENT_MAG_DETECTION, BOOT_EVENT_FLAGS_NONE, magHardware, 0, 0, 0);

    if (magHardware == MAG_NONE) {
        sensorsClear(SENSOR_MAG);
        return false;
    }

    detectedSensors[SENSOR_INDEX_MAG] = magHardware;
    sensorsSet(SENSOR_MAG);
    return true;
}

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
    return (mag.magADC[X] != 0) || (mag.magADC[Y] != 0) || (mag.magADC[Z] != 0);
}

bool isCompassReady(void)
{
    return magUpdatedAtLeastOnce;
}

void compassUpdate(timeUs_t currentTimeUs, flightDynamicsTrims_t *magZero)
{
    static sensorCalibrationState_t calState;
    static timeUs_t calStartedAt = 0;
    static int16_t magPrev[XYZ_AXIS_COUNT];

    // Check magZero
    if ((magZero->raw[X] == 0) && (magZero->raw[Y] == 0) && (magZero->raw[Z] == 0)) {
        DISABLE_STATE(COMPASS_CALIBRATED);
    }
    else {
        ENABLE_STATE(COMPASS_CALIBRATED);
    }

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
            saveConfigAndNotify();
        }
    }

    alignSensors(mag.magADC, mag.dev.magAlign);

    magUpdatedAtLeastOnce = 1;
}
#endif

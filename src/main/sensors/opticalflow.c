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


#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_OPTICALFLOW

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "scheduler/scheduler.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_lidarmt.h"
#ifdef USE_RANGEFINDER_UPT1
#include "drivers/rangefinder/rangefinder_upt1.h"
#endif

#include "io/beeper.h"
#include "io/serial.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/opticalflow.h"

#define OPTICALFLOW_CALIBRATION_DURATION_MS 30000
#define RATE_SCALE_RESOLUTION (1000.0f)


#define ROTATION_GYRO_LIMIT (float)200.0

// static prototypes
static void applySensorRotation(vector2_t * dst, vector2_t * src);

PG_REGISTER_WITH_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig, PG_OPTICALFLOW_CONFIG, 3);

PG_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig,
    .opticalflow_hardware = OPTICALFLOW_NONE,
    .rotation = 0,
    .flip_x = 0,
    .opticalflow_uart = SERIAL_PORT_NONE,
);

static opticalflow_t opticalflow;
static float cosRotAngle = 1.0f;
static float sinRotAngle = 0.0f;

// ======================================================================
// =================== Opticalflow Main Functions =======================
// ======================================================================
static bool opticalflowDetect(opticalflowDev_t * dev, uint8_t opticalflowHardwareToUse) {
    UNUSED(dev);

    opticalflowType_e opticalflowHardware = OPTICALFLOW_NONE;

    switch (opticalflowHardwareToUse) {
        case OPTICALFLOW_MT:
#if defined(USE_OPTICALFLOW_MT) && defined(USE_RANGEFINDER_MT)
            if (mtOpticalflowDetect(dev, rangefinderConfig()->rangefinder_hardware)) {
                opticalflowHardware = OPTICALFLOW_MT;
                rescheduleTask(TASK_OPTICALFLOW, TASK_PERIOD_MS(dev->delayMs));
            }
#endif
            break;

#if defined(USE_OPTICALFLOW_UPT1) && defined(USE_RANGEFINDER_UPT1)
        case OPTICALFLOW_UPT1:
            if (upt1OpticalflowDetect(dev)) {
                opticalflowHardware = OPTICALFLOW_UPT1;
                rescheduleTask(TASK_OPTICALFLOW, TASK_PERIOD_MS(dev->delayMs));
            }
            break;
#endif

        case OPTICALFLOW_NONE:
            opticalflowHardware = OPTICALFLOW_NONE;
            break;

        default:
            opticalflowHardware = OPTICALFLOW_NONE;
            break;
    }

    if (opticalflowHardware == OPTICALFLOW_NONE) {
        sensorsClear(SENSOR_OPTICALFLOW);
        return false;
    }

    detectedSensors[SENSOR_INDEX_OPTICALFLOW] = opticalflowHardware;
    sensorsSet(SENSOR_OPTICALFLOW);
    return true;
}

bool opticalflowInit(void) {
    if (!opticalflowDetect(&opticalflow.dev, opticalflowConfig()->opticalflow_hardware)) {
        return false;
    }

    opticalflow.dev.init(&opticalflow.dev);
    opticalflow.quality = OPTICALFLOW_NO_NEW_DATA;
    opticalflow.rawFlowRates.x = 0;
    opticalflow.rawFlowRates.y = 0;
    opticalflow.processedFlowRates.x = 0;
    opticalflow.processedFlowRates.y = 0;
    opticalflow.flowRateValid[0] = true;
    opticalflow.flowRateValid[1] = true;
    opticalflow.timeStampUs = micros();

    cosRotAngle = cosf(DEGREES_TO_RADIANS(opticalflowConfig()->rotation));
    sinRotAngle = sinf(DEGREES_TO_RADIANS(opticalflowConfig()->rotation));
    return true;
}

void opticalflowUpdate(void) {
    if (opticalflow.dev.update) {
        opticalflow.dev.update(&opticalflow.dev);
    }
}

void opticalflowProcess(void) {
    opticalflowData_t data = {0};
    uint32_t deltaTimeUs = 0;
    opticalflow.dev.read(&opticalflow.dev, &data);

    opticalflow.quality = data.quality;
    deltaTimeUs = cmp32(data.timeStampUs, opticalflow.timeStampUs);

    if (deltaTimeUs != 0) { // New data
        vector2_t raw = data.flowRate;
        vector2_t processed;
        uint8_t delayedGyroSampleIndex;

        applySensorRotation(&processed, &raw);

        // Attenuate the optical flow when body rotation is detected
        // There is a delay between a detected gyro motion and this
        // being seen in the optical flow output.
        // One gyro reading is kept per flow sample; the one closest to
        // gyroDelayUs old is the one that belongs to this flow measurement.
        static uint8_t gyroSampleIndex = 0;
        static float rollRate[MAX_GYRO_SAMPLE_DELAY];
        static float pitchRate[MAX_GYRO_SAMPLE_DELAY];
        static timeUs_t gyroSampleTimeUs[MAX_GYRO_SAMPLE_DELAY];

        gyroSampleIndex = (gyroSampleIndex + 1) % MAX_GYRO_SAMPLE_DELAY;
        rollRate[gyroSampleIndex] = (float)gyroGetFilteredDownsampled(X);
        pitchRate[gyroSampleIndex] = -(float)gyroGetFilteredDownsampled(Y);
        gyroSampleTimeUs[gyroSampleIndex] = data.timeStampUs;

        const timeDelta_t targetAgeUs = (timeDelta_t)opticalflow.dev.gyroDelayUs;
        delayedGyroSampleIndex = gyroSampleIndex;
        timeDelta_t bestAgeErrorUs = INT32_MAX;
        for (unsigned i = 0; i < MAX_GYRO_SAMPLE_DELAY; i++) {
            if (gyroSampleTimeUs[i] == 0) {
                continue;   // slot not filled yet
            }
            const timeDelta_t ageUs = cmpTimeUs(data.timeStampUs, gyroSampleTimeUs[i]);
            const timeDelta_t errorUs = ABS(ageUs - targetAgeUs);
            if (errorUs < bestAgeErrorUs) {
                bestAgeErrorUs = errorUs;
                delayedGyroSampleIndex = i;
            }
        }
        // The gyro rates are already in the craft frame, and applySensorRotation()
        // has just put the flow in that frame too, so the rotation term is
        // subtracted as it stands. Rotating it as well would apply the sensor
        // mounting rotation twice: at opticalflow_rotation = 180 that inverts the
        // term and the compensation doubles the rotation error instead of removing it.
        const vector2_t delayedGyro = {{
            rollRate[delayedGyroSampleIndex],
            pitchRate[delayedGyroSampleIndex]
        }};

        DEBUG_SET(DEBUG_OPTICALFLOW, 0, lrintf(processed.x * 1000));
        DEBUG_SET(DEBUG_OPTICALFLOW, 1, lrintf(processed.y * 1000));
        DEBUG_SET(DEBUG_OPTICALFLOW, 2, lrintf(DEGREES_TO_RADIANS(delayedGyro.x) * 1000));
        DEBUG_SET(DEBUG_OPTICALFLOW, 3, lrintf(DEGREES_TO_RADIANS(delayedGyro.y) * 1000));

        // Subtract the rate of body rotation (converted from dps to rad/s) from the
        // optical flow
        processed.x -= DEGREES_TO_RADIANS(delayedGyro.x);
        processed.y -= DEGREES_TO_RADIANS(delayedGyro.y);

        // Beyond this rate of body rotation the compensation cannot recover a
        // translation rate: the rotation term is larger than the signal and its
        // timing error alone exceeds it. Mark the axis unusable rather than
        // reporting zero, which a consumer would read as "not moving" and fuse.
        const bool xValid = fabsf(delayedGyro.x) <= ROTATION_GYRO_LIMIT;
        const bool yValid = fabsf(delayedGyro.y) <= ROTATION_GYRO_LIMIT;

        DEBUG_SET(DEBUG_OPTICALFLOW, 4, xValid ? lrintf(processed.x * 1000) : 0);
        DEBUG_SET(DEBUG_OPTICALFLOW, 5, yValid ? lrintf(processed.y * 1000) : 0);

        // For the same reason, an unusable axis holds its previous value rather than
        // being zeroed. The validity flag travels with it, so a consumer can still
        // skip the sample outright, which is what the estimator does.
        processed.x = xValid ? processed.x : opticalflow.processedFlowRates.x;
        processed.y = yValid ? processed.y : opticalflow.processedFlowRates.y;

        DEBUG_SET(DEBUG_OPTICALFLOW, 6, lrintf(processed.x * 1000));
        DEBUG_SET(DEBUG_OPTICALFLOW, 7, lrintf(processed.y * 1000));

        opticalflow.rawFlowRates = raw;
        opticalflow.processedFlowRates = processed;
        opticalflow.flowRateValid[0] = xValid;
        opticalflow.flowRateValid[1] = yValid;
        opticalflow.timeStampUs  = data.timeStampUs;
    }
}

static void applySensorRotation(vector2_t * dst, vector2_t * src) {
    dst->x = (opticalflowConfig()->flip_x ? -1.0f : 1.0f) * (src->x * cosRotAngle - src->y * sinRotAngle);
    dst->y = src->x * sinRotAngle + src->y * cosRotAngle;
}

const opticalflow_t * getOpticalFlowData(void) {
    return &opticalflow;
}

bool isOpticalflowHealthy(void) {
    return cmp32(micros(), opticalflow.timeStampUs) < OPTICALFLOW_HARDWARE_TIMEOUT_US;
}
#endif // USE_OPTICALFLOW

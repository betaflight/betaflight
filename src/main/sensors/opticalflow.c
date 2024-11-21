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
#include "common/filter.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/runtime_config.h"

#include "scheduler/scheduler.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/time.h"
#include "drivers/opticalflow/opticalflow.h"
#include "drivers/opticalflow/opticalflow_mt.h"

#include "io/beeper.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/opticalflow.h"

#define OPTICALFLOW_CALIBRATION_DURATION_MS 30000
#define RATE_SCALE_RESOLUTION (1000.0f)

// static prototypes
static void applySensorRotation(opticalflowRates_t * dist, opticalflowRates_t * src);
static void applyLPF(opticalflowRates_t * flowRates);

PG_REGISTER_WITH_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig, PG_OPTICALFLOW_CONFIG, 0);

PG_RESET_TEMPLATE(opticalflowConfig_t, opticalflowConfig,
    .opticalflow_hardware = OPTICALFLOW_NONE,
    .rotation = 0,
    .flow_lpf = 0,
    .flipX = 0,
    .flipY = 0
);

static opticalflow_t opticalflow;

// ======================================================================
// =================== Opticalflow Main Functions =======================
// ======================================================================
static bool opticalflowDetect(opticalflowDev_t * dev, uint8_t opticalflowHardwareToUse) {
    UNUSED(dev);

    opticalflowType_e opticalflowHardware = OPTICALFLOW_NONE;
    requestedSensors[SENSOR_INDEX_OPTICALFLOW] = opticalflowHardwareToUse;

    switch (opticalflowHardwareToUse) {
        case OPTICALFLOW_MT:
#ifdef USE_OPTICALFLOW_MT
            if (mtOpticalflowDetect(dev)) {
                opticalflowHardware = OPTICALFLOW_MT;
                rescheduleTask(TASK_OPTICALFLOW, TASK_PERIOD_MS(dev->delayMs));
            }
#endif
            break;

        case OPTICALFLOW_NONE:
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
    opticalflow.rawFlowRates.X = 0;
    opticalflow.rawFlowRates.Y = 0;
    opticalflow.processedFlowRates.X = 0;
    opticalflow.processedFlowRates.Y = 0;
    opticalflow.lastValidResponseTimeMs = millis();

    return true;
}

void opticalflowUpdate(void) {
    if (opticalflow.dev.update) {
        opticalflow.dev.update(&opticalflow.dev);
    }
}

void opticalflowProcess(void) {
    opticalflowData_t * data = opticalflow.dev.read(&opticalflow.dev);
    
    opticalflow.quality   = data->quality;

    if (opticalflow.quality > QUALITY_MINIMUM_THRESHOLD) {
        opticalflow.lastValidResponseTimeMs = millis();

        opticalflow.rawFlowRates = data->flowRate;
        opticalflow.deltaTimeUs  = data->deltaTimeUs;
        
        applySensorRotation(&opticalflow.processedFlowRates, &opticalflow.rawFlowRates);

        applyLPF(&opticalflow.processedFlowRates);

        // DEBUG SECTION
        DEBUG_SET(DEBUG_OPTICALFLOW, 0, opticalflow.quality);
        DEBUG_SET(DEBUG_OPTICALFLOW, 1, opticalflow.rawFlowRates.X);
        DEBUG_SET(DEBUG_OPTICALFLOW, 2, opticalflow.rawFlowRates.Y);
        DEBUG_SET(DEBUG_OPTICALFLOW, 3, opticalflow.processedFlowRates.X);
        DEBUG_SET(DEBUG_OPTICALFLOW, 4, opticalflow.processedFlowRates.Y);
        DEBUG_SET(DEBUG_OPTICALFLOW, 5, (int32_t)opticalflow.deltaTimeUs);
    }
}

static void applySensorRotation(opticalflowRates_t * dist, opticalflowRates_t * src) {
    if (opticalflowConfig()->rotation != 0) {
        float angle = DEGREES_TO_RADIANS(opticalflowConfig()->rotation);

        dist->X = (int32_t)(src->X * cosf(angle) - src->Y * sinf(angle));
        dist->Y = (int32_t)(src->X * sinf(angle) + src->Y * cosf(angle));
    } else {
        dist->X = src->X;
        dist->Y = src->Y;
    }

    if (opticalflowConfig()->flipX) {
        dist->X = -dist->X;
    }
    
    if (opticalflowConfig()->flipY) {
        dist->Y = -dist->Y;
    }
}

static void applyLPF(opticalflowRates_t * flowRates) {
    if (opticalflowConfig()->flow_lpf == 0) {
        return;
    }
    static bool firstRun = true;
    static pt2Filter_t xFlowLpf, yFlowLpf;

    if (firstRun) {   
        const float flowCutoffHz = (float)opticalflowConfig()->flow_lpf / 100.0f;
        const float flowGain     = pt2FilterGain(flowCutoffHz, opticalflow.dev.delayMs / 1000.0f);

        pt2FilterInit(&xFlowLpf, flowGain);
        pt2FilterInit(&yFlowLpf, flowGain);
        firstRun = false;
    }

    flowRates->X = pt2FilterApply(&xFlowLpf, flowRates->X);
    flowRates->Y = pt2FilterApply(&yFlowLpf, flowRates->Y);
}

opticalflow_t * getLatestFlowOpticalflowData(void) {
    return &opticalflow;
}

bool opticalflowIsHealthy(void) {
    return (millis() - opticalflow.lastValidResponseTimeMs) < OPTICALFLOW_HARDWARE_TIMEOUT_MS;
}
#endif // USE_OPTICALFLOW

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

#ifdef USE_OPTICALFLOW_MT
#include "build/build_config.h"
#include "build/debug.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder_lidarmt.h"
#include "drivers/opticalflow/opticalflow.h"
#include "drivers/opticalflow/opticalflow_mt.h"

static opticalflowData_t opticalflowSensorData = {0};
static bool hasNewData = false;

typedef struct __attribute__((packed)) {
    uint8_t quality;    // [0;255]
    int32_t motionX;
    int32_t motionY;
} mtOpticalflowDataMessage_t;

static void mtOpticalflowInit(opticalflowDev_t * dev) {
    UNUSED(dev);
}

static void mtOpticalflowUpdate(opticalflowDev_t * dev) {
    UNUSED(dev);
}

static opticalflowData_t * mtOpticalflowGetData(opticalflowDev_t * dev) {
    UNUSED(dev);
    if (hasNewData) {
        hasNewData = false;
    } else {
        opticalflowSensorData.quality = OPTICALFLOW_NO_NEW_DATA;
    }
    return &opticalflowSensorData;
}

bool mtOpticalflowDetect(opticalflowDev_t * dev) {
    
    if(!isMTRangefinderDetected()) {
        return false;
    }

    dev->delayMs = getMTDeviceConf()->delayMs;
    dev->minRangeCm = DT_OPTICALFLOW_MIN_RANGE;

    dev->init   = &mtOpticalflowInit;
    dev->update = &mtOpticalflowUpdate;
    dev->read   = &mtOpticalflowGetData;
    
    return true;
}

void mtOpticalflowReceiveNewData(const uint8_t * bufferPtr) {
    static uint32_t prevOpticalflowTime = 0;
    mtRangefinderData_t * latestRangefinderData = getMTRangefinderData();
    
    const mtOpticalflowDataMessage_t * pkt = (const mtOpticalflowDataMessage_t *)bufferPtr;
    
    opticalflowSensorData.deltaTimeUs = prevOpticalflowTime == 0 ? 0 : micros() - prevOpticalflowTime; // skip the first reading dt
    opticalflowSensorData.flowRate.X  = pkt->motionX; 
    opticalflowSensorData.flowRate.Y  = pkt->motionY;
    opticalflowSensorData.quality     = (int16_t)pkt->quality * 100 / 255;
    
    if (latestRangefinderData->distanceMm < DT_OPTICALFLOW_MIN_RANGE) {
        opticalflowSensorData.quality = OPTICALFLOW_OUT_OF_RANGE;
    } else if ((millis() - latestRangefinderData->timestamp) > 500) {
        opticalflowSensorData.quality = OPTICALFLOW_HARDWARE_FAILURE;
    }

    prevOpticalflowTime = micros();
    hasNewData = true;
}
#endif // USE_OPTICALFLOW_MT

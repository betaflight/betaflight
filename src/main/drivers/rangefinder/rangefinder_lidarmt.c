/*
 * This file is part of Betaflight and INAV
 *
 * Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This is a bridge driver between a sensor reader that operates at high level (i.e. on top of UART)
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_MT

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "drivers/rangefinder/rangefinder_lidarmt.h"
#include "sensors/rangefinder.h"

#include "drivers/opticalflow/opticalflow.h"

#define MT_OPTICALFLOW_MIN_RANGE 80  // mm
#define MT_OPFLOW_MIN_QUALITY_THRESHOLD 30

static opticalflowData_t opticalflowSensorData = {0};

static bool hasRFNewData = false;
static mtRangefinderData_t rfSensorData = {RANGEFINDER_NO_NEW_DATA, 0};
static const MTRangefinderConfig * deviceConf = NULL;

// Initialize the table with values for each rangefinder type
static const MTRangefinderConfig rangefinderConfigs[] = {
    { .deviceType = RANGEFINDER_MTF01,  .delayMs = 10, .maxRangeCm = 800  },
    { .deviceType = RANGEFINDER_MTF02,  .delayMs = 20, .maxRangeCm = 250  },
    { .deviceType = RANGEFINDER_MTF01P, .delayMs = 10, .maxRangeCm = 1200 },
    { .deviceType = RANGEFINDER_MTF02P, .delayMs = 20, .maxRangeCm = 600  },
};

typedef struct __attribute__((packed)) {
    uint8_t quality;    // [0;255]
    int32_t distanceMm; // Negative value for out of range
} mspSensorRangefinderLidarMtDataMessage_t;

static void mtRangefinderInit(rangefinderDev_t * dev) {
    UNUSED(dev);
}

static void mtRangefinderUpdate(rangefinderDev_t * dev) {
    UNUSED(dev);
}

static int32_t mtRangefinderGetDistance(rangefinderDev_t * dev) {
    UNUSED(dev);
    if (hasRFNewData) {
        hasRFNewData = false;
        return (rfSensorData.distanceMm >= 0) ? (rfSensorData.distanceMm / 10) : RANGEFINDER_OUT_OF_RANGE;
    } else {
        return RANGEFINDER_NO_NEW_DATA;
    }
}

bool mtRangefinderDetect(rangefinderDev_t * dev, rangefinderType_e mtRangefinderToUse) {
    deviceConf = getMTRangefinderDeviceConf(mtRangefinderToUse);
    if (!deviceConf) {
        return false;
    }

    dev->delayMs    = deviceConf->delayMs;
    dev->maxRangeCm = deviceConf->maxRangeCm;

    dev->detectionConeDeciDegrees = RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES;

    dev->init   = &mtRangefinderInit;
    dev->update = &mtRangefinderUpdate;
    dev->read   = &mtRangefinderGetDistance;
    return true;
}

void mtRangefinderReceiveNewData(const uint8_t * bufferPtr) {
    const mspSensorRangefinderLidarMtDataMessage_t * pkt = (const mspSensorRangefinderLidarMtDataMessage_t *)bufferPtr;

    rfSensorData.distanceMm = pkt->distanceMm;
    rfSensorData.timestampUs = micros();
    hasRFNewData = true;
}

const MTRangefinderConfig* getMTRangefinderDeviceConf(rangefinderType_e mtRangefinderToUse) {
    for (const MTRangefinderConfig* cfg =  rangefinderConfigs; cfg < ARRAYEND(rangefinderConfigs); cfg++) {
        if (cfg->deviceType == mtRangefinderToUse) {
            return cfg;
        }
    }
    return NULL;
}

const mtRangefinderData_t * getMTRangefinderData(void) {
    return &rfSensorData;
}

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

static void mtOpticalflowGetData(opticalflowDev_t * dev, opticalflowData_t * result) {
    UNUSED(dev);
    *result = opticalflowSensorData;
}

bool mtOpticalflowDetect(opticalflowDev_t * dev, rangefinderType_e mtRangefinderToUse) {
    deviceConf = getMTRangefinderDeviceConf(mtRangefinderToUse);
    if (!deviceConf) {
        return false;
    }

    dev->delayMs = deviceConf->delayMs;
    dev->minRangeCm = MT_OPTICALFLOW_MIN_RANGE;
    dev->minQualityThreshold = MT_OPFLOW_MIN_QUALITY_THRESHOLD;

    dev->init   = &mtOpticalflowInit;
    dev->update = &mtOpticalflowUpdate;
    dev->read   = &mtOpticalflowGetData;
    
    return true;
}

void mtOpticalflowReceiveNewData(const uint8_t * bufferPtr) {
    const mtRangefinderData_t * latestRangefinderData = getMTRangefinderData();
    
    const mtOpticalflowDataMessage_t * pkt = (const mtOpticalflowDataMessage_t *)bufferPtr;
    
    opticalflowSensorData.timeStampUs = micros();
    opticalflowSensorData.flowRate.x  = (float)pkt->motionX / 1000.0f; 
    opticalflowSensorData.flowRate.y  = (float)pkt->motionY / 1000.0f;
    opticalflowSensorData.quality     = pkt->quality * 100 / 255;
    
    if (latestRangefinderData->distanceMm < MT_OPTICALFLOW_MIN_RANGE) {
        opticalflowSensorData.quality = OPTICALFLOW_OUT_OF_RANGE;
    } else if (cmp32(micros(), latestRangefinderData->timestampUs) > (5000 * deviceConf->delayMs)) {   // 5 updates missing
        opticalflowSensorData.quality = OPTICALFLOW_HARDWARE_FAILURE;
    }
}
#endif // USE_RANGEFINDER_MT

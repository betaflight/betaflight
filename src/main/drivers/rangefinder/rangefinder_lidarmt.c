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

#ifdef USE_OPTICALFLOW_MT
#include "drivers/opticalflow/opticalflow.h"

#define MT_OPTICALFLOW_MIN_RANGE 80  // mm
#define MT_OPFLOW_MIN_QUALITY_THRESHOLD 30

static opticalflowData_t opticalflowSensorData = {0};
static bool hasOpflowNewData = false;
#endif

static bool hasRFNewData = false;
static mtRangefinderData_t sensorData = {RANGEFINDER_NO_NEW_DATA, 0};
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

static void doNothingRF(rangefinderDev_t * dev) {
    UNUSED(dev);
}

static void setDeviceConf(rangefinderType_e mtRangefinderToUse) {
    static bool alreadySet = false;
    if (!alreadySet) {
        deviceConf = getMTRangefinderDeviceConf(mtRangefinderToUse);
        alreadySet = true;
    }
}

static int32_t mtRangefinderGetDistance(rangefinderDev_t * dev) {
    UNUSED(dev);
    if (hasRFNewData) {
        hasRFNewData = false;
        return (sensorData.distanceMm >= 0) ? (sensorData.distanceMm / 10) : RANGEFINDER_OUT_OF_RANGE;
    } else {
        return RANGEFINDER_NO_NEW_DATA;
    }
}

bool mtRangefinderDetect(rangefinderDev_t * dev, rangefinderType_e mtRangefinderToUse) {
    setDeviceConf(mtRangefinderToUse);
    if (!deviceConf) {
        return false;
    }

    dev->delayMs    = deviceConf->delayMs;
    dev->maxRangeCm = deviceConf->maxRangeCm;

    dev->detectionConeDeciDegrees = RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES;

    dev->init   = &doNothingRF;
    dev->update = &doNothingRF;
    dev->read   = &mtRangefinderGetDistance;
    return true;
}

void mtRangefinderReceiveNewData(const uint8_t * bufferPtr) {
    const mspSensorRangefinderLidarMtDataMessage_t * pkt = (const mspSensorRangefinderLidarMtDataMessage_t *)bufferPtr;

    sensorData.distanceMm = pkt->distanceMm;
    sensorData.timestampUs = micros();
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
    return &sensorData;
}
#endif // USE_RANGEFINDER_MT

#if defined(USE_OPTICALFLOW_MT) && defined(USE_RANGEFINDER_MT)
typedef struct __attribute__((packed)) {
    uint8_t quality;    // [0;255]
    int32_t motionX;
    int32_t motionY;
} mtOpticalflowDataMessage_t;

static void doNothingOF(opticalflowDev_t * dev) {
    UNUSED(dev);
}

static bool mtOpticalflowGetData(opticalflowDev_t * dev, opticalflowData_t * result) {
    UNUSED(dev);
    if (!hasOpflowNewData) {
        return false;
    }
    *result = opticalflowSensorData;
    hasOpflowNewData = false;
    return true;
}

bool mtOpticalflowDetect(opticalflowDev_t * dev, rangefinderType_e mtRangefinderToUse) {
    setDeviceConf(mtRangefinderToUse);
    if (!deviceConf) {
        return false;
    }

    dev->delayMs = deviceConf->delayMs;
    dev->minRangeCm = MT_OPTICALFLOW_MIN_RANGE;
    dev->minQualityThreshold = MT_OPFLOW_MIN_QUALITY_THRESHOLD;

    dev->init   = &doNothingOF;
    dev->update = &doNothingOF;
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
    
    if (latestRangefinderData->distanceMm < MT_OPTICALFLOW_MIN_RANGE) {
        opticalflowSensorData.quality = OPTICALFLOW_OUT_OF_RANGE;
    } else if (cmp32(millis(), latestRangefinderData->timestampUs) > (5000 * deviceConf->delayMs)) {
        opticalflowSensorData.quality = OPTICALFLOW_HARDWARE_FAILURE;
    }

    prevOpticalflowTime = micros();
    hasOpflowNewData = true;
}
#endif // USE_OPTICALFLOW_MT

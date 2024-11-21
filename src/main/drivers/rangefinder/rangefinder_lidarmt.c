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

static bool hasNewData = false;
static bool mtRangefinderDetected = false;
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

static void mtRangefinderInit(rangefinderDev_t * dev) {
    UNUSED(dev);
}

static void mtRangefinderUpdate(rangefinderDev_t * dev) {
    UNUSED(dev);
}

static int32_t mtRangefinderGetDistance(rangefinderDev_t * dev) {
    UNUSED(dev);
    if (hasNewData) {
        hasNewData = false;
        return (sensorData.distanceMm >= 0) ? (sensorData.distanceMm / 10) : RANGEFINDER_OUT_OF_RANGE;
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

    dev->init = &mtRangefinderInit;
    dev->update = &mtRangefinderUpdate;
    dev->read = &mtRangefinderGetDistance;
    mtRangefinderDetected = true;
    return true;
}

void mtRangefinderReceiveNewData(const uint8_t * bufferPtr) {
    const mspSensorRangefinderLidarMtDataMessage_t * pkt = (const mspSensorRangefinderLidarMtDataMessage_t *)bufferPtr;

    sensorData.distanceMm = pkt->distanceMm;
    sensorData.timestamp = millis();
    hasNewData = true;
}

const MTRangefinderConfig* getMTRangefinderDeviceConf(rangefinderType_e mtRangefinderToUse) {
    for (const MTRangefinderConfig* cfg =  rangefinderConfigs; cfg < ARRAYEND(rangefinderConfigs); cfg++) {
        if (cfg->deviceType == mtRangefinderToUse) {
            return cfg;
        }
    }
    return NULL;
}

bool isMTRangefinderDetected(void) {
    return mtRangefinderDetected;
}

mtRangefinderData_t * getMTRangefinderData(void) {
    return &sensorData;
}

const MTRangefinderConfig * getMTDeviceConf(void) {
    return deviceConf;
}

#endif // USE_RANGEFINDER_MT

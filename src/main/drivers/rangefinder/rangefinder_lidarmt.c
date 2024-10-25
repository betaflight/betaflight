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
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/rangefinder/rangefinder_lidarmt.h"
#include "msp/msp_rangefinder.h"
#include "sensors/rangefinder.h"

static bool hasNewData = false;
static bool mtfConnected = false;
static int32_t sensorData = RANGEFINDER_NO_NEW_DATA;

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
        return (sensorData > 0) ? sensorData : RANGEFINDER_OUT_OF_RANGE;
    }
    else {
        return RANGEFINDER_NO_NEW_DATA;
    }
}

bool mtRangefinderDetect(rangefinderDev_t * dev, uint8_t mtRangefinderToUse) {   
    UNUSED(dev);
    switch (mtRangefinderToUse)
    {   
        case RANGEFINDER_MT01P:
            dev->delayMs = RANGEFINDER_MT01P_TASK_PERIOD_MS;
            dev->maxRangeCm = RANGEFINDER_MT01P_MAX_RANGE_CM;
            break;
        
        case RANGEFINDER_MTF01:
            dev->delayMs = RANGEFINDER_MTF01_TASK_PERIOD_MS;
            dev->maxRangeCm = RANGEFINDER_MTF01_MAX_RANGE_CM;
            break;
        
        case RANGEFINDER_MTF02:
            dev->delayMs = RANGEFINDER_MTF02_TASK_PERIOD_MS;
            dev->maxRangeCm = RANGEFINDER_MTF02_MAX_RANGE_CM;
            break;
        
        case RANGEFINDER_MTF01P:
            dev->delayMs = RANGEFINDER_MTF01P_TASK_PERIOD_MS;
            dev->maxRangeCm = RANGEFINDER_MTF01P_MAX_RANGE_CM;
            break;

        case RANGEFINDER_MTF02P:
            dev->delayMs = RANGEFINDER_MTF02P_TASK_PERIOD_MS;
            dev->maxRangeCm = RANGEFINDER_MTF02P_MAX_RANGE_CM;
            break;

        default:
            return false;
            break;
        }

    dev->detectionConeDeciDegrees = RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = RANGEFINDER_MT_DETECTION_CONE_DECIDEGREES;

    dev->init = &mtRangefinderInit;
    dev->update = &mtRangefinderUpdate;
    dev->read = &mtRangefinderGetDistance;

    return true;
}

void mtRangefinderReceiveNewData(uint8_t * bufferPtr) {   
    mtfConnected = true;
    const mspSensorRangefinderDataMessage_t * pkt = (const mspSensorRangefinderDataMessage_t *)bufferPtr;

    sensorData = pkt->distanceMm / 10;
    hasNewData = true;
}

uint16_t getMtRangefinderTaskPeriodMs(uint8_t mtRangefinderToUse) {
    switch (mtRangefinderToUse)
    {   
        case RANGEFINDER_MT01P:
            return RANGEFINDER_MT01P_TASK_PERIOD_MS;
            break;
        
        case RANGEFINDER_MTF01:
            return RANGEFINDER_MTF01_TASK_PERIOD_MS;
            break;
        
        case RANGEFINDER_MTF02:
            return RANGEFINDER_MTF02_TASK_PERIOD_MS;
            break;

        case RANGEFINDER_MTF01P:
            return RANGEFINDER_MTF01P_TASK_PERIOD_MS;
            break;
        
        case RANGEFINDER_MTF02P:
            return RANGEFINDER_MTF02P_TASK_PERIOD_MS;
            break;

        default:
            return 0;
            break;
        }
}

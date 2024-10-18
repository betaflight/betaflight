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

#ifdef USE_RANGEFINDER_MSP
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/rangefinder/rangefinder_virtual.h"

static const virtualRangefinderVTable_t * highLevelDeviceVTable = NULL;

static void virtualRangefinderInit(rangefinderDev_t * dev)
{
    UNUSED(dev);
    return highLevelDeviceVTable->init();
}

static void virtualRangefinderUpdate(rangefinderDev_t * dev)
{
    UNUSED(dev);
    return highLevelDeviceVTable->update();
}

static int32_t virtualRangefinderGetDistance(rangefinderDev_t * dev)
{
    UNUSED(dev);
    return highLevelDeviceVTable->read();
}

bool virtualRangefinderDetect(rangefinderDev_t * dev, const virtualRangefinderVTable_t * vtable)
{
    if (vtable && vtable->detect()) {
        highLevelDeviceVTable = vtable;

        dev->delayMs = RANGEFINDER_VIRTUAL_TASK_PERIOD_MS;
        dev->maxRangeCm = RANGEFINDER_VIRTUAL_MAX_RANGE_CM;
        dev->detectionConeDeciDegrees = RANGEFINDER_VIRTUAL_DETECTION_CONE_DECIDEGREES;
        dev->detectionConeExtendedDeciDegrees = RANGEFINDER_VIRTUAL_DETECTION_CONE_DECIDEGREES;

        dev->init = &virtualRangefinderInit;
        dev->update = &virtualRangefinderUpdate;
        dev->read = &virtualRangefinderGetDistance;

        return true;
    }

    return false;
}
#endif
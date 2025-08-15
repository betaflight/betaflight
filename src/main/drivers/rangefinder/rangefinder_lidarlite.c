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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RANGEFINDER_LIDARLITE)

#include "drivers/bus.h"
#include "drivers/time.h"

#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_lidarlite.h"


#define LIDARLITE_RANGEFINDER_I2C_ADDRESS 0x62

#define LIDARLITE_DELAY_MILLISECONDS 20
#define LIDARLITE_RANGE_MAX_CENTIMETERS 4000
#define LIDARLITE_DETECTION_CONE_DECIDEGREES 80 // TODO: Verify

// Registers
#define LIDARLITE_ACQ_COMMAND 0x00    // Device commands
#define LIDARLITE_STATUS 0x01
#define LIDARLITE_DISTANCE_HIGH 0x0f  // (Delay register) Distance in centimeters, upper byte
#define LIDARLITE_DISTANCE_LOW 0x10   // (Delay register) Distance in centimeters, lower byte

#define LIDARLITE_COMMAND_RESET 0x00
#define LIDARLITE_COMMAND_MEASURE 0x03
#define LIDARLITE_COMMAND_MEASURE_CORRECTION 0x04
#define LIDARLITE_BUSY_FLAG 0x01
#define LIDARLITE_SIGNAL_OVERFLOW_FLAG (0x01 << 1)

// static int num_measurements = 0;
static int32_t lastCalculatedDistance = RANGEFINDER_NO_NEW_DATA;

static void lidarLiteInit(rangefinderDev_t *rangefinderDev)
{
    extDevice_t *dev = &rangefinderDev->dev;
    busDeviceRegister(dev);
}

/**
If there's no measurement, currently in progress, read the last result and start a new measurement.
Otherwise, wait for that measurement to complete.
*/
static void lidarLiteUpdate(rangefinderDev_t *rangefinderDev)
{
    extDevice_t *dev = &rangefinderDev->dev;


    uint8_t status;
    if (!busReadRegisterBuffer(dev, LIDARLITE_STATUS, &status, 1)
        || status & LIDARLITE_BUSY_FLAG) {
        return;
    }

    uint8_t buf[2];
    if (!busReadRegisterBuffer(dev, LIDARLITE_DISTANCE_HIGH, buf, 2)) {
        return;
    }
    int32_t distance = ((int32_t)buf[0] << 8) | buf[1];

    if (distance <= 1 && (status & LIDARLITE_SIGNAL_OVERFLOW_FLAG)) {
        lastCalculatedDistance = RANGEFINDER_OUT_OF_RANGE;
    } else {
        lastCalculatedDistance = distance;
    }

    busWriteRegister(dev, LIDARLITE_ACQ_COMMAND, LIDARLITE_COMMAND_MEASURE_CORRECTION);
}

static int32_t lidarLiteRead(rangefinderDev_t *dev)
{
    UNUSED(dev);
    int32_t distance = lastCalculatedDistance;
    lastCalculatedDistance = RANGEFINDER_NO_NEW_DATA;
    return distance;
}

bool lidarLiteDetect(rangefinderDev_t *rangefinderDev, rangefinderType_e rfType)
{
    if (rfType != RANGEFINDER_LIDARLITE) {
        return false;
    }

    extDevice_t *dev = &rangefinderDev->dev;
    dev->busType_u.i2c.address = LIDARLITE_RANGEFINDER_I2C_ADDRESS;

    bool ack = busWriteRegister(dev, LIDARLITE_ACQ_COMMAND, LIDARLITE_COMMAND_RESET);
    if (!ack) {
        return false;
    }
    delay(22);

    rangefinderDev->delayMs = LIDARLITE_DELAY_MILLISECONDS;
    rangefinderDev->maxRangeCm = LIDARLITE_RANGE_MAX_CENTIMETERS;

    rangefinderDev->detectionConeDeciDegrees = LIDARLITE_DETECTION_CONE_DECIDEGREES;
    rangefinderDev->detectionConeExtendedDeciDegrees = LIDARLITE_DETECTION_CONE_DECIDEGREES;

    rangefinderDev->init = &lidarLiteInit;
    rangefinderDev->update = &lidarLiteUpdate;
    rangefinderDev->read = &lidarLiteRead;

    return true;
}
#endif

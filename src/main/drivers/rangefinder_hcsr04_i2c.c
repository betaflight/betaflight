/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_HCSR04_I2C)

#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/bus_i2c.h"

#include "drivers/rangefinder.h"
#include "drivers/rangefinder_hcsr04_i2c.h"

#include "build/debug.h"

#ifndef RANGEFINDER_HCSR04_I2C_I2C_INSTANCE
#define RANGEFINDER_HCSR04_I2C_I2C_INSTANCE I2C_DEVICE
#endif

#define HCSR04_I2C_MAX_RANGE_CM 400
#define HCSR04_I2C_DETECTION_CONE_DECIDEGREES 300
#define HCSR04_I2C_DETECTION_CONE_EXTENDED_DECIDEGREES 450

#define HCSR04_I2C_Address 0x14

#define HCSR04_I2C_REGISTRY_STATUS 0x00
#define HCSR04_I2C_REGISTRY_DISTANCE_HIGH 0x01
#define HCSR04_I2C_REGISTRY_DISTANCE_LOW 0x02

volatile int32_t hcsr04i2cMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
static bool isHcsr04i2cResponding = false;

static uint8_t hcsr04i2cReadByte(uint8_t registry) {
    uint8_t buffer;

    isHcsr04i2cResponding = i2cRead(I2C_DEVICE, HCSR04_I2C_Address, registry, 1, &buffer);
    return buffer;
}

static void hcsr04i2cInit(void) {
}

void hcsr04i2cUpdate(void) {
    uint8_t response[3];

    isHcsr04i2cResponding = i2cRead(I2C_DEVICE, HCSR04_I2C_Address, HCSR04_I2C_REGISTRY_STATUS, 3, response);
    
    if (!isHcsr04i2cResponding) {
        hcsr04i2cMeasurementCm = RANGEFINDER_HARDWARE_FAILURE;
        return;
    } 

    if (response[HCSR04_I2C_REGISTRY_STATUS] == 0) {

        hcsr04i2cMeasurementCm = 
            (int32_t)((int32_t)response[HCSR04_I2C_REGISTRY_DISTANCE_HIGH] << 8) + 
            response[HCSR04_I2C_REGISTRY_DISTANCE_LOW];

    } else {
        /*
         * Rangefinder is reporting out-of-range situation
         */
        hcsr04i2cMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
    }
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t hcsr04i2cGetDistance(void) {
    return hcsr04i2cMeasurementCm;
}

bool hcsr04i2c0Detect(rangefinderDev_t *dev)
{
    hcsr04i2cReadByte(HCSR04_I2C_REGISTRY_STATUS);

    if (isHcsr04i2cResponding) {
        dev->delayMs = RANGEFINDER_HCSR04_i2C_TASK_PERIOD_MS;
        dev->maxRangeCm = HCSR04_I2C_MAX_RANGE_CM;
        dev->detectionConeDeciDegrees = HCSR04_I2C_DETECTION_CONE_DECIDEGREES;
        dev->detectionConeExtendedDeciDegrees = HCSR04_I2C_DETECTION_CONE_EXTENDED_DECIDEGREES;

        dev->init = &hcsr04i2cInit;
        dev->update = &hcsr04i2cUpdate;
        dev->read = &hcsr04i2cGetDistance;

        return true;
    } else {
        return false;
    }
}
#endif

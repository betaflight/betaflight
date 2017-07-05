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
#define RANGEFINDER_HCSR04_I2C_I2C_INSTANCE I2CDEV_1
#endif

#define HCSR04_I2C_MAX_RANGE_CM 400
#define HCSR04_I2C_DETECTION_CONE_DECIDEGREES 300
#define HCSR04_I2C_DETECTION_CONE_EXTENDED_DECIDEGREES 450

#define HCSR04_I2C_Address 0x14

volatile int32_t hcsr04i2cMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
// static bool isSensorResponding = true;

static void hcsr04i2c_init(void)
{
    // timeOfLastMeasurementMs = millis();
}

void hcsr04i2c_update(void)
{
    uint8_t buf;

    i2cRead(I2C_DEVICE, HCSR04_I2C_Address, 0, 1, &buf);
    debug[0] = buf;
    i2cRead(I2C_DEVICE, HCSR04_I2C_Address, 1, 1, &buf);
    debug[1] = buf;
    i2cRead(I2C_DEVICE, HCSR04_I2C_Address, 2, 1, &buf);
    debug[2] = buf;

    debug[3] = millis();
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
static int32_t hcsr04i2c_get_distance(void)
{
    // uint8_t buf[3];

    // bool isSensorResponding = i2cRead(RANGEFINDER_HCSR04_I2C_I2C_INSTANCE, HCSR04_I2C_AddressI2C, 0, 3, &byte);

    // debug[0] = buf[0];
    // debug[1] = buf[1];
    // debug[2] = buf[3];
    return 7;
    
}

bool hcsr04i2c0Detect(rangefinderDev_t *dev)
{
    dev->delayMs = RANGEFINDER_HCSR04_i2C_TASK_PERIOD_MS;
    dev->maxRangeCm = HCSR04_I2C_MAX_RANGE_CM;
    dev->detectionConeDeciDegrees = HCSR04_I2C_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = HCSR04_I2C_DETECTION_CONE_EXTENDED_DECIDEGREES;

    dev->init = &hcsr04i2c_init;
    dev->update = &hcsr04i2c_update;
    dev->read = &hcsr04i2c_get_distance;

    return true;
}
#endif

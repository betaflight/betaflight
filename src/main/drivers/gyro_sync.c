/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>
#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

extern gyro_t gyro;

uint32_t targetLooptime;
static uint8_t mpuDividerDrops;

bool gyroSyncCheckUpdate(void)
{
    return gyro.isDataReady && gyro.isDataReady();
}

void gyroSetSampleRate(uint32_t looptime, uint8_t lpf, uint8_t gyroSync, uint8_t gyroSyncDenominator)
{
    if (gyroSync) {
        int gyroSamplePeriod;
        if (lpf == 0) {
            gyroSamplePeriod = 125;
        } else {
            gyroSamplePeriod = 1000;
        }
        mpuDividerDrops = gyroSyncDenominator - 1;
        targetLooptime = gyroSyncDenominator * gyroSamplePeriod;
    } else {
        mpuDividerDrops = 0;
        targetLooptime = looptime;
    }
}

uint8_t gyroMPU6xxxCalculateDivider(void)
{
    return mpuDividerDrops;
}

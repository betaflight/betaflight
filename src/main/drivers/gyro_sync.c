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

#include "platform.h"

#include "sensor.h"
#include "accgyro.h"
#include "gyro_sync.h"

static uint8_t mpuDividerDrops;

bool gyroSyncCheckUpdate(gyroDev_t *gyro)
{
    if (!gyro->intStatus)
        return false;
    return gyro->intStatus(gyro);
}

uint32_t gyroSetSampleRate(uint32_t looptime, uint8_t lpf, uint8_t gyroSync, uint8_t gyroSyncDenominator)
{
    if (gyroSync) {
        int gyroSamplePeriod;
        if (lpf == 0) {
            gyroSamplePeriod = 125;

        } else {
            gyroSamplePeriod = 1000;
        }

        mpuDividerDrops  = gyroSyncDenominator - 1;
        looptime = gyroSyncDenominator * gyroSamplePeriod;
    } else {
        mpuDividerDrops = 0;
    }
    return looptime;
}

uint8_t gyroMPU6xxxCalculateDivider(void)
{
    return mpuDividerDrops;
}

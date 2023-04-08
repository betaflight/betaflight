/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>
#endif

#ifdef USE_VIRTUAL_GYRO

#include "build/build_config.h"

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_virtual.h"

static int16_t virtualGyroADC[XYZ_AXIS_COUNT];
gyroDev_t *virtualGyroDev;

static void virtualGyroInit(gyroDev_t *gyro)
{
    virtualGyroDev = gyro;
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&gyro->lock, NULL) != 0) {
        printf("Create gyro lock error!\n");
    }
#endif
}

void virtualGyroSet(gyroDev_t *gyro, int16_t x, int16_t y, int16_t z)
{
    gyroDevLock(gyro);

    virtualGyroADC[X] = x;
    virtualGyroADC[Y] = y;
    virtualGyroADC[Z] = z;

    gyro->dataReady = true;

    gyroDevUnLock(gyro);
}

STATIC_UNIT_TESTED bool virtualGyroRead(gyroDev_t *gyro)
{
    gyroDevLock(gyro);
    if (gyro->dataReady == false) {
        gyroDevUnLock(gyro);
        return false;
    }
    gyro->dataReady = false;

    gyro->gyroADCRaw[X] = virtualGyroADC[X];
    gyro->gyroADCRaw[Y] = virtualGyroADC[Y];
    gyro->gyroADCRaw[Z] = virtualGyroADC[Z];

    gyroDevUnLock(gyro);
    return true;
}

static bool virtualGyroReadTemperature(gyroDev_t *gyro, int16_t *temperatureData)
{
    UNUSED(gyro);
    UNUSED(temperatureData);
    return true;
}

bool virtualGyroDetect(gyroDev_t *gyro)
{
    gyro->initFn = virtualGyroInit;
    gyro->readFn = virtualGyroRead;
    gyro->temperatureFn = virtualGyroReadTemperature;
#if defined(SIMULATOR_BUILD)
    gyro->scale = GYRO_SCALE_2000DPS;
#else
    gyro->scale = 1.0f;
#endif
    return true;
}
#endif // USE_VIRTUAL_GYRO


#ifdef USE_VIRTUAL_ACC

static int16_t virtualAccData[XYZ_AXIS_COUNT];
accDev_t *virtualAccDev;

static void virtualAccInit(accDev_t *acc)
{
    virtualAccDev = acc;
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&acc->lock, NULL) != 0) {
        printf("Create acc lock error!\n");
    }
#endif
}

void virtualAccSet(accDev_t *acc, int16_t x, int16_t y, int16_t z)
{
    accDevLock(acc);

    virtualAccData[X] = x;
    virtualAccData[Y] = y;
    virtualAccData[Z] = z;

    acc->dataReady = true;

    accDevUnLock(acc);
}

static bool virtualAccRead(accDev_t *acc)
{
    accDevLock(acc);
    if (acc->dataReady == false) {
        accDevUnLock(acc);
        return false;
    }
    acc->dataReady = false;

    acc->ADCRaw[X] = virtualAccData[X];
    acc->ADCRaw[Y] = virtualAccData[Y];
    acc->ADCRaw[Z] = virtualAccData[Z];

    accDevUnLock(acc);
    return true;
}

bool virtualAccDetect(accDev_t *acc)
{
    acc->initFn = virtualAccInit;
    acc->readFn = virtualAccRead;
    acc->revisionCode = 0;
    return true;
}
#endif // USE_VIRTUAL_ACC

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

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
#include <stdio.h>
#include <pthread.h>
#endif

#ifdef USE_FAKE_GYRO

#include "common/axis.h"
#include "common/utils.h"

#include "accgyro.h"
#include "accgyro_fake.h"

static int16_t fakeGyroADC[XYZ_AXIS_COUNT];
gyroDev_t *fakeGyroDev;

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
static pthread_mutex_t gyroLock;
#endif

static void fakeGyroInit(gyroDev_t *gyro)
{
    fakeGyroDev = gyro;
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&gyroLock, NULL) != 0) {
        printf("Create gyroLock error!\n");
    }
#endif
}

static void fakeGyroLock(gyroDev_t *gyro)
{
    UNUSED(gyro);
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_lock(&gyroLock);
#endif
}

static void fakeGyroUnLock(gyroDev_t *gyro)
{
    UNUSED(gyro);
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_unlock(&gyroLock);
#endif
}

void fakeGyroSet(gyroDev_t *gyro, int16_t x, int16_t y, int16_t z)
{
    fakeGyroLock(gyro);

    fakeGyroADC[X] = x;
    fakeGyroADC[Y] = y;
    fakeGyroADC[Z] = z;

    gyro->dataReady = true;

    fakeGyroUnLock(gyro);
}

static bool fakeGyroRead(gyroDev_t *gyro)
{
    fakeGyroLock(gyro);
    if (gyro->dataReady == false) {
        fakeGyroUnLock(gyro);
        return false;
    }
    gyro->dataReady = false;

    gyro->gyroADCRaw[X] = fakeGyroADC[X];
    gyro->gyroADCRaw[Y] = fakeGyroADC[Y];
    gyro->gyroADCRaw[Z] = fakeGyroADC[Z];

    fakeGyroUnLock(gyro);
    return true;
}

static bool fakeGyroReadTemperature(gyroDev_t *gyro, int16_t *temperatureData)
{
    UNUSED(gyro);
    UNUSED(temperatureData);
    return true;
}

static bool fakeGyroInitStatus(gyroDev_t *gyro)
{
    UNUSED(gyro);
    return true;
}

bool fakeGyroDetect(gyroDev_t *gyro)
{
    gyro->init = fakeGyroInit;
    gyro->intStatus = fakeGyroInitStatus;
    gyro->read = fakeGyroRead;
    gyro->temperature = fakeGyroReadTemperature;
#if defined(SIMULATOR_BUILD)
    gyro->scale = 1.0f / 16.4f;
#else
    gyro->scale = 1.0f;
#endif
    return true;
}
#endif // USE_FAKE_GYRO


#ifdef USE_FAKE_ACC

static int16_t fakeAccData[XYZ_AXIS_COUNT];
accDev_t *fakeAccDev;

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
static pthread_mutex_t accLock;
#endif

static void fakeAccInit(accDev_t *acc)
{
    fakeAccDev = acc;
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&accLock, NULL) != 0) {
        printf("Create accLock error!\n");
    }
#endif
}

static void fakeAccLock(accDev_t *acc)
{
    UNUSED(acc);
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_lock(&accLock);
#endif
}

static void fakeAccUnLock(accDev_t *acc)
{
    UNUSED(acc);
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    pthread_mutex_unlock(&accLock);
#endif
}

void fakeAccSet(accDev_t *acc, int16_t x, int16_t y, int16_t z)
{
    fakeAccLock(acc);

    fakeAccData[X] = x;
    fakeAccData[Y] = y;
    fakeAccData[Z] = z;

    acc->dataReady = true;

    fakeAccUnLock(acc);
}

static bool fakeAccRead(accDev_t *acc)
{
    fakeAccLock(acc);
    if (acc->dataReady == false) {
        fakeAccUnLock(acc);
        return false;
    }
    acc->dataReady = false;

    acc->ADCRaw[X] = fakeAccData[X];
    acc->ADCRaw[Y] = fakeAccData[Y];
    acc->ADCRaw[Z] = fakeAccData[Z];

    fakeAccUnLock(acc);
    return true;
}

bool fakeAccDetect(accDev_t *acc)
{
    acc->init = fakeAccInit;
    acc->read = fakeAccRead;
    acc->revisionCode = 0;
    return true;
}
#endif // USE_FAKE_ACC


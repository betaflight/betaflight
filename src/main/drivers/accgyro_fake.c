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
#define printf printf
#define sprintf sprintf

#define ACC_LOCK pthread_mutex_unlock(&accUpdateLock)
#define ACC_UNLOCK pthread_mutex_unlock(&accUpdateLock)

#define GYRO_LOCK pthread_mutex_unlock(&gyroUpdateLock)
#define GYRO_UNLOCK pthread_mutex_unlock(&gyroUpdateLock)

#else

#define ACC_LOCK
#define ACC_UNLOCK

#define GYRO_LOCK
#define GYRO_UNLOCK

#endif

#ifdef USE_FAKE_GYRO

#include "common/axis.h"
#include "common/utils.h"

#include "accgyro.h"
#include "accgyro_fake.h"

static int16_t fakeGyroADC[XYZ_AXIS_COUNT];

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
static pthread_mutex_t gyroUpdateLock;
#endif
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_GYRO_SYNC)
static bool gyroUpdated = false;
#endif

static void fakeGyroInit(gyroDev_t *gyro)
{
    UNUSED(gyro);
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&gyroUpdateLock, NULL) != 0) {
        printf("Create gyroUpdateLock error!\n");
    }
#endif
}

void fakeGyroSet(int16_t x, int16_t y, int16_t z)
{
    GYRO_LOCK;

    fakeGyroADC[X] = x;
    fakeGyroADC[Y] = y;
    fakeGyroADC[Z] = z;

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_GYRO_SYNC)
    gyroUpdated = true;
#endif
    GYRO_UNLOCK;
}

static bool fakeGyroRead(gyroDev_t *gyro)
{
    GYRO_LOCK;
#if defined(SIMULATOR_GYRO_SYNC)
    if(gyroUpdated == false) {
        GYRO_UNLOCK;
        return false;
    }
    gyroUpdated = false;
#endif

    gyro->gyroADCRaw[X] = fakeGyroADC[X];
    gyro->gyroADCRaw[Y] = fakeGyroADC[Y];
    gyro->gyroADCRaw[Z] = fakeGyroADC[Z];

    GYRO_UNLOCK;
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

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
static pthread_mutex_t accUpdateLock;
#endif
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_ACC_SYNC)
static bool accUpdated = false;
#endif

static void fakeAccInit(accDev_t *acc)
{
    UNUSED(acc);
#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_MULTITHREAD)
    if (pthread_mutex_init(&accUpdateLock, NULL) != 0) {
        printf("Create accUpdateLock error!\n");
    }
#endif
}

void fakeAccSet(int16_t x, int16_t y, int16_t z)
{
    ACC_LOCK;

    fakeAccData[X] = x;
    fakeAccData[Y] = y;
    fakeAccData[Z] = z;

#if defined(SIMULATOR_BUILD) && defined(SIMULATOR_ACC_SYNC)
    accUpdated = true;
#endif
    ACC_LOCK;
}

static bool fakeAccRead(accDev_t *acc)
{
    ACC_LOCK;
#if defined(SIMULATOR_ACC_SYNC)
    if(accUpdated == false) {
        ACC_UNLOCK;
        return false;
    }
    accUpdated = false;
#endif

    acc->ADCRaw[X] = fakeAccData[X];
    acc->ADCRaw[Y] = fakeAccData[Y];
    acc->ADCRaw[Z] = fakeAccData[Z];

    ACC_UNLOCK;
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


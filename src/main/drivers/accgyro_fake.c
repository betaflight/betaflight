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

#include "common/axis.h"
#include "common/utils.h"

#include "accgyro.h"
#include "accgyro_fake.h"


#ifdef USE_FAKE_GYRO

static int16_t fakeGyroADC[XYZ_AXIS_COUNT];

static void fakeGyroInit(uint8_t lpf)
{
    UNUSED(lpf);
}

void fakeGyroSet(int16_t x, int16_t y, int16_t z)
{
    fakeGyroADC[X] = x;
    fakeGyroADC[Y] = y;
    fakeGyroADC[Z] = z;
}

static bool fakeGyroRead(int16_t *gyroADC)
{
    gyroADC[X] = fakeGyroADC[X];
    gyroADC[Y] = fakeGyroADC[Y];
    gyroADC[Z] = fakeGyroADC[Z];
    return true;
}

static bool fakeGyroReadTemp(int16_t *tempData)
{
    UNUSED(tempData);
    return true;
}

static bool fakeGyroInitStatus(void)
{
    return true;
}

bool fakeGyroDetect(gyro_t *gyro)
{
    gyro->init = fakeGyroInit;
    gyro->intStatus = fakeGyroInitStatus;
    gyro->read = fakeGyroRead;
    gyro->temperature = fakeGyroReadTemp;
    gyro->scale = 1.0f / 16.4f;
    return true;
}
#endif // USE_FAKE_GYRO


#ifdef USE_FAKE_ACC

static int16_t fakeAccData[XYZ_AXIS_COUNT];

static void fakeAccInit(acc_t *acc)
{
    UNUSED(acc);
}

void fakeAccSet(int16_t x, int16_t y, int16_t z)
{
    fakeAccData[X] = x;
    fakeAccData[Y] = y;
    fakeAccData[Z] = z;
}

static bool fakeAccRead(int16_t *accData)
{
    accData[X] = fakeAccData[X];
    accData[Y] = fakeAccData[Y];
    accData[Z] = fakeAccData[Z];
    return true;
}

bool fakeAccDetect(acc_t *acc)
{
    acc->init = fakeAccInit;
    acc->read = fakeAccRead;
    acc->revisionCode = 0;
    return true;
}
#endif // USE_FAKE_ACC


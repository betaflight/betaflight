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

#ifdef USE_FAKE_MAG

#include "build/build_config.h"

#include "common/axis.h"
#include "common/utils.h"

#include "drivers/compass/compass.h"
#include "drivers/compass/compass_fake.h"


static int16_t fakeMagData[XYZ_AXIS_COUNT];

static bool fakeMagInit(magDev_t *magDev)
{
    UNUSED(magDev);
    // initially point north
    fakeMagData[X] = 4096;
    fakeMagData[Y] = 0;
    fakeMagData[Z] = 0;
    return true;
}

void fakeMagSet(int16_t x, int16_t y, int16_t z)
{
    fakeMagData[X] = x;
    fakeMagData[Y] = y;
    fakeMagData[Z] = z;
}

static bool fakeMagRead(magDev_t *magDev)
{
    UNUSED(magDev);
    magDev->magADCRaw[X] = fakeMagData[X];
    magDev->magADCRaw[Y] = fakeMagData[Y];
    magDev->magADCRaw[Z] = fakeMagData[Z];
    return true;
}

bool fakeMagDetect(magDev_t *mag)
{
    mag->init = fakeMagInit;
    mag->read = fakeMagRead;
    return true;
}
#endif // USE_FAKE_MAG


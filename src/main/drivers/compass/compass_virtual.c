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

#ifdef USE_VIRTUAL_MAG

#include "build/build_config.h"

#include "common/axis.h"
#include "common/utils.h"

#include "compass.h"
#include "compass_virtual.h"

static int16_t virtualMagData[XYZ_AXIS_COUNT];

static bool virtualMagInit(magDev_t *mag)
{
    UNUSED(mag);

    // initially point north
    virtualMagData[X] = 4096;
    virtualMagData[Y] = 0;
    virtualMagData[Z] = 0;
    return true;
}

void virtualMagSet(int16_t x, int16_t y, int16_t z)
{
    virtualMagData[X] = x;
    virtualMagData[Y] = y;
    virtualMagData[Z] = z;
}

static bool virtualMagRead(magDev_t *mag, int16_t *magData)
{
    UNUSED(mag);

    magData[X] = virtualMagData[X];
    magData[Y] = virtualMagData[Y];
    magData[Z] = virtualMagData[Z];
    return true;
}

bool virtualMagDetect(magDev_t *mag)
{
    mag->init = virtualMagInit;
    mag->read = virtualMagRead;
    return true;
}
#endif // USE_VIRTUAL_MAG


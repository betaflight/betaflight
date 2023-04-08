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

#ifdef USE_VIRTUAL_BARO

#include "common/utils.h"

#include "barometer.h"
#include "barometer_virtual.h"


static int32_t virtualPressure;
static int32_t virtualTemperature;


static void virtualBaroStart(baroDev_t *baro)
{
    UNUSED(baro);
}

static bool virtualBaroReadGet(baroDev_t *baro)
{
    UNUSED(baro);

    return true;
}

static void virtualBaroCalculate(int32_t *pressure, int32_t *temperature)
{
    if (pressure)
        *pressure = virtualPressure;
    if (temperature)
        *temperature = virtualTemperature;
}

void virtualBaroSet(int32_t pressure, int32_t temperature)
{
    virtualPressure = pressure;
    virtualTemperature = temperature;
}

bool virtualBaroDetect(baroDev_t *baro)
{
    virtualPressure = 101325;    // pressure in Pa (0m MSL)
    virtualTemperature = 2500;   // temperature in 0.01 C = 25 deg

    // these are dummy as temperature is measured as part of pressure
    baro->combined_read = true;
    baro->ut_delay = 10000;
    baro->get_ut = virtualBaroReadGet;
    baro->read_ut = virtualBaroReadGet;
    baro->start_ut = virtualBaroStart;

    // only _up part is executed, and gets both temperature and pressure
    baro->up_delay = 10000;
    baro->start_up = virtualBaroStart;
    baro->read_up = virtualBaroReadGet;
    baro->get_up = virtualBaroReadGet;
    baro->calculate = virtualBaroCalculate;

    return true;
}
#endif // USE_VIRTUAL_BARO

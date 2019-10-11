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

#ifdef USE_FAKE_BARO

#include "common/utils.h"

#include "barometer.h"
#include "barometer_fake.h"


static int32_t fakePressure;
static int32_t fakeTemperature;


static void fakeBaroStart(baroDev_t *baro)
{
    UNUSED(baro);
}

static bool fakeBaroReadGet(baroDev_t *baro)
{
    UNUSED(baro);

    return true;
}

static void fakeBaroCalculate(int32_t *pressure, int32_t *temperature)
{
    if (pressure)
        *pressure = fakePressure;
    if (temperature)
        *temperature = fakeTemperature;
}

void fakeBaroSet(int32_t pressure, int32_t temperature)
{
    fakePressure = pressure;
    fakeTemperature = temperature;
}

bool fakeBaroDetect(baroDev_t *baro)
{
    fakePressure = 101325;    // pressure in Pa (0m MSL)
    fakeTemperature = 2500;   // temperature in 0.01 C = 25 deg

    // these are dummy as temperature is measured as part of pressure
    baro->combined_read = true;
    baro->ut_delay = 10000;
    baro->get_ut = fakeBaroReadGet;
    baro->read_ut = fakeBaroReadGet;
    baro->start_ut = fakeBaroStart;

    // only _up part is executed, and gets both temperature and pressure
    baro->up_delay = 10000;
    baro->start_up = fakeBaroStart;
    baro->read_up = fakeBaroReadGet;
    baro->get_up = fakeBaroReadGet;
    baro->calculate = fakeBaroCalculate;

    return true;
}
#endif // USE_FAKE_BARO

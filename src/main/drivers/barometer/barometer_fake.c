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

#include <platform.h>

#ifdef USE_FAKE_BARO

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_fake.h"


static int32_t fakePressure;
static int32_t fakeTemperature;


static void fakeBaroStartGet(void)
{
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
    baro->ut_delay = 10000;
    baro->get_ut = fakeBaroStartGet;
    baro->start_ut = fakeBaroStartGet;

    // only _up part is executed, and gets both temperature and pressure
    baro->up_delay = 10000;
    baro->start_up = fakeBaroStartGet;
    baro->get_up = fakeBaroStartGet;
    baro->calculate = fakeBaroCalculate;

    return true;
}
#endif // USE_FAKE_BARO


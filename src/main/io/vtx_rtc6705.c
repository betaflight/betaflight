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
#include "build/debug.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/vtx_rtc6705.h"

#include "io/vtx.h"

void vtxCycleChannel(void)
{
    vtxState.channel++;
    if (vtxState.channel >= RTC6705_CHANNEL_COUNT) {
        vtxState.channel = 0;
    }
    rtc6705SetChannel(vtxState.band, vtxState.channel);
}

void vtxCycleBand(void)
{
    vtxState.band++;
    if (vtxState.band >= RTC6705_BAND_COUNT) {
        vtxState.band = 0;
    }
    rtc6705SetChannel(vtxState.band, vtxState.channel);
}

void vtxCycleRFPower(void)
{
    vtxState.rfPower++;
    if (vtxState.rfPower >= RTC6705_RF_POWER_COUNT) {
        vtxState.rfPower = 0;
    }
    rtc6705SetRFPower(vtxState.rfPower);
}

void vtxEnable(void)
{
    if (vtxState.enabled) {
        return;
    }
    vtxState.enabled = true;

    rtc6705Enable();

    delay(RTC6705_BOOT_DELAY);

    rtc6705SetRFPower(vtxState.rfPower);
    rtc6705SetChannel(vtxState.band, vtxState.channel);
}

void vtxDisable(void)
{
    if (!vtxState.enabled) {
        return;
    }

    vtxState.enabled = false;

    rtc6705Disable();

}

void vtxIOInit(void)
{
    rtc6705IOInit();
}

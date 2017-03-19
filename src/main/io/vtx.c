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


// Get target build configuration
#include "platform.h"

#include "common/maths.h"

#include "config/config_eeprom.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/vtx_rtc6705.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/osd.h"
#include "io/vtx.h"

#if defined(USE_RTC6705) || defined(VTX)

PG_REGISTER_WITH_RESET_TEMPLATE(vtxConfig_t, vtxConfig, PG_VTX_CONFIG, 0);

PG_RESET_TEMPLATE(vtxConfig_t, vtxConfig,
    .vtx_band = 4,    //Fatshark/Airwaves
    .vtx_channel = 1, //CH1
    .vtx_mode = 0,    //CH+BAND mode
    .vtx_mhz = 5740  //F0
);

#endif

#ifdef VTX

static uint8_t locked = 0;

void vtxInit(void)
{
    rtc6705Init();
    if (vtxConfig()->vtx_mode == 0) {
        rtc6705SetChannel(vtxConfig()->vtx_band, vtxConfig()->vtx_channel);
    } else if (vtxConfig()->vtx_mode == 1) {
        rtc6705SetFreq(vtxConfig()->vtx_mhz);
    }
}

static void setChannelSaveAndNotify(uint8_t *bandOrChannel, uint8_t step, int32_t min, int32_t max)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (vtxConfig()->vtx_mode == 0 && !locked) {
        uint8_t temp = (*bandOrChannel) + step;
        temp = constrain(temp, min, max);
        *bandOrChannel = temp;

        rtc6705SetChannel(vtxConfig()->vtx_band, vtxConfig()->vtx_channel);
        writeEEPROM();
        readEEPROM();
        beeperConfirmationBeeps(temp);
    }
}

void vtxIncrementBand(void)
{
    setChannelSaveAndNotify(&(vtxConfigMutable()->vtx_band), 1, RTC6705_BAND_MIN, RTC6705_BAND_MAX);
}

void vtxDecrementBand(void)
{
    setChannelSaveAndNotify(&(vtxConfigMutable()->vtx_band), -1, RTC6705_BAND_MIN, RTC6705_BAND_MAX);
}

void vtxIncrementChannel(void)
{
    setChannelSaveAndNotify(&(vtxConfigMutable()->vtx_channel), 1, RTC6705_CHANNEL_MIN, RTC6705_CHANNEL_MAX);
}

void vtxDecrementChannel(void)
{
    setChannelSaveAndNotify(&(vtxConfigMutable()->vtx_channel), -1, RTC6705_CHANNEL_MIN, RTC6705_CHANNEL_MAX);
}

void vtxUpdateActivatedChannel(void)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (vtxConfig()->vtx_mode == 2 && !locked) {
        static uint8_t lastIndex = -1;

        for (uint8_t index = 0; index < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; index++) {
            const vtxChannelActivationCondition_t *vtxChannelActivationCondition = &vtxConfig()->vtxChannelActivationConditions[index];

            if (isRangeActive(vtxChannelActivationCondition->auxChannelIndex, &vtxChannelActivationCondition->range)
                && index != lastIndex) {
                lastIndex = index;
                rtc6705SetChannel(vtxChannelActivationCondition->band, vtxChannelActivationCondition->channel);
                break;
            }
        }
    }
}

#endif


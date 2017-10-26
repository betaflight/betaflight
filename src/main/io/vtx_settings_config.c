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

#include "platform.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "fc/config.h"

#include "io/vtx_settings_config.h"
#include "io/vtx_string.h"

#ifdef VTX_SETTINGS_CONFIG

PG_REGISTER_WITH_RESET_TEMPLATE(vtxSettingsConfig_t, vtxSettingsConfig, PG_VTX_SETTINGS_CONFIG, 0);

PG_RESET_TEMPLATE(vtxSettingsConfig_t, vtxSettingsConfig,
    .band = VTX_SETTINGS_DEFAULT_BAND,
    .channel = VTX_SETTINGS_DEFAULT_CHANNEL,
    .power = VTX_SETTINGS_DEFAULT_POWER,
    .freq = 0
);

static bool vtxChangePending = false;

static void vtxChangeSetting(bool changed)
{
    if (changed && !vtxChangePending) {
        vtxChangePending = true;
    }
}

static bool vtxSetBand(uint8_t band)
{
    if (band != vtxSettingsConfig()->band) {
        vtxSettingsConfigMutable()->band = band;
        return true;
    }
    return false;
}

static bool vtxSetChannel(uint8_t channel)
{
    if (channel != vtxSettingsConfig()->channel) {
        vtxSettingsConfigMutable()->channel = channel;
        return true;
    }
    return false;
}

static bool vtxSetFreq(uint16_t freq)
{
    if (freq && freq != vtxSettingsConfig()->freq) {
        uint8_t band;
        uint8_t channel;
        if (vtx58_Freq2Bandchan(freq, &band, &channel)) {
            vtxChangeSetting(vtxSetBand(band));
            vtxChangeSetting(vtxSetChannel(channel));
        } else {
            vtxChangeSetting(vtxSetBand(0));
        }
        vtxSettingsConfigMutable()->freq = freq;
        return true;
    }
    return false;
}

static bool vtxSetPower(uint8_t powerIndex)
{
    if (powerIndex != vtxSettingsConfig()->power) {
        vtxSettingsConfigMutable()->power = powerIndex;
        return true;
    }
    return false;
}

static void vtxCommitChanges(void)
{
    if (vtxChangePending) {
        saveConfigAndNotify();
    }
    vtxChangePending = false;
}

//Saves the given band/channel values to configuration settings.
// band:  Band value (1 to 5).
// channel:  Channel value (1 to 8).
void vtxSettingsSaveBandAndChannel(uint8_t band, uint8_t channel)
{
    vtxChangeSetting(vtxSetBand(band));
    vtxChangeSetting(vtxSetChannel(channel));
    if (band) {
        vtxChangeSetting(vtxSetFreq(vtx58_Bandchan2Freq(band, channel)));
    }
    vtxCommitChanges();
}

//Saves the given power-index value to the configuration setting.
// index:  Power-index value.
void vtxSettingsSavePowerByIndex(uint8_t index)
{
    vtxChangeSetting(vtxSetPower(index));
    vtxCommitChanges();
}

//Saves the given band/channel/power values to configuration settings.
// band:  Band value (1 to 5).
// channel:  Channel value (1 to 8).
// index:  Power-index value.
void vtxSettingsSaveBandChanAndPower(uint8_t band, uint8_t channel, uint8_t index)
{
    vtxChangeSetting(vtxSetBand(band));
    vtxChangeSetting(vtxSetChannel(channel));
    vtxChangeSetting(vtxSetPower(index));
    if (band) {
        vtxChangeSetting(vtxSetFreq(vtx58_Bandchan2Freq(band, channel)));
    }
    vtxCommitChanges();
}

//Saves the given frequency value to the configuration setting.
// freq:  Frequency value in MHz.
void vtxSettingsSaveFrequency(uint16_t freq)
{
    vtxChangeSetting(vtxSetFreq(freq));
    vtxCommitChanges();
}

//Saves the given frequency/power values to configuration settings.
// freq:  Frequency value in MHz.
// index:  Power-index value.
void vtxSettingsSaveFreqAndPower(uint16_t freq, uint8_t index)
{
    vtxChangeSetting(vtxSetFreq(freq));
    vtxChangeSetting(vtxSetPower(index));
    vtxCommitChanges();
}

#endif  //VTX_SETTINGS_CONFIG

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

#pragma once

#define VTX_SETTINGS_MIN_BAND 1
#define VTX_SETTINGS_MAX_BAND 5
#define VTX_SETTINGS_MIN_CHANNEL 1
#define VTX_SETTINGS_MAX_CHANNEL 8

#define VTX_SETTINGS_BAND_COUNT (VTX_SETTINGS_MAX_BAND - VTX_SETTINGS_MIN_BAND + 1)
#define VTX_SETTINGS_CHANNEL_COUNT (VTX_SETTINGS_MAX_CHANNEL - VTX_SETTINGS_MIN_CHANNEL + 1)

#define VTX_SETTINGS_DEFAULT_BAND 4         //Fatshark/Airwaves
#define VTX_SETTINGS_DEFAULT_CHANNEL 1      //CH1

#define VTX_SETTINGS_MAX_FREQUENCY_MHZ 5999          //max freq (in MHz) for 'vtx_freq' setting

#if defined(VTX_SMARTAUDIO) || defined(VTX_TRAMP)

#define VTX_SETTINGS_POWER_COUNT 5
#define VTX_SETTINGS_DEFAULT_POWER 1
#define VTX_SETTINGS_MIN_POWER 0
#define VTX_SETTINGS_CONFIG
#define VTX_SETTINGS_FREQCMD

#elif defined(VTX_RTC6705)

#include "io/vtx_rtc6705.h"

#define VTX_SETTINGS_POWER_COUNT VTX_RTC6705_POWER_COUNT
#define VTX_SETTINGS_DEFAULT_POWER VTX_RTC6705_DEFAULT_POWER
#define VTX_SETTINGS_MIN_POWER VTX_RTC6705_MIN_POWER
#define VTX_SETTINGS_CONFIG

#endif


#ifdef VTX_SETTINGS_CONFIG

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

typedef struct vtxSettingsConfig_s {
    uint8_t band;       // 1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t channel;    // 1-8
    uint8_t power;      // 0 = lowest
    uint16_t freq;      // sets freq in MHz if band=0
} vtxSettingsConfig_t;

PG_DECLARE(vtxSettingsConfig_t, vtxSettingsConfig);

void vtxSettingsSaveBandAndChannel(uint8_t band, uint8_t channel);
void vtxSettingsSavePowerByIndex(uint8_t index);
void vtxSettingsSaveBandChanAndPower(uint8_t band, uint8_t channel, uint8_t index);
void vtxSettingsSaveFrequency(uint16_t freq);
void vtxSettingsSaveFreqAndPower(uint16_t freq, uint8_t index);

#endif  //VTX_SETTINGS_CONFIG

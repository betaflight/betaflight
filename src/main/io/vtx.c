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

#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(VTX_COMMON)

#include "common/time.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/vtx_common.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/vtx.h"
#include "io/vtx_string.h"


PG_REGISTER_WITH_RESET_TEMPLATE(vtxSettingsConfig_t, vtxSettingsConfig, PG_VTX_SETTINGS_CONFIG, 0);

PG_RESET_TEMPLATE(vtxSettingsConfig_t, vtxSettingsConfig,
    .band = VTX_SETTINGS_DEFAULT_BAND,
    .channel = VTX_SETTINGS_DEFAULT_CHANNEL,
    .power = VTX_SETTINGS_DEFAULT_POWER,
    .freq = VTX_SETTINGS_DEFAULT_FREQ,
    .lowPowerDisarm = 0,
);

#define VTX_PARAM_CYCLE_TIME_US 100000 // 10Hz

typedef enum {
    VTX_PARAM_BANDCHAN = 0,
    VTX_PARAM_POWER,
    VTX_PARAM_CONFIRM,
    VTX_PARAM_COUNT
} vtxScheduleParams_e;

static uint8_t vtxParamScheduleCount;
static uint8_t vtxParamSchedule[VTX_PARAM_COUNT];

void vtxInit(void)
{
    uint8_t index = 0;
    vtxParamSchedule[index++] = VTX_PARAM_BANDCHAN;
    vtxParamSchedule[index++] = VTX_PARAM_POWER;
    vtxParamSchedule[index++] = VTX_PARAM_CONFIRM;

    vtxParamScheduleCount = index;

    // sync frequency in parameter group when band/channel are specified
    const uint16_t freq = vtx58_Bandchan2Freq(vtxSettingsConfig()->band, vtxSettingsConfig()->channel);
    if (vtxSettingsConfig()->band && freq != vtxSettingsConfig()->freq) {
        vtxSettingsConfigMutable()->freq = freq;
        saveConfigAndNotify();
    }
}

static bool vtxProcessBandAndChannel(void) {
    if(!ARMING_FLAG(ARMED)) {
        uint8_t vtxBand;
        uint8_t vtxChan;
        if (vtxCommonGetBandAndChannel(&vtxBand, &vtxChan)) {
            if (vtxSettingsConfig()->band != vtxBand || vtxSettingsConfig()->channel != vtxChan) {
                vtxCommonSetBandAndChannel(vtxSettingsConfig()->band, vtxSettingsConfig()->channel);
                return true;
            }
        }
    }
    return false;
}

#if defined(VTX_SETTINGS_FREQCMD)
static bool vtxProcessFrequency(void) {
    if(!ARMING_FLAG(ARMED)) {
        uint16_t vtxFreq;
        if (vtxCommonGetFrequency(&vtxFreq)) {
            if (vtxSettingsConfig()->freq != vtxFreq) {
                vtxCommonSetFrequency(vtxSettingsConfig()->freq);
                return true;
            }
        }
    }
    return false;
}
#endif

static bool vtxProcessPower(void) {
    uint8_t vtxPower;
    uint8_t newPower;
    if (vtxCommonGetPowerIndex(&vtxPower)) {
        if (!ARMING_FLAG(ARMED) && vtxSettingsConfig()->lowPowerDisarm) {
            newPower = VTX_SETTINGS_DEFAULT_POWER;
        } else {
            newPower = vtxSettingsConfig()->power;
        }
        if (vtxPower != newPower) {
            vtxCommonSetPowerByIndex(newPower);
            return true;
        }
    }
    return false;
}

static bool vtxProcessStateUpdate(void) {
    const vtxSettingsConfig_t vtxSettingsState = {
      .band = vtxSettingsConfig()->band,
      .channel = vtxSettingsConfig()->channel,
      .power = vtxSettingsConfig()->power,
      .freq = vtxSettingsConfig()->freq,
      .lowPowerDisarm = vtxSettingsConfig()->lowPowerDisarm,
    };
    vtxSettingsConfig_t vtxState = vtxSettingsState;

    if (vtxSettingsState.band) {
        vtxCommonGetBandAndChannel(&vtxState.band, &vtxState.channel);
#if defined(VTX_SETTINGS_FREQCMD)
    } else {
        vtxCommonGetFrequency(&vtxState.freq);
#endif
    }

    vtxCommonGetPowerIndex(&vtxState.power);

    return (bool) memcmp(&vtxSettingsState, &vtxState, sizeof(vtxSettingsConfig_t));
}

void vtxProcessSchedule(timeUs_t currentTimeUs)
{
    static timeUs_t lastCycleTimeUs;
    static uint8_t scheduleIndex;
    bool vtxUpdatePending = false;

    if (vtxCommonDeviceRegistered()) {
        const uint8_t currentSchedule = vtxParamSchedule[scheduleIndex];
        // Process VTX changes from the parameter group at 10Hz
        if (currentTimeUs > lastCycleTimeUs + VTX_PARAM_CYCLE_TIME_US) {
            switch (currentSchedule) {
            case VTX_PARAM_BANDCHAN:
                if (vtxSettingsConfig()->band) {
                    vtxUpdatePending = vtxProcessBandAndChannel();
#if defined(VTX_SETTINGS_FREQCMD)
                } else {
                    vtxUpdatePending = vtxProcessFrequency();
#endif
                }
                break;
            case VTX_PARAM_POWER:
                vtxUpdatePending = vtxProcessPower();
                break;
            case VTX_PARAM_CONFIRM:
                 vtxUpdatePending = vtxProcessStateUpdate();
                 break;
            default:
                break;
            }
            lastCycleTimeUs = currentTimeUs;
            scheduleIndex = (scheduleIndex + 1) % vtxParamScheduleCount;
        }
        if (!ARMING_FLAG(ARMED) || vtxUpdatePending) {
            vtxCommonProcess(currentTimeUs);
        }
    }
}

#endif

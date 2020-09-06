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

#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_VTX_COMMON)

#include "cli/cli.h"

#include "common/maths.h"
#include "common/time.h"

#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "config/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"

#include "io/vtx_control.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "vtx.h"


PG_REGISTER_WITH_RESET_FN(vtxSettingsConfig_t, vtxSettingsConfig, PG_VTX_SETTINGS_CONFIG, 0);

void pgResetFn_vtxSettingsConfig(vtxSettingsConfig_t *vtxSettingsConfig)
{
#ifdef USE_VTX_TABLE
    vtxSettingsConfig->band = 0;
    vtxSettingsConfig->channel = 0;
    vtxSettingsConfig->power = 0;
    vtxSettingsConfig->freq = 0;
#else
    vtxSettingsConfig->freq = VTX_TABLE_DEFAULT_FREQ;
    vtxSettingsConfig->band = VTX_TABLE_DEFAULT_BAND;
    vtxSettingsConfig->channel = VTX_TABLE_DEFAULT_CHANNEL;
    vtxSettingsConfig->power = VTX_TABLE_DEFAULT_POWER;
#endif
    vtxSettingsConfig->pitModeFreq = VTX_TABLE_DEFAULT_PITMODE_FREQ;
    vtxSettingsConfig->lowPowerDisarm = VTX_LOW_POWER_DISARM_OFF;
}

typedef enum {
    VTX_PARAM_POWER = 0,
    VTX_PARAM_BANDCHAN,
    VTX_PARAM_PITMODE,
    VTX_PARAM_CONFIRM,
    VTX_PARAM_COUNT
} vtxScheduleParams_e;

void vtxInit(void)
{
    bool settingsUpdated = false;

    vtxDevice_t *vtxDevice = vtxCommonDevice();

    if (!vtxDevice) {
        // If a device is not registered, we don't have any table to refer.
        // Don't manipulate settings and just return in this case.
        return;
    }

    // sync frequency in parameter group when band/channel are specified
    const uint16_t freq = vtxCommonLookupFrequency(vtxDevice, vtxSettingsConfig()->band, vtxSettingsConfig()->channel);
    if (vtxSettingsConfig()->band && freq != vtxSettingsConfig()->freq) {
        vtxSettingsConfigMutable()->freq = freq;
        settingsUpdated = true;
    }

#if defined(VTX_SETTINGS_FREQCMD)
    // constrain pit mode frequency
    if (vtxSettingsConfig()->pitModeFreq) {
        const uint16_t constrainedPitModeFreq = MAX(vtxSettingsConfig()->pitModeFreq, VTX_TABLE_MIN_USER_FREQ);
        if (constrainedPitModeFreq != vtxSettingsConfig()->pitModeFreq) {
            vtxSettingsConfigMutable()->pitModeFreq = constrainedPitModeFreq;
            settingsUpdated = true;
        }
    }
#endif

    if (settingsUpdated) {
        saveConfigAndNotify();
    }
}

STATIC_UNIT_TESTED vtxSettingsConfig_t vtxGetSettings(void)
{
    vtxSettingsConfig_t settings = {
        .band = vtxSettingsConfig()->band,
        .channel = vtxSettingsConfig()->channel,
        .power = vtxSettingsConfig()->power,
        .freq = vtxSettingsConfig()->freq,
        .pitModeFreq = vtxSettingsConfig()->pitModeFreq,
        .lowPowerDisarm = vtxSettingsConfig()->lowPowerDisarm,
    };

#if defined(VTX_SETTINGS_FREQCMD)
    if (IS_RC_MODE_ACTIVE(BOXVTXPITMODE) && settings.pitModeFreq) {
        settings.band = 0;
        settings.freq = settings.pitModeFreq;
        settings.power = VTX_TABLE_DEFAULT_POWER;
    }
#endif

    if (!ARMING_FLAG(ARMED) && !failsafeIsActive() &&
        (settings.lowPowerDisarm == VTX_LOW_POWER_DISARM_ALWAYS ||
        (settings.lowPowerDisarm == VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM && !ARMING_FLAG(WAS_EVER_ARMED)))) {
        settings.power = VTX_TABLE_DEFAULT_POWER;
    }

    return settings;
}

static bool vtxProcessBandAndChannel(vtxDevice_t *vtxDevice)
{
    if (!ARMING_FLAG(ARMED)) {
        uint8_t vtxBand;
        uint8_t vtxChan;
        if (vtxCommonGetBandAndChannel(vtxDevice, &vtxBand, &vtxChan)) {
            const vtxSettingsConfig_t settings = vtxGetSettings();
            if (vtxBand != settings.band || vtxChan != settings.channel) {
                vtxCommonSetBandAndChannel(vtxDevice, settings.band, settings.channel);
                return true;
            }
        }
    }
    return false;
}

#if defined(VTX_SETTINGS_FREQCMD)
static bool vtxProcessFrequency(vtxDevice_t *vtxDevice)
{
    if (!ARMING_FLAG(ARMED)) {
        uint16_t vtxFreq;
        if (vtxCommonGetFrequency(vtxDevice, &vtxFreq)) {
            const vtxSettingsConfig_t settings = vtxGetSettings();
            if (vtxFreq != settings.freq) {
                vtxCommonSetFrequency(vtxDevice, settings.freq);
                return true;
            }
        }
    }
    return false;
}
#endif

static bool vtxProcessPower(vtxDevice_t *vtxDevice)
{
    uint8_t vtxPower;
    if (vtxCommonGetPowerIndex(vtxDevice, &vtxPower)) {
        const vtxSettingsConfig_t settings = vtxGetSettings();
        if (vtxPower != settings.power) {
            vtxCommonSetPowerByIndex(vtxDevice, settings.power);
            return true;
        }
    }
    return false;
}

static bool vtxProcessPitMode(vtxDevice_t *vtxDevice)
{
    unsigned vtxStatus;
    if (vtxCommonGetStatus(vtxDevice, &vtxStatus)) {
        bool currPmSwitchState = IS_RC_MODE_ACTIVE(BOXVTXPITMODE);
        if (currPmSwitchState) {
            if (!ARMING_FLAG(ARMED)) {
#if defined(VTX_SETTINGS_FREQCMD)
                if (vtxSettingsConfig()->pitModeFreq) {
                    return false;
                }
#endif
                if (!(vtxStatus & VTX_STATUS_PIT_MODE)) {
                    vtxCommonSetPitMode(vtxDevice, true);

                    return true;
                }
            }
        } else {
            if (vtxStatus & VTX_STATUS_PIT_MODE) {
                vtxCommonSetPitMode(vtxDevice, false);

                return true;
            }
        }
    }

    return false;
}

static bool vtxProcessStateUpdate(vtxDevice_t *vtxDevice)
{
    const vtxSettingsConfig_t vtxSettingsState = vtxGetSettings();
    vtxSettingsConfig_t vtxState = vtxSettingsState;

    if (vtxSettingsState.band) {
        vtxCommonGetBandAndChannel(vtxDevice, &vtxState.band, &vtxState.channel);
#if defined(VTX_SETTINGS_FREQCMD)
    } else {
        vtxCommonGetFrequency(vtxDevice, &vtxState.freq);
#endif
    }

    vtxCommonGetPowerIndex(vtxDevice, &vtxState.power);

    return (bool)memcmp(&vtxSettingsState, &vtxState, sizeof(vtxSettingsConfig_t));
}

void vtxUpdate(timeUs_t currentTimeUs)
{
    static uint8_t currentSchedule = 0;

    if (cliMode) {
        return;
    }

    vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        // Check input sources for config updates
        vtxControlInputPoll();

        const uint8_t startingSchedule = currentSchedule;
        bool vtxUpdatePending = false;
        do {
            switch (currentSchedule) {
                case VTX_PARAM_POWER:
                    vtxUpdatePending = vtxProcessPower(vtxDevice);
                    break;
                case VTX_PARAM_BANDCHAN:
                    if (vtxGetSettings().band) {
                        vtxUpdatePending = vtxProcessBandAndChannel(vtxDevice);
#if defined(VTX_SETTINGS_FREQCMD)
                    } else {
                        vtxUpdatePending = vtxProcessFrequency(vtxDevice);
#endif
                    }
                    break;
                case VTX_PARAM_PITMODE:
                    vtxUpdatePending = vtxProcessPitMode(vtxDevice);
                    break;
                case VTX_PARAM_CONFIRM:
                    vtxUpdatePending = vtxProcessStateUpdate(vtxDevice);
                    break;
                default:
                    break;
            }
            currentSchedule = (currentSchedule + 1) % VTX_PARAM_COUNT;
        } while (!vtxUpdatePending && currentSchedule != startingSchedule);

        if (!ARMING_FLAG(ARMED) || vtxUpdatePending) {
            vtxCommonProcess(vtxDevice, currentTimeUs);
        }
    }
}

#endif

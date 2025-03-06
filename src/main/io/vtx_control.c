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
#include <drivers/vtx_table.h>

#include "platform.h"

#if defined(USE_VTX_CONTROL) && defined(USE_VTX_COMMON)

#include "common/maths.h"

#include "config/config_eeprom.h"

#include "drivers/buttons.h"
#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "io/spektrum_vtx_control.h"
#include "io/vtx.h"
#include "io/vtx_control.h"

#include "osd/osd.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"


PG_REGISTER_WITH_RESET_TEMPLATE(vtxConfig_t, vtxConfig, PG_VTX_CONFIG, 1);

PG_RESET_TEMPLATE(vtxConfig_t, vtxConfig,
//    .vtxChannelActivationConditions = { 0 },
    .halfDuplex = true,
    .useVTXSync = true
);

static uint8_t locked = 0;


void vtxControlInit(void)
{
    // NOTHING TO DO
}

void vtxControlInputPoll(void)
{
    // Check variuos input sources for VTX config updates
#if defined(USE_SPEKTRUM_VTX_CONTROL)
    // Get VTX updates
    spektrumVtxControl();
#endif
}

static void vtxUpdateBandAndChannel(uint8_t bandStep, uint8_t channelStep)
{
    // if (ARMING_FLAG(ARMED)) {
    //     locked = 1;
    // }

    if (!locked && vtxCommonDevice()) {
        vtxSettingsConfigMutable()->band += bandStep;
        vtxSettingsConfigMutable()->channel += channelStep;
    }
}

void vtxIncrementBand(void)
{
    vtxUpdateBandAndChannel(+1, 0);
}

void vtxDecrementBand(void)
{
    vtxUpdateBandAndChannel(-1, 0);
}

void vtxIncrementChannel(void)
{
    vtxUpdateBandAndChannel(0, +1);
}

void vtxDecrementChannel(void)
{
    vtxUpdateBandAndChannel(0, -1);
}

uint8_t numPowerRules = 5;
void vtxUpdateActivatedChannel(void)
{
    // if (ARMING_FLAG(ARMED)) {
    //     locked = 1;
    // }

    if (vtxCommonDevice()) {
        static uint8_t lastIndex = -1;

        for (uint8_t index = 0; index < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; index++) {
            const vtxChannelActivationCondition_t *vtxChannelActivationCondition = &vtxConfig()->vtxChannelActivationConditions[index];
            //Added logic to disable non-power rules when VTX sync is disabled
            if((index >= numPowerRules) && (vtxConfig()->useVTXSync == false)) {
                break;
            }
            if (isRangeActive(vtxChannelActivationCondition->auxChannelIndex, &vtxChannelActivationCondition->range)
                && index != lastIndex) {
                lastIndex = index;

                if (!locked) {
                    if (vtxChannelActivationCondition->band > 0) {
                        vtxSettingsConfigMutable()->band = vtxChannelActivationCondition->band;
                    }
                    if (vtxChannelActivationCondition->channel > 0) {
                        vtxSettingsConfigMutable()->channel = vtxChannelActivationCondition->channel;
                    }
                }

                if (vtxChannelActivationCondition->power > 0) {
                    vtxSettingsConfigMutable()->power = vtxChannelActivationCondition->power;
                }
                break;
            }
        }
    }
}

void vtxCycleBandOrChannel(const uint8_t bandStep, const uint8_t channelStep)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        uint8_t band = 0, channel = 0;

        const bool haveAllNeededInfo = vtxCommonGetBandAndChannel(vtxDevice, &band, &channel);
        if (!haveAllNeededInfo) {
            return;
        }

        int newChannel = channel + channelStep;
        if (newChannel > vtxTableChannelCount) {
            newChannel = 1;
        } else if (newChannel < 1) {
            newChannel = vtxTableChannelCount;
        }

        int newBand = band + bandStep;
        if (newBand > vtxTableBandCount) {
            newBand = 1;
        } else if (newBand < 1) {
            newBand = vtxTableBandCount;
        }

        vtxSettingsConfigMutable()->band = newBand;
        vtxSettingsConfigMutable()->channel = newChannel;
    }
}

void vtxCyclePower(const uint8_t powerStep)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        uint8_t power = 0;
        const bool haveAllNeededInfo = vtxCommonGetPowerIndex(vtxDevice, &power);
        if (!haveAllNeededInfo) {
            return;
        }

        int newPower = power + powerStep;
        if (newPower >= vtxTablePowerLevels) {
            newPower = 1;
        } else if (newPower < 0) {
            newPower = vtxTablePowerLevels;
        }

        vtxSettingsConfigMutable()->power = newPower;
    }
}

/**
 * Allow VTX channel/band/rf power/on-off and save via a single button.
 *
 * The LED1 flashes a set number of times, followed by a short pause, one per second.  The amount of flashes decreases over time while
 * the button is held to indicate the action that will be performed upon release.
 * The actions are ordered by most-frequently used action.  i.e. you change channel more frequently than band.
 *
 * The vtx settings can be changed while the VTX is OFF.
 * If the VTX is OFF when the settings are saved the VTX will be OFF on the next boot, likewise
 * If the VTX is ON when the settings are saved the VTX will be ON on the next boot.
 *
 * Future: It would be nice to re-use the code in statusindicator.c and blink-codes but target a different LED instead of the simple timed
 * behaviour of the LED1 here.
 *
 * Future: Blink out the state after changing it.
 */
void handleVTXControlButton(void)
{
#if defined(USE_VTX_RTC6705) && defined(BUTTON_A_PIN)
    bool buttonWasPressed = false;
    const timeMs_t start = millis();
    timeMs_t ledToggleAt = start;
    bool ledEnabled = false;
    uint8_t flashesDone = 0;

    uint8_t actionCounter = 0;
    bool buttonHeld;
    while ((buttonHeld = buttonAPressed())) {
        const timeMs_t end = millis();

        int32_t diff = cmp32(end, start);
        if (diff > 25 && diff <= 1000) {
            actionCounter = 4;
        } else if (diff > 1000 && diff <= 3000) {
            actionCounter = 3;
        } else if (diff > 3000 && diff <= 5000) {
            actionCounter = 2;
        } else if (diff > 5000) {
            actionCounter = 1;
        }

        if (actionCounter) {

            diff = cmp32(ledToggleAt, end);

            if (diff < 0) {
                ledEnabled = !ledEnabled;

                const uint8_t updateDuration = 60;

                ledToggleAt = end + updateDuration;

                if (ledEnabled) {
                    LED1_ON;
                } else {
                    LED1_OFF;
                    flashesDone++;
                }

                if (flashesDone == actionCounter) {
                    ledToggleAt += (1000 - ((flashesDone * updateDuration) * 2));
                    flashesDone = 0;
                }
            }
            buttonWasPressed = true;
        }
    }

    if (!buttonWasPressed) {
        return;
    }

    LED1_OFF;

    switch (actionCounter) {
    case 4:
        vtxCycleBandOrChannel(0, +1);
        break;
    case 3:
        vtxCycleBandOrChannel(+1, 0);
        break;
    case 2:
        vtxCyclePower(+1);
        break;
    case 1:
        saveConfigAndNotify();
        break;
    }
#endif
}

#endif

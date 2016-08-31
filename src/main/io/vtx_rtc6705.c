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

#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/vtx_rtc6705.h"

#include "io/vtx.h"

// FIXME see boot.c, there is potential for mismatched defaults resulting in the button actions having no effect until pressed multiple times.
static uint8_t vtxChannel = RTC6705_CHANNEL_MIN;   // FIXME sync with boot.c
static uint8_t vtxBand = RTC6705_BAND_MIN;         // FIXME sync with boot.c
static uint8_t vtxRFPower = RTC6705_RF_POWER_MIN;  // FIXME sync with boot.c
static uint8_t vtxEnabled = false;                 // FIXME sync with boot.c

void vtxCycleChannel(void)
{
    vtxChannel++;
    if (vtxChannel > RTC6705_CHANNEL_MAX) {
        vtxChannel = RTC6705_CHANNEL_MIN;
    }
    rtc6705SetChannel(vtxBand, vtxChannel);
}

void vtxCycleBand(void)
{
    vtxBand++;
    if (vtxBand > RTC6705_BAND_MAX) {
        vtxBand = RTC6705_BAND_MIN;
    }
    rtc6705SetChannel(vtxBand, vtxChannel);
}

void vtxCycleRFPower(void)
{
    vtxRFPower++;
    if (vtxRFPower > RTC6705_RF_POWER_MAX) {
        vtxRFPower = RTC6705_RF_POWER_MIN;
    }
    rtc6705SetRFPower(vtxRFPower);
}

void vtxTogglePower(void) {
    vtxEnabled = !vtxEnabled;
    if (vtxEnabled) {
        rtc6705Enable();

        delay(RTC6705_BOOT_DELAY);

        rtc6705SetRFPower(vtxRFPower);
        rtc6705SetChannel(vtxBand, vtxChannel);
    } else {
        rtc6705Disable();
    }
}


/**
 * Allow VTX channel/band/rf power/on-off control via a single button.
 *
 * The LED1 flashes fast and progressively slower the longer a button is held to indicate the action that will be performed.
 * The actions are ordered by most-frequently used action.  i.e. you change channel more frequently than band.
 *
 * Future: It would be nice to re-use the code in statusindicator.c and blink-codes but target a different LED instead of the simple timed
 * behaviour of the LED1 here.
 */
void handleVTXControlButton(void)
{
#if defined(VTX) && defined(BUTTON_A_PIN)
    bool buttonHeld;
    bool buttonWasPressed = false;
    uint32_t start = millis();
    uint32_t ledToggleAt = start;
    bool ledEnabled = false;

    uint8_t actionCounter = 0;
    while ((buttonHeld = !digitalIn(BUTTON_A_PORT, BUTTON_A_PIN))) {
        uint32_t end = millis();

        int32_t diff = cmp32(end, start);
        if (diff > 25 && diff <= 1000) {
            actionCounter = 1;
        } else if (diff > 1000 && diff <= 3000) {
            actionCounter = 2;
        } else if (diff > 3000 && diff <= 5000) {
            actionCounter = 3;
        } else if (diff > 5000) {
            actionCounter = 4;
        }

        if (actionCounter) {

            diff = cmp32(ledToggleAt, end);

            if (diff < 0) {
                ledEnabled = !ledEnabled;

                if (ledEnabled) {
                    LED1_ON;
                } else {
                    LED1_OFF;
                }
                ledToggleAt = end + (100 * actionCounter);
            }
            buttonWasPressed = true;
        }
    }

    if (!buttonWasPressed) {
        return;
    }

    LED1_OFF;

    switch(actionCounter) {
        case 1:
            vtxCycleChannel();
            break;
        case 2:
            vtxCycleBand();
            break;
        case 3:
            vtxCycleRFPower();
            break;
        case 4:
            vtxTogglePower();
            break;
    }
#endif
}

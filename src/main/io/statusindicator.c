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

#include "platform.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"

#include "statusindicator.h"

void blinkLedAndSoundBeeper(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;

    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LED0_TOGGLE;            // switch LEDPIN state
            BEEP_ON;
            delay(wait);
            BEEP_OFF;
        }
        delay(60);
    }
}


static uint32_t warningLedTimer = 0;

void enableWarningLed(uint32_t currentTime)
{
    if (warningLedTimer != 0) {
        return; // already enabled
    }
    warningLedTimer = currentTime + 500000;
    LED0_ON;
}

void disableWarningLed(void)
{
    warningLedTimer = 0;
    LED0_OFF;
}

void updateWarningLed(uint32_t currentTime)
{
    if (warningLedTimer && (int32_t)(currentTime - warningLedTimer) >= 0) {
        LED0_TOGGLE;
        warningLedTimer = warningLedTimer + 500000;
    }
}


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

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"

#include "statusindicator.h"

static uint32_t warningLedTimer = 0;
static uint32_t blinkMask = 0;
static uint32_t nextBlinkMask = 0;

void warningLedRefresh(void)
{
    if (blinkMask <= 1) {  // skip interval for terminator bit, allow continuous on
        blinkMask = nextBlinkMask;
    }

    if (blinkMask) {
        if (blinkMask & 1)
            LED0_ON;
        else
            LED0_OFF;
        blinkMask >>= 1;
    }
}

void warningLedBeeper(bool on)
{
    if (!blinkMask) {
        if (on) {
            LED0_ON;
        } else {
            LED0_OFF;
        }
    }
}

void warningLedPulse(void)
{
	// FIXME this is not really pulse now - original code pulsed the light once a second (on short, off long), this code is a result of rebasing onto commit beac0a35cec690f33a2ef5d13f41f3e3a9d8e57a
	nextBlinkMask = 0b10000000;
}

void warningLedSetBlinkMask(uint32_t newBlinkMask)
{
	nextBlinkMask = newBlinkMask;
}

void warningLedUpdate(void)
{
    uint32_t now = micros();
    if ((int32_t)(now - warningLedTimer) > 0) {
        warningLedRefresh();
        warningLedTimer = now + WARNING_LED_BLINK_DELAY;
    }
}


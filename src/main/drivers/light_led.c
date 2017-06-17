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

#include "drivers/io.h"
#include "io_impl.h"

#include "light_led.h"

static IO_t leds[STATUS_LED_NUMBER];
static uint8_t ledInversion = 0;

void ledInit(const statusLedConfig_t *statusLedConfig)
{
    ledInversion = statusLedConfig->inversion;
    for (int i = 0; i < STATUS_LED_NUMBER; i++) {
        if (statusLedConfig->ioTags[i]) {
            leds[i] = IOGetByTag(statusLedConfig->ioTags[i]);
            IOInit(leds[i], OWNER_LED, RESOURCE_INDEX(i));
            IOConfigGPIO(leds[i], IOCFG_OUT_PP);
        } else {
            leds[i] = IO_NONE;
        }
    }

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;
}

void ledToggle(int led)
{
    IOToggle(leds[led]);
}

void ledSet(int led, bool on)
{
    const bool inverted = (1 << (led)) & ledInversion;
    IOWrite(leds[led], on ? inverted : !inverted);
}

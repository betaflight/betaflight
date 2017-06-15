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

#include "config/parameter_group_ids.h"
#include "drivers/io.h"
#include "light_led.h"

static IO_t leds[LED_NUMBER];
static uint8_t ledPolarity = 0;

PG_REGISTER_WITH_RESET_FN(statusLedConfig_t, statusLedConfig, PG_STATUS_LED_CONFIG, 0);

#ifndef LED0
#define LED0 NONE
#endif

#ifndef LED1
#define LED1 NONE
#endif

#ifndef LED2
#define LED2 NONE
#endif

void pgResetFn_statusLedConfig(statusLedConfig_t *statusLedConfig)
{
#ifdef LED0
    statusLedConfig->ledTags[0] = IO_TAG(LED0);
#endif
#ifdef LED1
    statusLedConfig->ledTags[1] = IO_TAG(LED1);
#endif
#ifdef LED2
    statusLedConfig->ledTags[2] = IO_TAG(LED2);
#endif

    statusLedConfig->polarity = 0
#ifdef LED0_INVERTED
    | BIT(0)
#endif
#ifdef LED1_INVERTED
    | BIT(1)
#endif
#ifdef LED2_INVERTED
    | BIT(2)
#endif
    ;
}

void ledInit(const statusLedConfig_t *statusLedConfig)
{
    LED0_OFF;
    LED1_OFF;
    LED2_OFF;

    ledPolarity = statusLedConfig->polarity;
    for (int i = 0; i < LED_NUMBER; i++) {
        if (statusLedConfig->ledTags[i]) {
            leds[i] = IOGetByTag(statusLedConfig->ledTags[i]);
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
    const bool inverted = (1 << (led)) & ledPolarity;
    IOWrite(leds[led], on ? inverted : !inverted);
}

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

#include "platform.h"

#include "pg/pg_ids.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"

#include "light_led.h"

#if !defined(USE_VIRTUAL_LED)

static IO_t leds[STATUS_LED_COUNT];
static uint8_t ledInversion = 0;

PG_REGISTER_WITH_RESET_TEMPLATE(statusLedConfig_t, statusLedConfig, PG_STATUS_LED_CONFIG, 0);

PG_RESET_TEMPLATE(statusLedConfig_t, statusLedConfig,
    .ioTags = {
#if STATUS_LED_COUNT > 0 && defined(LED0_PIN)
        [0] = IO_TAG(LED0_PIN),
#endif
#if STATUS_LED_COUNT > 1 && defined(LED1_PIN)
        [1] = IO_TAG(LED1_PIN),
#endif
#if STATUS_LED_COUNT > 2 && defined(LED2_PIN)
        [2] = IO_TAG(LED2_PIN),
#endif
    },
    .inversion = 0
#ifdef LED0_INVERTED
    | BIT(0)
#endif
#ifdef LED1_INVERTED
    | BIT(1)
#endif
#ifdef LED2_INVERTED
    | BIT(2)
#endif
    ,
);

void ledInit(const statusLedConfig_t *statusLedConfig)
{
    ledInversion = statusLedConfig->inversion;
    for (int i = 0; i < (int)ARRAYLEN(leds); i++) {
        leds[i] = IOGetByTag(statusLedConfig->ioTags[i]);
        if (leds[i]) {
            IOInit(leds[i], OWNER_LED, RESOURCE_INDEX(i));
            IOConfigGPIO(leds[i], IOCFG_OUT_PP);
        }
        ledSet(i, false);
    }
}

void ledToggle(int led)
{
    if (led < 0 || led >= (int)ARRAYLEN(leds)) {
        return;
    }
    IOToggle(leds[led]);
}

void ledSet(int led, bool on)
{
    if (led < 0 || led >= (int)ARRAYLEN(leds)) {
        return;
    }
    const bool inverted = ledInversion & (1 << led);
    IOWrite(leds[led], on ? inverted : !inverted);
}

#endif

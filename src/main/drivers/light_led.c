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

PG_REGISTER_WITH_RESET_FN(statusLedConfig_t, statusLedConfig, PG_STATUS_LED_CONFIG, 0);

static IO_t leds[STATUS_LED_NUMBER];
static uint8_t ledInversion = 0;

#ifndef LED0_PIN
#define LED0_PIN NONE
#endif

#ifndef LED1_PIN
#define LED1_PIN NONE
#endif

#ifndef LED2_PIN
#define LED2_PIN NONE
#endif

void pgResetFn_statusLedConfig(statusLedConfig_t *statusLedConfig)
{
    statusLedConfig->ioTags[0] = IO_TAG(LED0_PIN);
    statusLedConfig->ioTags[1] = IO_TAG(LED1_PIN);
    statusLedConfig->ioTags[2] = IO_TAG(LED2_PIN);

    statusLedConfig->inversion = 0
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

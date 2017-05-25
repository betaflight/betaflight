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

#include "drivers/light_led.h"

static const IO_t leds[] = {
#ifdef LED0
    DEFIO_IO(LED0),
#else
    DEFIO_IO(NONE),
#endif
#ifdef LED1
    DEFIO_IO(LED1),
#else
    DEFIO_IO(NONE),
#endif
#ifdef LED2
    DEFIO_IO(LED2),
#else
    DEFIO_IO(NONE),
#endif
#if defined(LED0_A) || defined(LED1_A) || defined(LED2_A)
#ifdef LED0_A
    DEFIO_IO(LED0_A),
#else
    DEFIO_IO(NONE),
#endif
#ifdef LED1_A
    DEFIO_IO(LED1_A),
#else
    DEFIO_IO(NONE),
#endif
#ifdef LED2_A
    DEFIO_IO(LED2_A),
#else
    DEFIO_IO(NONE),
#endif
#endif
};

uint8_t ledPolarity = 0
#ifdef LED0_INVERTED
    | BIT(0)
#endif
#ifdef LED1_INVERTED
    | BIT(1)
#endif
#ifdef LED2_INVERTED
    | BIT(2)
#endif
#ifdef LED0_A_INVERTED
    | BIT(3)
#endif
#ifdef LED1_A_INVERTED
    | BIT(4)
#endif
#ifdef LED2_A_INVERTED
    | BIT(5)
#endif
    ;

static uint8_t ledOffset = 0;

void ledInit(bool alternative_led)
{
#if defined(LED0_A) || defined(LED1_A) || defined(LED2_A)
    if (alternative_led) {
        ledOffset = LED_NUMBER;
    }
#else
    UNUSED(alternative_led);
#endif

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;

    for (int i = 0; i < LED_NUMBER; i++) {
        if (leds[i + ledOffset]) {
            IOInit(leds[i + ledOffset], OWNER_LED, RESOURCE_OUTPUT, RESOURCE_INDEX(i));
            IOConfigGPIO(leds[i + ledOffset], IOCFG_OUT_PP);
        }
    }

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;
}

void ledToggle(int led)
{
    IOToggle(leds[led + ledOffset]);
}

void ledSet(int led, bool on)
{
    const bool inverted = (1 << (led + ledOffset)) & ledPolarity;
    IOWrite(leds[led + ledOffset], on ? inverted : !inverted);
}

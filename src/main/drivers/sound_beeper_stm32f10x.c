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
#include <stdlib.h>

#include "platform.h"

#include "system.h"
#include "gpio.h"

#include "sound_beeper.h"

void initBeeperHardware(void)
{
#ifdef BEEPER
    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup = {
        .gpio = BEEP_GPIO,
        .cfg = { BEEP_PIN, Mode_Out_OD, Speed_2MHz }
    };

    RCC_APB2PeriphClockCmd(BEEP_PERIPHERAL, ENABLE);

#ifdef NAZE
    // Hack - naze rev4 and below used opendrain to PNP for buzzer. Rev5 and above use PP to NPN.

    if (hse_value == 12000000 && gpio_setup.cfg.mode == Mode_Out_OD)
        gpio_setup.cfg.mode = Mode_Out_PP;
#endif

    gpioInit(gpio_setup.gpio, &gpio_setup.cfg);

#endif
}

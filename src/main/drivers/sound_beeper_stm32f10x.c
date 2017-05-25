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

#include "build/build_config.h"


#include "drivers/time.h"
#include "drivers/gpio.h"

#include "sound_beeper.h"

void initBeeperHardware(beeperConfig_t *config)
{
#ifndef BEEPER
    UNUSED(config);
#else
    gpio_config_t gpioConfig = {
        config->gpioPin,
        config->gpioMode,
        Speed_2MHz
    };

    RCC_APB2PeriphClockCmd(config->gpioPeripheral, ENABLE);

    gpioInit(config->gpioPort, &gpioConfig);
#endif
}
